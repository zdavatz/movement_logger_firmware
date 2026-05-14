/**
  ******************************************************************************
  * @file    sd_fatfs.c
  * @brief   Minimal FAT32 append-only writer. Polled HAL_SD, no DMA, no ISR.
  *
  *          Layout we assume:
  *            - LBA 0 = MBR (with 1 active FAT32 partition) OR direct FAT32 BPB
  *            - FAT32 with 512-byte sectors, 8-bit cluster sizes (1..64
  *              sectors per cluster — typical 32 KB clusters on 4-64 GB cards)
  *            - One root cluster chain; we walk it as needed
  *
  *          Append strategy:
  *            - File handle keeps the current end cluster + byte offset.
  *            - Each Append() walks bytes through the current sector buffer;
  *              when the sector fills, we write it back to SD; when the
  *              cluster fills, we allocate the next free cluster from FAT,
  *              link it into the chain, and continue.
  *            - Flush() writes back the dirty sector and patches the dir
  *              entry's size + first cluster (low/high) fields so a
  *              hard-power-off still leaves a parseable file.
  *
  *          This is enough for Phase 3 (continuous CSV append). Phase 5
  *          will replace it with a real FatFs port that also does
  *          read/delete for BLE FileSync.
  ******************************************************************************
  */

#include "main.h"
#include "sd_fatfs.h"
#include <string.h>
#include <stdio.h>

/* ============================================================================
   SDMMC1 hardware setup (lifted from BSP, stripped of EXTI / NVIC) */

extern void Error_Handler(const char *file, int line);

static SD_HandleTypeDef g_hsd;
static int g_mounted = 0;

/* FAT32 geometry — populated by SDFat_Mount */
static uint32_t g_part_lba       = 0;     /* LBA of partition start */
static uint32_t g_bytes_per_sec  = 512;
static uint32_t g_sec_per_clus   = 0;
static uint32_t g_rsvd_secs      = 0;
static uint32_t g_num_fats       = 0;
static uint32_t g_fat_size_secs  = 0;
static uint32_t g_root_cluster   = 0;
static uint32_t g_fat_lba        = 0;     /* LBA of FAT #0 */
static uint32_t g_data_lba       = 0;     /* LBA where cluster 2 lives */
static uint64_t g_total_sectors  = 0;

/* Cached one-sector FAT page (write-through). */
static uint8_t  g_fatpage[512];
static uint32_t g_fatpage_lba    = 0;     /* 0 = invalid */
static int      g_fatpage_dirty  = 0;

/* Allocation hint: remember where the last successful fat_alloc_one stopped.
   Without this the scan starts at cluster 2 every time → quadratic cost as
   the file grows (Build #11 showed 2-3 s stalls at every cluster boundary,
   most of the wall-clock budget). With the hint, each allocation is O(1)
   on a freshly-formatted card so the cluster-boundary "stutter" drops from
   ~2.5 s to a single FAT-page write (~10-15 ms). */
static uint32_t g_alloc_hint     = 2;

/* General-purpose scratch sector for directory scans. */
static uint8_t  g_scratch[512];

static HAL_StatusTypeDef sd_read_block(uint32_t lba, uint8_t *buf)
{
  /* HAL_SD_ReadBlocks signature is (hsd, buf, BlockAdd, NumberOfBlocks, Timeout) */
  return HAL_SD_ReadBlocks(&g_hsd, buf, lba, 1, 1000);
}

static HAL_StatusTypeDef sd_write_block(uint32_t lba, const uint8_t *buf)
{
  return HAL_SD_WriteBlocks(&g_hsd, buf, lba, 1, 1000);
}

/* Wait for the card to leave HAL_SD_CARD_PROGRAMMING / RECEIVING after a
   write. Cheap busy-loop, bounded ~1 s. */
static void sd_wait_idle(void)
{
  uint32_t deadline = HAL_GetTick() + 1000U;
  while (HAL_SD_GetCardState(&g_hsd) != HAL_SD_CARD_TRANSFER) {
    if ((int32_t)(deadline - HAL_GetTick()) <= 0) return;
  }
}

/* Read a 16-bit LE word from buf at offset off. */
static inline uint16_t rd16(const uint8_t *b, uint32_t off) {
  return (uint16_t)(b[off] | (b[off+1] << 8));
}
static inline uint32_t rd32(const uint8_t *b, uint32_t off) {
  return (uint32_t)b[off] | ((uint32_t)b[off+1] << 8) |
         ((uint32_t)b[off+2] << 16) | ((uint32_t)b[off+3] << 24);
}
static inline void wr16(uint8_t *b, uint32_t off, uint16_t v) {
  b[off]   = (uint8_t)(v & 0xFF);
  b[off+1] = (uint8_t)(v >> 8);
}
static inline void wr32(uint8_t *b, uint32_t off, uint32_t v) {
  b[off]   = (uint8_t)(v & 0xFF);
  b[off+1] = (uint8_t)(v >> 8);
  b[off+2] = (uint8_t)(v >> 16);
  b[off+3] = (uint8_t)(v >> 24);
}

/* ============================================================================
   FAT page cache + cluster chain walking */

static pl_fx_status_t fat_load_page_for_cluster(uint32_t cluster, uint32_t *page_off)
{
  uint32_t fat_byte_off = cluster * 4U;
  uint32_t page_lba     = g_fat_lba + (fat_byte_off / 512U);
  *page_off             = fat_byte_off % 512U;
  if (g_fatpage_lba == page_lba) return PL_FX_OK;

  if (g_fatpage_dirty && g_fatpage_lba) {
    /* Write back current page AND the corresponding entry in FAT #1 (so the
       two FATs stay in sync after every modification — minimal but correct). */
    if (sd_write_block(g_fatpage_lba, g_fatpage) != HAL_OK) return PL_FX_ERR_IO;
    uint32_t alt = g_fatpage_lba + g_fat_size_secs;
    (void)sd_write_block(alt, g_fatpage);
    sd_wait_idle();
    g_fatpage_dirty = 0;
  }

  if (sd_read_block(page_lba, g_fatpage) != HAL_OK) return PL_FX_ERR_IO;
  g_fatpage_lba = page_lba;
  return PL_FX_OK;
}

static pl_fx_status_t fat_get_next(uint32_t cluster, uint32_t *next_out)
{
  uint32_t po;
  pl_fx_status_t s = fat_load_page_for_cluster(cluster, &po);
  if (s != PL_FX_OK) return s;
  uint32_t v = rd32(g_fatpage, po) & 0x0FFFFFFFu;
  *next_out = v;
  return PL_FX_OK;
}

static pl_fx_status_t fat_set_next(uint32_t cluster, uint32_t next_value)
{
  uint32_t po;
  pl_fx_status_t s = fat_load_page_for_cluster(cluster, &po);
  if (s != PL_FX_OK) return s;
  uint32_t old = rd32(g_fatpage, po) & 0xF0000000u;
  wr32(g_fatpage, po, old | (next_value & 0x0FFFFFFFu));
  g_fatpage_dirty = 1;
  return PL_FX_OK;
}

static pl_fx_status_t fat_flush_page(void)
{
  if (!g_fatpage_dirty || !g_fatpage_lba) return PL_FX_OK;
  if (sd_write_block(g_fatpage_lba, g_fatpage) != HAL_OK) return PL_FX_ERR_IO;
  uint32_t alt = g_fatpage_lba + g_fat_size_secs;
  (void)sd_write_block(alt, g_fatpage);
  sd_wait_idle();
  g_fatpage_dirty = 0;
  return PL_FX_OK;
}

/* Walk the free-cluster scan from cluster 2 upward, return the first
   cluster with FAT entry == 0. -1 if disk full. Marks it as EOC on success. */
static uint32_t fat_alloc_one(void)
{
  uint32_t max_clusters = (uint32_t)((g_total_sectors > 0 ? g_total_sectors : (1ULL<<31)) / g_sec_per_clus);
  if (max_clusters > 0x0FFFFFF0u) max_clusters = 0x0FFFFFF0u;
  if (g_alloc_hint < 2) g_alloc_hint = 2;

  /* Scan forward from the hint first — for a session writing into a fresh
     unused area, every allocation is O(1) (find first hole, advance hint).
     If we walk off the end without finding anything (rare; only on a near-
     full card), fall back to a full scan from cluster 2. */
  for (uint32_t c = g_alloc_hint; c < max_clusters; c++) {
    uint32_t next;
    if (fat_get_next(c, &next) != PL_FX_OK) return 0;
    if (next == 0) {
      if (fat_set_next(c, 0x0FFFFFFFu) != PL_FX_OK) return 0;
      g_alloc_hint = c + 1;
      return c;
    }
  }
  for (uint32_t c = 2; c < g_alloc_hint; c++) {
    uint32_t next;
    if (fat_get_next(c, &next) != PL_FX_OK) return 0;
    if (next == 0) {
      if (fat_set_next(c, 0x0FFFFFFFu) != PL_FX_OK) return 0;
      g_alloc_hint = c + 1;
      return c;
    }
  }
  return 0;
}

static inline uint32_t cluster_to_lba(uint32_t cluster) {
  return g_data_lba + (cluster - 2u) * g_sec_per_clus;
}

/* ============================================================================
   Mount: read MBR + BPB, populate g_* globals */

static pl_fx_status_t sd_hw_init(void)
{
  /* Enable HSI48 + SDMMC clock */
  RCC_OscInitTypeDef osc = {0};
  HAL_RCC_GetOscConfig(&osc);
  if (osc.HSI48State != RCC_HSI48_ON) {
    osc.OscillatorType = RCC_OSCILLATORTYPE_HSI48;
    osc.HSI48State     = RCC_HSI48_ON;
    osc.PLL.PLLState   = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&osc) != HAL_OK) return PL_FX_ERR_INIT;
  }

  RCC_PeriphCLKInitTypeDef pc = {0};
  pc.PeriphClockSelection = RCC_PERIPHCLK_SDMMC;
  pc.SdmmcClockSelection  = RCC_SDMMCCLKSOURCE_CLK48;
  if (HAL_RCCEx_PeriphCLKConfig(&pc) != HAL_OK) return PL_FX_ERR_INIT;

  __HAL_RCC_SDMMC1_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();

  /* Enable + voltage-select pins */
  GPIO_InitTypeDef g = {0};
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin   = PL_SD_ENABLE_PIN | PL_SD_SEL_V_PIN;
  HAL_GPIO_Init(PL_SD_ENABLE_PORT, &g);
  HAL_GPIO_WritePin(PL_SD_ENABLE_PORT, PL_SD_ENABLE_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(PL_SD_SEL_V_PORT,  PL_SD_SEL_V_PIN,  GPIO_PIN_RESET);

  /* SDMMC1 GPIO — PC8..12 (CMD,D0..3) on AF12, PC6/PC7 + PB8/PB9 transceiver on AF8 */
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_PULLUP;
  g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
  g.Alternate = GPIO_AF12_SDMMC1;
  g.Pin       = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
  HAL_GPIO_Init(GPIOC, &g);

  g.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &g);

  g.Pin       = GPIO_PIN_7 | GPIO_PIN_6;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF8_SDMMC1;
  HAL_GPIO_Init(GPIOC, &g);

  g.Pin = GPIO_PIN_9 | GPIO_PIN_8;
  HAL_GPIO_Init(GPIOB, &g);

  /* Note: no NVIC enable — we use polled HAL_SD_*Blocks(). */

  memset(&g_hsd, 0, sizeof(g_hsd));
  g_hsd.Instance                 = SDMMC1;
  g_hsd.Init.ClockEdge           = SDMMC_CLOCK_EDGE_RISING;
  g_hsd.Init.ClockPowerSave      = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  g_hsd.Init.BusWide             = SDMMC_BUS_WIDE_4B;
  g_hsd.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  /* HSpeed CLK_DIV — same constant the BSP uses on this board */
  g_hsd.Init.ClockDiv            = SDMMC_HSpeed_CLK_DIV;

  if (HAL_SD_Init(&g_hsd) != HAL_OK) return PL_FX_ERR_INIT;
  return PL_FX_OK;
}

pl_fx_status_t SDFat_Mount(void)
{
  pl_fx_status_t s = sd_hw_init();
  if (s != PL_FX_OK) return s;

  /* Read LBA 0; could be MBR or volume boot sector. */
  uint8_t boot[512];
  if (sd_read_block(0, boot) != HAL_OK) return PL_FX_ERR_IO;

  uint32_t bpb_lba = 0;
  /* MBR signature is at offset 510 (0x55 0xAA). If first byte == 0xEB or 0xE9
     it's already a BPB; otherwise it's an MBR. */
  if ((boot[0] == 0xEB || boot[0] == 0xE9) &&
      boot[510] == 0x55 && boot[511] == 0xAA) {
    bpb_lba = 0;
  } else if (boot[510] == 0x55 && boot[511] == 0xAA) {
    /* Take first partition entry. Offset 446, 16 bytes per entry. */
    bpb_lba = rd32(boot, 446 + 8);
    if (sd_read_block(bpb_lba, boot) != HAL_OK) return PL_FX_ERR_IO;
  } else {
    return PL_FX_ERR_NO_FAT32;
  }
  if (boot[510] != 0x55 || boot[511] != 0xAA) return PL_FX_ERR_NO_FAT32;

  g_part_lba       = bpb_lba;
  g_bytes_per_sec  = rd16(boot, 11);
  g_sec_per_clus   = boot[13];
  g_rsvd_secs      = rd16(boot, 14);
  g_num_fats       = boot[16];
  uint32_t fat_sz16= rd16(boot, 22);
  uint32_t fat_sz32= rd32(boot, 36);
  g_fat_size_secs  = (fat_sz16 != 0) ? fat_sz16 : fat_sz32;
  g_root_cluster   = rd32(boot, 44);
  g_total_sectors  = (rd16(boot, 19) != 0) ? rd16(boot, 19) : rd32(boot, 32);

  if (g_bytes_per_sec != 512 || g_sec_per_clus == 0 || g_root_cluster < 2)
    return PL_FX_ERR_NO_FAT32;

  g_fat_lba  = bpb_lba + g_rsvd_secs;
  g_data_lba = g_fat_lba + g_num_fats * g_fat_size_secs;

  g_fatpage_lba   = 0;
  g_fatpage_dirty = 0;
  g_alloc_hint    = 2;     /* reset across mount so each session starts fresh */
  g_mounted       = 1;
  return PL_FX_OK;
}

int SDFat_IsMounted(void) { return g_mounted; }

uint64_t SDFat_FreeBytes(void)
{
  if (!g_mounted) return 0;
  /* Coarse estimate: scan FAT looking for zero entries. Bound scan time by
     reading only the first 64 KB of the FAT (covers cards up to ~8 GB with
     32 KB clusters = ~256k clusters per 1 MB of FAT page). Good enough for
     the heartbeat-log "free=…" field. */
  uint32_t scan_secs = (g_fat_size_secs < 128U) ? g_fat_size_secs : 128U;
  uint32_t free_cl = 0;
  for (uint32_t s = 0; s < scan_secs; s++) {
    if (sd_read_block(g_fat_lba + s, g_scratch) != HAL_OK) break;
    for (int i = 0; i < 512; i += 4) {
      uint32_t v = rd32(g_scratch, i) & 0x0FFFFFFFu;
      if (v == 0) free_cl++;
    }
  }
  /* extrapolate to the full FAT */
  if (scan_secs > 0 && scan_secs < g_fat_size_secs) {
    free_cl = (uint32_t)(((uint64_t)free_cl * g_fat_size_secs) / scan_secs);
  }
  return (uint64_t)free_cl * g_sec_per_clus * 512u;
}

/* ============================================================================
   Directory scanning + entry creation */

typedef struct {
  char     name83[11];
  uint32_t first_cluster;
  uint32_t size;
  uint32_t dir_block;
  uint16_t dir_idx;     /* 0..15 */
} dir_entry_t;

/* Format a regular ASCII "NAME.EXT" into 11-byte 8.3 padded with spaces. */
static void to_83(const char *src, char out[11])
{
  memset(out, ' ', 11);
  int i = 0, j = 0;
  while (src[i] && src[i] != '.' && j < 8) {
    char c = src[i++];
    if (c >= 'a' && c <= 'z') c -= 32;
    out[j++] = c;
  }
  if (src[i] == '.') i++;
  int k = 0;
  while (src[i] && k < 3) {
    char c = src[i++];
    if (c >= 'a' && c <= 'z') c -= 32;
    out[8 + k++] = c;
  }
}

/* Walk the root directory; for every valid entry call cb(entry, user).
   cb returns 0 to continue, 1 to stop. */
typedef int (*dir_cb_t)(const dir_entry_t *, void *);
static pl_fx_status_t walk_root(dir_cb_t cb, void *user)
{
  uint32_t cluster = g_root_cluster;
  while (cluster >= 2 && cluster < 0x0FFFFFF8u) {
    uint32_t lba = cluster_to_lba(cluster);
    for (uint32_t s = 0; s < g_sec_per_clus; s++) {
      if (sd_read_block(lba + s, g_scratch) != HAL_OK) return PL_FX_ERR_IO;
      for (int i = 0; i < 512; i += 32) {
        uint8_t first = g_scratch[i];
        if (first == 0x00) return PL_FX_OK;       /* end of dir */
        if (first == 0xE5) continue;              /* deleted */
        uint8_t attr = g_scratch[i + 11];
        if (attr == 0x0F) continue;               /* LFN */
        if (attr & 0x08) continue;                /* volume label */
        dir_entry_t e;
        memcpy(e.name83, &g_scratch[i], 11);
        uint16_t hi = rd16(g_scratch, i + 20);
        uint16_t lo = rd16(g_scratch, i + 26);
        e.first_cluster = ((uint32_t)hi << 16) | lo;
        e.size          = rd32(g_scratch, i + 28);
        e.dir_block     = lba + s;
        e.dir_idx       = (uint16_t)(i / 32);
        if (cb(&e, user)) return PL_FX_OK;
      }
    }
    uint32_t next;
    if (fat_get_next(cluster, &next) != PL_FX_OK) return PL_FX_ERR_IO;
    cluster = next;
  }
  return PL_FX_OK;
}

typedef struct {
  const char *name83;
  dir_entry_t found;
  uint8_t     ok;
} find_ctx_t;

static int find_cb(const dir_entry_t *e, void *user)
{
  find_ctx_t *c = (find_ctx_t *)user;
  if (memcmp(e->name83, c->name83, 11) == 0) {
    c->found = *e;
    c->ok    = 1;
    return 1;
  }
  return 0;
}

static int exists_83(const char name83[11])
{
  find_ctx_t c = { .name83 = name83, .ok = 0 };
  walk_root(find_cb, &c);
  return c.ok;
}

int SDFat_NextSessionNumber(void)
{
  if (!g_mounted) return -1;
  for (int n = 0; n < 1000; n++) {
    char name[16]; char n83[11];
    /* SENSNNN.CSV */
    snprintf(name, sizeof(name), "SENS%03d.CSV", n);
    to_83(name, n83);
    if (exists_83(n83)) continue;
    snprintf(name, sizeof(name), "GPS%03d.CSV", n);
    to_83(name, n83);
    if (exists_83(n83)) continue;
    snprintf(name, sizeof(name), "BAT%03d.CSV", n);
    to_83(name, n83);
    if (exists_83(n83)) continue;
    return n;
  }
  return -1;
}

/* Allocate the first cluster + create a directory entry. Returns the entry's
   sector/idx + the chosen first cluster. */
static pl_fx_status_t create_file_entry(const char name83[11],
                                        uint32_t *first_cluster_out,
                                        uint32_t *dir_block_out,
                                        uint16_t *dir_idx_out)
{
  uint32_t cluster = g_root_cluster;
  while (cluster >= 2 && cluster < 0x0FFFFFF8u) {
    uint32_t lba = cluster_to_lba(cluster);
    for (uint32_t s = 0; s < g_sec_per_clus; s++) {
      if (sd_read_block(lba + s, g_scratch) != HAL_OK) return PL_FX_ERR_IO;
      for (int i = 0; i < 512; i += 32) {
        uint8_t f = g_scratch[i];
        if (f == 0x00 || f == 0xE5) {
          /* Allocate first cluster for the new file. */
          uint32_t fc = fat_alloc_one();
          if (fc == 0) return PL_FX_ERR_FULL;

          memset(&g_scratch[i], 0, 32);
          memcpy(&g_scratch[i], name83, 11);
          g_scratch[i + 11] = 0x20;             /* attr = archive */
          /* Set first cluster low/high. */
          wr16(g_scratch, i + 20, (uint16_t)(fc >> 16));
          wr16(g_scratch, i + 26, (uint16_t)(fc & 0xFFFF));
          wr32(g_scratch, i + 28, 0);

          /* If the slot we used was 0x00 (end-of-dir), advance the marker by
             zero-clearing the next slot if there is one in this sector. */
          if (f == 0x00 && (i + 32) < 512) {
            g_scratch[i + 32] = 0x00;
          }

          if (sd_write_block(lba + s, g_scratch) != HAL_OK) return PL_FX_ERR_IO;
          sd_wait_idle();

          /* Zero out the first cluster's first sector so the file body starts
             empty (avoids garbage from a previous deleted file). */
          memset(g_scratch, 0, 512);
          uint32_t flba = cluster_to_lba(fc);
          (void)sd_write_block(flba, g_scratch);
          sd_wait_idle();
          /* Flush FAT page in case fat_alloc_one left it dirty. */
          fat_flush_page();

          *first_cluster_out = fc;
          *dir_block_out     = lba + s;
          *dir_idx_out       = (uint16_t)(i / 32);
          return PL_FX_OK;
        }
      }
    }
    uint32_t next;
    if (fat_get_next(cluster, &next) != PL_FX_OK) return PL_FX_ERR_IO;
    cluster = next;
  }
  /* TODO: extend root directory cluster chain — needed only for very full SDs */
  return PL_FX_ERR_FULL;
}

pl_fx_status_t SDFat_OpenAppend(PL_File *out, const char *name)
{
  if (!g_mounted || !out || !name) return PL_FX_ERR_INVAL;
  memset(out, 0, sizeof(*out));
  to_83(name, out->name83);

  find_ctx_t c = { .name83 = out->name83, .ok = 0 };
  walk_root(find_cb, &c);

  uint32_t first_cl;
  if (c.ok) {
    first_cl       = c.found.first_cluster;
    out->size      = c.found.size;
    out->dir_block = c.found.dir_block;
    out->dir_entry_idx = c.found.dir_idx;
  } else {
    pl_fx_status_t s = create_file_entry(out->name83, &first_cl,
                                         &out->dir_block, &out->dir_entry_idx);
    if (s != PL_FX_OK) return s;
    out->size = 0;
  }
  out->first_cluster = first_cl;

  /* Walk to the end of the chain to set cur_cluster + cur_offset. */
  uint32_t cluster = first_cl;
  uint32_t cluster_bytes = g_sec_per_clus * 512u;
  uint32_t remaining = out->size;
  for (;;) {
    if (remaining <= cluster_bytes) {
      out->cur_cluster = cluster;
      out->cur_offset  = remaining;
      break;
    }
    uint32_t next;
    if (fat_get_next(cluster, &next) != PL_FX_OK) return PL_FX_ERR_IO;
    if (next >= 0x0FFFFFF8u || next < 2) {
      /* size > chain length — corrupt. Reset to chain end. */
      out->cur_cluster = cluster;
      out->cur_offset  = cluster_bytes;  /* will allocate next cluster on first write */
      break;
    }
    cluster    = next;
    remaining -= cluster_bytes;
  }

  /* Preload the partial-end sector if needed. */
  out->sector_dirty = 0;
  out->sector_lba   = 0;
  if (out->cur_offset > 0 && out->cur_offset < g_sec_per_clus * 512u) {
    uint32_t sec_in_cl = out->cur_offset / 512u;
    uint32_t sec_off   = out->cur_offset % 512u;
    if (sec_off != 0) {
      uint32_t lba = cluster_to_lba(out->cur_cluster) + sec_in_cl;
      if (sd_read_block(lba, out->sector_buf) != HAL_OK) return PL_FX_ERR_IO;
      out->sector_lba = lba;
    }
  }

  out->used = 1;
  return PL_FX_OK;
}

/* Internal: ensure the sector buffer corresponds to the writeable LBA at
   (cur_cluster, cur_offset). Allocates a new cluster if cur_offset hit
   cluster boundary. */
static pl_fx_status_t advance_to_writable(PL_File *f)
{
  uint32_t cluster_bytes = g_sec_per_clus * 512u;
  if (f->cur_offset >= cluster_bytes) {
    /* Allocate next cluster + link */
    uint32_t fc = fat_alloc_one();
    if (fc == 0) return PL_FX_ERR_FULL;
    if (fat_set_next(f->cur_cluster, fc) != PL_FX_OK) return PL_FX_ERR_IO;
    fat_flush_page();
    f->cur_cluster = fc;
    f->cur_offset  = 0;
    f->sector_lba  = 0;
    f->sector_dirty= 0;
  }
  uint32_t sec_in_cl = f->cur_offset / 512u;
  uint32_t sec_off   = f->cur_offset % 512u;
  uint32_t target_lba = cluster_to_lba(f->cur_cluster) + sec_in_cl;

  if (f->sector_lba != target_lba) {
    /* Flush previous sector if dirty. */
    if (f->sector_dirty && f->sector_lba) {
      if (sd_write_block(f->sector_lba, f->sector_buf) != HAL_OK) return PL_FX_ERR_IO;
      sd_wait_idle();
      f->sector_dirty = 0;
    }
    if (sec_off != 0) {
      /* mid-sector — read first */
      if (sd_read_block(target_lba, f->sector_buf) != HAL_OK) return PL_FX_ERR_IO;
    } else {
      memset(f->sector_buf, 0, 512);
    }
    f->sector_lba = target_lba;
  }
  return PL_FX_OK;
}

pl_fx_status_t SDFat_Append(PL_File *f, const void *data, uint32_t len)
{
  if (!f || !f->used) return PL_FX_ERR_INVAL;
  const uint8_t *p = (const uint8_t *)data;

  while (len > 0) {
    pl_fx_status_t s = advance_to_writable(f);
    if (s != PL_FX_OK) return s;

    uint32_t sec_off = f->cur_offset % 512u;
    uint32_t room    = 512u - sec_off;
    uint32_t take    = (len < room) ? len : room;

    memcpy(&f->sector_buf[sec_off], p, take);
    f->sector_dirty = 1;
    p              += take;
    len            -= take;
    f->cur_offset  += take;
    f->size        += take;

    if ((f->cur_offset % 512u) == 0) {
      /* Sector full → write back. */
      if (sd_write_block(f->sector_lba, f->sector_buf) != HAL_OK) return PL_FX_ERR_IO;
      sd_wait_idle();
      f->sector_dirty = 0;
    }
  }
  return PL_FX_OK;
}

static pl_fx_status_t patch_dir_entry(PL_File *f)
{
  if (sd_read_block(f->dir_block, g_scratch) != HAL_OK) return PL_FX_ERR_IO;
  uint32_t off = (uint32_t)f->dir_entry_idx * 32u;
  wr16(g_scratch, off + 20, (uint16_t)(f->first_cluster >> 16));
  wr16(g_scratch, off + 26, (uint16_t)(f->first_cluster & 0xFFFF));
  wr32(g_scratch, off + 28, f->size);
  if (sd_write_block(f->dir_block, g_scratch) != HAL_OK) return PL_FX_ERR_IO;
  sd_wait_idle();
  return PL_FX_OK;
}

pl_fx_status_t SDFat_Flush(PL_File *f)
{
  if (!f || !f->used) return PL_FX_ERR_INVAL;
  if (f->sector_dirty && f->sector_lba) {
    if (sd_write_block(f->sector_lba, f->sector_buf) != HAL_OK) return PL_FX_ERR_IO;
    sd_wait_idle();
    f->sector_dirty = 0;
  }
  fat_flush_page();
  return patch_dir_entry(f);
}

pl_fx_status_t SDFat_Close(PL_File *f)
{
  if (!f || !f->used) return PL_FX_OK;
  pl_fx_status_t s = SDFat_Flush(f);
  f->used = 0;
  return s;
}

/* ============================================================================
   BLE FileSync support (Phase 5) */

/* Convert an 11-byte padded 8.3 name into a NUL-terminated "NAME.EXT" string.
   `out` must hold at least 13 bytes. */
static void name83_to_str(const char name83[11], char *out)
{
  int o = 0;
  for (int i = 0; i < 8 && name83[i] != ' '; i++) out[o++] = name83[i];
  if (name83[8] != ' ') {
    out[o++] = '.';
    for (int i = 8; i < 11 && name83[i] != ' '; i++) out[o++] = name83[i];
  }
  out[o] = '\0';
}

typedef struct {
  SDFat_ListCb cb;
  void        *user;
  int          stop;
} list_ctx_t;

static int list_cb(const dir_entry_t *e, void *user)
{
  list_ctx_t *c = (list_ctx_t *)user;
  char name[13];
  name83_to_str(e->name83, name);
  if (c->cb(name, e->size, c->user)) { c->stop = 1; return 1; }
  return 0;
}

pl_fx_status_t SDFat_ListRoot(SDFat_ListCb cb, void *user)
{
  if (!g_mounted) return PL_FX_ERR_INIT;
  if (!cb)        return PL_FX_ERR_INVAL;
  list_ctx_t c = { .cb = cb, .user = user, .stop = 0 };
  return walk_root(list_cb, &c);
}

pl_fx_status_t SDFat_OpenRead(PL_File *out, const char *name)
{
  if (!g_mounted || !out || !name) return PL_FX_ERR_INVAL;
  memset(out, 0, sizeof(*out));
  to_83(name, out->name83);

  find_ctx_t c = { .name83 = out->name83, .ok = 0 };
  walk_root(find_cb, &c);
  if (!c.ok) return PL_FX_ERR_NOT_FOUND;

  out->first_cluster = c.found.first_cluster;
  out->cur_cluster   = c.found.first_cluster;
  out->cur_offset    = 0;
  out->size          = c.found.size;
  out->dir_block     = c.found.dir_block;
  out->dir_entry_idx = c.found.dir_idx;
  out->sector_lba    = 0;
  out->rd_pos        = 0;
  out->used          = 1;
  return PL_FX_OK;
}

pl_fx_status_t SDFat_Read(PL_File *f, void *buf, uint32_t len, uint32_t *got)
{
  if (got) *got = 0;
  if (!f || !f->used) return PL_FX_ERR_INVAL;

  uint32_t cluster_bytes = g_sec_per_clus * 512u;
  uint8_t *dst = (uint8_t *)buf;
  uint32_t total = 0;

  while (len > 0 && f->rd_pos < f->size) {
    /* Cross to the next cluster if the within-cluster cursor is exhausted. */
    if (f->cur_offset >= cluster_bytes) {
      uint32_t next;
      if (fat_get_next(f->cur_cluster, &next) != PL_FX_OK) return PL_FX_ERR_IO;
      if (next < 2 || next >= 0x0FFFFFF8u) break;   /* end of chain */
      f->cur_cluster = next;
      f->cur_offset  = 0;
    }

    uint32_t sec_in_cl  = f->cur_offset / 512u;
    uint32_t sec_off    = f->cur_offset % 512u;
    uint32_t target_lba = cluster_to_lba(f->cur_cluster) + sec_in_cl;

    if (sd_read_block(target_lba, g_scratch) != HAL_OK) return PL_FX_ERR_IO;

    uint32_t avail_in_sec  = 512u - sec_off;
    uint32_t avail_in_file = f->size - f->rd_pos;
    uint32_t take = len;
    if (take > avail_in_sec)  take = avail_in_sec;
    if (take > avail_in_file) take = avail_in_file;

    memcpy(dst, &g_scratch[sec_off], take);
    dst           += take;
    total         += take;
    len           -= take;
    f->cur_offset += take;
    f->rd_pos     += take;
  }

  if (got) *got = total;
  return PL_FX_OK;
}

pl_fx_status_t SDFat_Seek(PL_File *f, uint32_t offset)
{
  if (!f || !f->used) return PL_FX_ERR_INVAL;
  if (offset > f->size) offset = f->size;

  uint32_t cluster_bytes = g_sec_per_clus * 512u;
  uint32_t cluster = f->first_cluster;
  uint32_t skip    = offset / cluster_bytes;
  for (uint32_t i = 0; i < skip; i++) {
    uint32_t next;
    if (fat_get_next(cluster, &next) != PL_FX_OK) return PL_FX_ERR_IO;
    if (next < 2 || next >= 0x0FFFFFF8u) return PL_FX_ERR_IO;
    cluster = next;
  }
  f->cur_cluster = cluster;
  f->cur_offset  = offset % cluster_bytes;
  f->rd_pos      = offset;
  return PL_FX_OK;
}

pl_fx_status_t SDFat_Delete(const char *name)
{
  if (!g_mounted || !name) return PL_FX_ERR_INVAL;

  char n83[11];
  to_83(name, n83);
  find_ctx_t c = { .name83 = n83, .ok = 0 };
  walk_root(find_cb, &c);
  if (!c.ok) return PL_FX_ERR_NOT_FOUND;

  /* Free the cluster chain: walk first→next, marking each cluster 0 (free).
     Capture `next` before zeroing the current entry. A zero or invalid
     first_cluster means the file body was never allocated — skip. */
  uint32_t cluster = c.found.first_cluster;
  while (cluster >= 2 && cluster < 0x0FFFFFF8u) {
    uint32_t next;
    if (fat_get_next(cluster, &next) != PL_FX_OK) return PL_FX_ERR_IO;
    if (fat_set_next(cluster, 0) != PL_FX_OK)     return PL_FX_ERR_IO;
    cluster = next;
  }
  if (fat_flush_page() != PL_FX_OK) return PL_FX_ERR_IO;

  /* Mark the directory entry deleted (first byte 0xE5). */
  if (sd_read_block(c.found.dir_block, g_scratch) != HAL_OK) return PL_FX_ERR_IO;
  g_scratch[c.found.dir_idx * 32u] = 0xE5;
  if (sd_write_block(c.found.dir_block, g_scratch) != HAL_OK) return PL_FX_ERR_IO;
  sd_wait_idle();

  /* Reclaim the freed clusters on the next allocation. */
  if (c.found.first_cluster >= 2 && c.found.first_cluster < g_alloc_hint) {
    g_alloc_hint = c.found.first_cluster;
  }
  return PL_FX_OK;
}
