/**
  ******************************************************************************
  * @file    fwupdate.c
  * @brief   Brick-safe dual-bank (A/B) firmware-over-BLE receiver. See
  *          fwupdate.h for the protocol + design rationale, and DESIGN.md
  *          §"Firmware update over BLE (FOTA)".
  *
  *          Brick-safety invariants (do not weaken without re-reading the ST
  *          dual-bank gotchas in CLAUDE.md):
  *            1. We only ever erase/program the INACTIVE bank, computed live
  *               from the SWAP_BANK option byte. HAL erase bank numbers follow
  *               the swap state, so a hardcoded FLASH_BANK_2 would erase the
  *               RUNNING bank once swapped — we read FLASH->OPTR every time.
  *            2. The active (running) bank is NEVER touched. The previous image
  *               stays put as an automatic rollback.
  *            3. SWAP_BANK is toggled ONLY after the full image is programmed
  *               and its SHA-256 matches the host-declared digest. A corrupt or
  *               interrupted upload therefore cannot brick the box.
  *            4. A verified-but-runtime-hanging image is caught by the SD
  *               boot-attempt counter (BootCheck/ConfirmBoot), which reverts
  *               the swap after FW_MAX_BOOT_ATTEMPTS unconfirmed boots.
  ******************************************************************************
  */
#include "main.h"
#include "fwupdate.h"
#include "sha256.h"
#include "sd_fatfs.h"
#include "errlog.h"
#include "watchdog.h"
#include <string.h>

/* The two physical banks alias to fixed addresses; SWAP_BANK only changes
   which one the boot-ROM maps to 0x08000000. The running bank is therefore
   always at 0x08000000 and the inactive bank always at 0x08000000+1MB. */
#define FW_INACTIVE_BASE   (0x08000000UL + FLASH_BANK_SIZE)   /* 0x08100000 */

/* SD pending-update marker (8.3 name). Written at COMMIT, cleared once the new
   image confirms healthy, used by BootCheck to drive runtime-hang rollback. */
#define FW_MARKER_NAME     "FWPEND.STA"
#define FW_MARKER_MAGIC    0x4D4C4655u   /* "MLFU" — MovementLogger FW Update */

typedef struct {
  uint32_t magic;
  uint32_t attempts;
} fw_marker_t;

static struct {
  int      active;
  int      verified;                       /* COMMIT passed; Activate allowed  */
  uint32_t image_len;
  uint8_t  expected_sha[32];
  uint32_t prog_off;                       /* bytes fully programmed (×16)     */
  uint32_t stage_len;                      /* bytes buffered for next quadword */
  uint8_t  stage[16] __attribute__((aligned(16)));
} fw;

/* ----- flash helpers ----------------------------------------------------- */

/* Inactive physical bank number for HAL erase — inverted when swapped. */
static uint32_t fw_inactive_bank(void)
{
  return (READ_BIT(FLASH->OPTR, FLASH_OPTR_SWAP_BANK)) ? FLASH_BANK_1
                                                       : FLASH_BANK_2;
}

/* Erase exactly the inactive-bank pages the image will occupy, one 8 KB page
   per HAL call so the IWDG is fed between pages. Returns 0 / -1. */
static int fw_erase_inactive(uint32_t image_len)
{
  uint32_t npages = (image_len + FLASH_PAGE_SIZE - 1u) / FLASH_PAGE_SIZE;
  if (npages == 0) npages = 1;
  uint32_t bank = fw_inactive_bank();

  if (HAL_FLASH_Unlock() != HAL_OK) return -1;
  for (uint32_t p = 0; p < npages; p++) {
    FLASH_EraseInitTypeDef er = {0};
    er.TypeErase = FLASH_TYPEERASE_PAGES;
    er.Banks     = bank;
    er.Page      = p;
    er.NbPages   = 1;
    uint32_t perr = 0;
    HAL_StatusTypeDef s = HAL_FLASHEx_Erase(&er, &perr);
    Watchdog_Kick();                       /* erase can take tens of ms/page  */
    if (s != HAL_OK) { HAL_FLASH_Lock(); return -1; }
  }
  HAL_FLASH_Lock();
  return 0;
}

/* Program one 128-bit quadword at FW_INACTIVE_BASE+off. q must be 16 bytes. */
static int fw_program_quadword(uint32_t off, const uint8_t q[16])
{
  if (HAL_FLASH_Unlock() != HAL_OK) return -1;
  HAL_StatusTypeDef s = HAL_FLASH_Program(FLASH_TYPEPROGRAM_QUADWORD,
                                          FW_INACTIVE_BASE + off,
                                          (uint32_t)(uintptr_t)q);
  HAL_FLASH_Lock();
  return (s == HAL_OK) ? 0 : -1;
}

/* Toggle the SWAP_BANK option byte and launch it (reloads option bytes +
   resets the MCU). Returns only on failure. */
static int fw_swap_bank_and_reset(void)
{
  if (HAL_FLASH_Unlock() != HAL_OK) return -1;
  if (HAL_FLASH_OB_Unlock() != HAL_OK) { HAL_FLASH_Lock(); return -1; }

  FLASH_OBProgramInitTypeDef ob = {0};
  ob.OptionType = OPTIONBYTE_USER;
  ob.USERType   = OB_USER_SWAP_BANK;
  ob.USERConfig = (READ_BIT(FLASH->OPTR, FLASH_OPTR_SWAP_BANK))
                    ? OB_SWAP_BANK_DISABLE : OB_SWAP_BANK_ENABLE;
  if (HAL_FLASHEx_OBProgram(&ob) != HAL_OK) {
    HAL_FLASH_OB_Lock(); HAL_FLASH_Lock(); return -1;
  }
  HAL_FLASH_OB_Launch();                   /* commits + system reset          */

  /* Belt-and-suspenders: some parts return here instead of resetting. */
  HAL_FLASH_OB_Lock();
  HAL_FLASH_Lock();
  HAL_NVIC_SystemReset();
  return 0;                                /* unreached */
}

/* ----- SD pending marker ------------------------------------------------- */

static int fw_marker_read(fw_marker_t *m)
{
  PL_File f;
  if (SDFat_OpenRead(&f, FW_MARKER_NAME) != PL_FX_OK) return 0;   /* absent */
  uint32_t got = 0;
  pl_fx_status_t s = SDFat_Read(&f, m, sizeof(*m), &got);
  SDFat_Close(&f);
  if (s != PL_FX_OK || got < sizeof(*m) || m->magic != FW_MARKER_MAGIC) return 0;
  return 1;
}

static int fw_marker_write(uint32_t attempts)
{
  SDFat_Delete(FW_MARKER_NAME);            /* overwrite = delete + recreate   */
  PL_File f;
  if (SDFat_OpenAppend(&f, FW_MARKER_NAME) != PL_FX_OK) return -1;
  fw_marker_t m = { FW_MARKER_MAGIC, attempts };
  pl_fx_status_t s = SDFat_Append(&f, &m, sizeof(m));
  SDFat_Flush(&f);
  SDFat_Close(&f);
  return (s == PL_FX_OK) ? 0 : -1;
}

static void fw_marker_clear(void)
{
  SDFat_Delete(FW_MARKER_NAME);
}

/* ----- public API -------------------------------------------------------- */

fwup_status_t FwUpdate_Begin(uint32_t image_len, const uint8_t sha256[32])
{
  if (image_len == 0 || image_len > FW_MAX_IMAGE_BYTES) return FWUP_ERR_TOO_BIG;

  ErrLog_Writef("fwupdate: BEGIN len=%lu bank=%lu",
                (unsigned long)image_len, (unsigned long)fw_inactive_bank());

  if (fw_erase_inactive(image_len) != 0) {
    fw.active = 0;
    ErrLog_Write("fwupdate: ERASE FAIL");
    return FWUP_ERR_FLASH;
  }

  fw.active    = 1;
  fw.verified  = 0;
  fw.image_len = image_len;
  fw.prog_off  = 0;
  fw.stage_len = 0;
  memcpy(fw.expected_sha, sha256, 32);
  return FWUP_OK;
}

fwup_status_t FwUpdate_Data(uint32_t offset, const uint8_t *data, uint32_t len,
                            uint32_t *next_offset)
{
  if (!fw.active) return FWUP_ERR_NOT_READY;

  uint32_t expected = fw.prog_off + fw.stage_len;
  if (offset < expected) {
    /* Idempotent retransmit (a lost ACK made the host resend). Re-ACK the
       current cursor without reprogramming. */
    *next_offset = expected;
    return FWUP_OK;
  }
  if (offset > expected) return FWUP_ERR_BAD_SEQ;   /* gap — host skipped     */

  for (uint32_t i = 0; i < len; i++) {
    if (fw.prog_off + fw.stage_len >= fw.image_len) break;  /* ignore overrun */
    fw.stage[fw.stage_len++] = data[i];
    if (fw.stage_len == 16) {
      if (fw_program_quadword(fw.prog_off, fw.stage) != 0) {
        ErrLog_Writef("fwupdate: PROGRAM FAIL off=%lu", (unsigned long)fw.prog_off);
        fw.active = 0;
        return FWUP_ERR_FLASH;
      }
      fw.prog_off += 16;
      fw.stage_len = 0;
    }
  }
  *next_offset = fw.prog_off + fw.stage_len;
  return FWUP_OK;
}

fwup_status_t FwUpdate_Commit(void)
{
  if (!fw.active) return FWUP_ERR_NOT_READY;

  /* Flush the final partial quadword, 0xFF-padded. */
  if (fw.stage_len > 0) {
    for (uint32_t i = fw.stage_len; i < 16; i++) fw.stage[i] = 0xFF;
    if (fw_program_quadword(fw.prog_off, fw.stage) != 0) {
      fw.active = 0;
      return FWUP_ERR_FLASH;
    }
    fw.prog_off += 16;
    fw.stage_len = 0;
  }
  if (fw.prog_off < fw.image_len) return FWUP_ERR_BAD_SEQ;   /* short image   */

  /* Verify SHA-256 over exactly image_len bytes of the freshly-programmed
     inactive bank before we dare touch SWAP_BANK. */
  Watchdog_Kick();
  sha256_ctx c;
  uint8_t dig[32];
  Sha256_Init(&c);
  Sha256_Update(&c, (const uint8_t *)FW_INACTIVE_BASE, fw.image_len);
  Sha256_Final(&c, dig);
  Watchdog_Kick();

  if (memcmp(dig, fw.expected_sha, 32) != 0) {
    ErrLog_Write("fwupdate: COMMIT hash MISMATCH — refusing to activate");
    ErrLog_Flush();
    fw.active = 0;
    fw.verified = 0;
    return FWUP_ERR_HASH;
  }

  ErrLog_Write("fwupdate: image verified, ready to activate");
  fw.verified = 1;             /* FwUpdate_Activate() may now swap + reset      */
  return FWUP_OK;
}

void FwUpdate_Activate(void)
{
  if (!fw.verified) return;    /* only after a successful FwUpdate_Commit       */
  ErrLog_Write("fwupdate: arming bank swap + reset");
  ErrLog_Flush();
  fw_marker_write(0);          /* best-effort rollback net; proceed regardless  */
  fw.active   = 0;
  fw.verified = 0;
  fw_swap_bank_and_reset();    /* does not return on success                    */
}

void FwUpdate_Abort(void)
{
  if (fw.active) ErrLog_Write("fwupdate: ABORT");
  fw.active = 0;
  fw.verified = 0;
  fw.stage_len = 0;
}

int FwUpdate_Active(void) { return fw.active; }

/* ----- boot-time rollback ------------------------------------------------ */

void FwUpdate_BootCheck(void)
{
  fw_marker_t m;
  if (!fw_marker_read(&m)) return;         /* no update pending — normal boot */

  uint32_t attempts = m.attempts + 1;
  if (attempts > FW_MAX_BOOT_ATTEMPTS) {
    ErrLog_Write("fwupdate: trial image unconfirmed — reverting bank swap");
    ErrLog_Flush();
    fw_marker_clear();
    fw_swap_bank_and_reset();              /* revert to previous image (reset)*/
    return;                                /* unreached on success            */
  }
  fw_marker_write(attempts);
  ErrLog_Writef("fwupdate: trial boot %lu/%u",
                (unsigned long)attempts, FW_MAX_BOOT_ATTEMPTS);
}

void FwUpdate_ConfirmBoot(void)
{
  static int done = 0;
  if (done) return;
  if (HAL_GetTick() < FW_CONFIRM_UPTIME_MS) return;
  done = 1;

  fw_marker_t m;
  if (fw_marker_read(&m)) {
    fw_marker_clear();
    ErrLog_Write("fwupdate: new image confirmed healthy");
  }
}
