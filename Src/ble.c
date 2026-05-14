/**
  ******************************************************************************
  * @file    ble.c
  * @brief   BlueNRG-LP HCI driver — minimal subset, polling only.
  *          Wiring (lifted from BLEDualProgram::hci_tl_interface.h):
  *            SPI1 SCK   PA5  AF5
  *            SPI1 MOSI  PA7  AF5
  *            SPI1 MISO  PA6  AF5
  *            CS         PA2  GPIO output, idle HIGH
  *            RESET      PD4  GPIO output, idle HIGH (active LOW)
  *            IRQ        PB11 GPIO input (NO NVIC — we poll the level
  *                            during BLE_Tick / inside SPI transactions)
  *
  *          BlueNRG SPI protocol (5-byte header):
  *            Write:  master sends {0x0a,0,0,0,0}, slave returns its
  *                    available-buffer-size in bytes 1-2 (LE).
  *            Read:   master sends {0x0b,0,0,0,0}, slave returns its
  *                    pending-byte-count in bytes 3-4 (LE).
  *
  *          HCI frame format on the wire:
  *            Command: 0x01 <opcode_lo> <opcode_hi> <param_len> <params...>
  *            Event:   0x04 <event_code> <param_len> <params...>
  *
  *          Minimum advertising sequence for Phase 4:
  *            HCI_RESET                       (opcode 0x0C03)
  *            HCI_LE_Set_Advertising_Params   (opcode 0x2006)
  *            HCI_LE_Set_Advertising_Data     (opcode 0x2008)
  *            HCI_LE_Set_Advertising_Enable   (opcode 0x200A)
  ******************************************************************************
  */
#include "main.h"
#include "ble.h"
#include "errlog.h"
#include "sd_fatfs.h"
#include "watchdog.h"
#include "logger.h"
#include "stream.h"
#include <string.h>
#include <stdio.h>

/* ----- Pin / port macros ------------------------------------------------- */
#define BLE_SPI_SCK_PORT   GPIOA
#define BLE_SPI_SCK_PIN    GPIO_PIN_5
#define BLE_SPI_MISO_PORT  GPIOA
#define BLE_SPI_MISO_PIN   GPIO_PIN_6
#define BLE_SPI_MOSI_PORT  GPIOA
#define BLE_SPI_MOSI_PIN   GPIO_PIN_7
#define BLE_SPI_AF         GPIO_AF5_SPI1
#define BLE_CS_PORT        GPIOA
#define BLE_CS_PIN         GPIO_PIN_2
#define BLE_RST_PORT       GPIOD
#define BLE_RST_PIN        GPIO_PIN_4
#define BLE_IRQ_PORT       GPIOB
#define BLE_IRQ_PIN        GPIO_PIN_11

/* ----- HCI opcodes ------------------------------------------------------- */
#define HCI_OP_RESET                    0x0C03

/* ACI (BlueNRG-LP vendor) opcodes — OGF=0x3F, packed as (OGF<<10)|OCF.
   The advertising flow uses the BT 5.0 extended set:
     SET_ADVERTISING_CONFIGURATION → SET_ADVERTISING_DATA_NWK →
     SET_ADVERTISING_ENABLE
   Standard HCI_LE_Set_Adv_* commands are rejected (status 7) after
   aci_gap_init takes over the advertising machinery — verified in Build #17. */
#define ACI_OP_GAP_INIT                       0xFC8A   /* OCF=0x08A */
#define ACI_OP_GAP_SET_ADV_CONFIGURATION      0xFCAB   /* OCF=0x0AB */
#define ACI_OP_GAP_SET_ADV_ENABLE             0xFCAC   /* OCF=0x0AC */
#define ACI_OP_GAP_SET_ADV_DATA_NWK           0xFCAD   /* OCF=0x0AD */

/* GATT server */
#define ACI_OP_GATT_SRV_INIT                  0xFD01   /* OCF=0x101 */
#define ACI_OP_GATT_SRV_ADD_SERVICE_NWK       0xFD02   /* OCF=0x102 */
#define ACI_OP_GATT_SRV_ADD_CHAR_NWK          0xFD04   /* OCF=0x104 */
#define ACI_OP_GATT_SRV_NOTIFY                0xFD2F   /* OCF=0x12F */

/* FileSync wire protocol (CLAUDE.md) */
#define FSYNC_OP_LIST       0x01
#define FSYNC_OP_READ       0x02
#define FSYNC_OP_DELETE     0x03
#define FSYNC_OP_STOP_LOG   0x04
/* Status bytes for READ/DELETE replies */
#define FSYNC_ST_OK         0x00
#define FSYNC_ST_BUSY       0xB0
#define FSYNC_ST_NOT_FOUND  0xE1
#define FSYNC_ST_IO_ERROR   0xE2
#define FSYNC_ST_BAD_REQ    0xE3

/* BlueST FileSync UUIDs (CLAUDE.md). 128-bit, little-endian on the wire. */
static const uint8_t BLE_SVC_UUID[16] = {
  /* 00000000-0001-11e1-9ab4-0002a5d5c51b */
  0x1b,0xc5,0xd5,0xa5,0x02,0x00,0xb4,0x9a,
  0xe1,0x11,0x01,0x00,0x00,0x00,0x00,0x00,
};
static const uint8_t BLE_FILECMD_UUID[16] = {
  /* 00000080-0010-11e1-ac36-0002a5d5c51b */
  0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,
  0xe1,0x11,0x10,0x00,0x80,0x00,0x00,0x00,
};
static const uint8_t BLE_FILEDATA_UUID[16] = {
  /* 00000040-0010-11e1-ac36-0002a5d5c51b */
  0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,
  0xe1,0x11,0x10,0x00,0x40,0x00,0x00,0x00,
};
static const uint8_t BLE_SENSORSTREAM_UUID[16] = {
  /* 00000100-0010-11e1-ac36-0002a5d5c51b */
  0x1b,0xc5,0xd5,0xa5,0x02,0x00,0x36,0xac,
  0xe1,0x11,0x10,0x00,0x00,0x01,0x00,0x00,
};

/* Char property bitfield */
#define BLE_PROP_WRITE          0x08
#define BLE_PROP_WRITE_NO_RSP   0x04
#define BLE_PROP_NOTIFY         0x10

/* ----- State ------------------------------------------------------------- */
static SPI_HandleTypeDef g_hspi1;
static uint8_t           g_advertising = 0;

/* GATT handles captured during BLE_Init from ACI add_service / add_char
   responses. cmd_handle is what the host writes commands to; data_handle
   is the source of our notifications. */
static uint16_t          g_svc_handle      = 0;
static uint16_t          g_filecmd_handle  = 0;
static uint16_t          g_filedata_handle = 0;
static uint16_t          g_stream_handle   = 0;

/* Current connection handle (0 = no active connection). Set in BLE_Tick on
   HCI_LE_Connection_Complete; cleared on Disconnection_Complete. */
static uint16_t          g_conn_handle     = 0;

/* SensorStream: emitted only while a client has subscribed to its CCCD.
   No mode switch — SD logging runs regardless (DESIGN.md Section 2/3). */
static uint8_t           g_stream_subscribed = 0;
static uint32_t          g_stream_last_ms    = 0;
#define STREAM_PERIOD_MS 2000

#define BLE_ADV_NAME      "PumpTsueri"
#define BLE_ADV_NAME_LEN  10

/* ----- Pin helpers ------------------------------------------------------- */
static inline void cs_lo(void) { HAL_GPIO_WritePin(BLE_CS_PORT, BLE_CS_PIN, GPIO_PIN_RESET); }
static inline void cs_hi(void) { HAL_GPIO_WritePin(BLE_CS_PORT, BLE_CS_PIN, GPIO_PIN_SET); }
static inline int  irq_high(void) {
  return HAL_GPIO_ReadPin(BLE_IRQ_PORT, BLE_IRQ_PIN) == GPIO_PIN_SET;
}

/* ----- SPI transfer ------------------------------------------------------ */

static int spi_xfer(const uint8_t *tx, uint8_t *rx, uint16_t n)
{
  /* Caller must hold CS. We use HAL polling with a generous timeout —
     transfers are tiny (max ~64 bytes per HCI command). */
  HAL_StatusTypeDef s = HAL_SPI_TransmitReceive(&g_hspi1, (uint8_t *)tx, rx, n, 100);
  return (s == HAL_OK) ? 0 : -1;
}

/* Diagnostic snapshots of the last header exchanges — emitted to ErrLog
   during init so we can see exactly what the chip reports. */
static uint8_t  g_diag_last_send_hdr[5];
static uint8_t  g_diag_last_recv_hdr[5];

/* Send the BlueNRG 5-byte write-mode header. Returns chip's reported
   available write buffer size (bytes), or -1 on SPI error. */
static int ble_spi_send_header(void)
{
  uint8_t tx[5] = { 0x0a, 0, 0, 0, 0 };
  uint8_t rx[5] = { 0 };
  if (spi_xfer(tx, rx, 5) != 0) return -1;
  memcpy(g_diag_last_send_hdr, rx, 5);
  /* per ST source: rx_bytes = (header_slave[2] << 8) | header_slave[1] */
  return (int)((((uint16_t)rx[2]) << 8) | (uint16_t)rx[1]);
}

/* Send the BlueNRG 5-byte read-mode header. Returns chip's pending byte
   count, or -1 on SPI error. */
static int ble_spi_recv_header(void)
{
  uint8_t tx[5] = { 0x0b, 0, 0, 0, 0 };
  uint8_t rx[5] = { 0 };
  if (spi_xfer(tx, rx, 5) != 0) return -1;
  memcpy(g_diag_last_recv_hdr, rx, 5);
  /* per ST source: byte_count = (header_slave[4] << 8) | header_slave[3] */
  return (int)((((uint16_t)rx[4]) << 8) | (uint16_t)rx[3]);
}

/* Send a HCI command + payload to BlueNRG-LP. Loops with timeout until the
   chip is ready (IRQ HIGH + write-header reports enough buffer space). */
static int ble_hci_send(const uint8_t *frame, uint16_t len)
{
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < 100) {
    cs_lo();

    /* Wait for IRQ pin high — chip ready to be addressed. */
    uint32_t s2 = HAL_GetTick();
    while (!irq_high()) {
      if ((HAL_GetTick() - s2) > 15) { cs_hi(); goto retry; }
    }

    int rx_avail = ble_spi_send_header();
    if (rx_avail < (int)len) {
      cs_hi();
      goto retry;
    }

    /* HAL_SPI_TransmitReceive requires both buffers non-NULL — Build #15
       passed NULL for the discard buffer and got HAL_ERROR back. Use a
       static scratch buffer big enough for any HCI command (max ~260 B). */
    static uint8_t s_rx_discard[260];
    uint16_t xn = (len > sizeof(s_rx_discard)) ? sizeof(s_rx_discard) : len;
    if (spi_xfer(frame, s_rx_discard, xn) != 0) { cs_hi(); return -2; }

    cs_hi();
    /* Wait IRQ falls (transaction complete). */
    uint32_t s3 = HAL_GetTick();
    while (irq_high()) {
      if ((HAL_GetTick() - s3) > 100) break;
    }
    return 0;

retry:
    HAL_Delay(1);
  }
  return -3;
}

/* Drain one HCI event (if any) from the chip into `buf` (size cap). Returns
   bytes received (0 = no event yet, <0 = error). */
static int ble_hci_recv(uint8_t *buf, uint16_t cap)
{
  if (!irq_high()) return 0;

  cs_lo();
  int byte_count = ble_spi_recv_header();
  if (byte_count <= 0) { cs_hi(); return 0; }
  if (byte_count > cap) byte_count = cap;

  uint8_t zero = 0;
  for (int i = 0; i < byte_count; i++) {
    if (spi_xfer(&zero, &buf[i], 1) != 0) { cs_hi(); return -1; }
  }

  cs_hi();
  /* Wait IRQ falls. */
  uint32_t s = HAL_GetTick();
  while (irq_high()) { if ((HAL_GetTick() - s) > 100) break; }
  return byte_count;
}

/* Send a HCI command (opcode + optional payload) and wait for the matching
   CommandComplete event. Returns the status byte (0=OK) or -1 on timeout. */
static int ble_hci_cmd(uint16_t opcode, const uint8_t *payload, uint8_t plen)
{
  uint8_t frame[260];
  frame[0] = 0x01;                                 /* HCI command packet */
  frame[1] = (uint8_t)(opcode & 0xFF);
  frame[2] = (uint8_t)(opcode >> 8);
  frame[3] = plen;
  if (plen > 0 && payload) memcpy(&frame[4], payload, plen);

  if (ble_hci_send(frame, (uint16_t)(4 + plen)) != 0) return -10;

  /* Poll for CommandComplete event (opcode 0x0E). */
  uint8_t evt[260];
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < 500) {
    int n = ble_hci_recv(evt, sizeof(evt));
    if (n <= 0) { HAL_Delay(2); continue; }
    /* Event format: 0x04 <evt_code> <plen> <params> */
    if (n < 7) continue;
    if (evt[0] != 0x04) continue;
    if (evt[1] != 0x0E) continue;                /* not CommandComplete */
    /* CommandComplete params: <num_pkts> <opcode_lo> <opcode_hi> <status> */
    uint16_t evt_op = (uint16_t)(evt[4] | (evt[5] << 8));
    if (evt_op != opcode) continue;              /* status of another cmd */
    return (int)evt[6];
  }
  return -11;
}

/* ACI extended command frame:
     0x81 <opcode_lo> <opcode_hi> <plen_lo> <plen_hi> <params...>
   BlueNRG-LP vendor commands (OGF=0x3F + OCF) use this format with
   16-bit plen. Response comes back as a normal CommandComplete event
   (0x04 0x0E ...) keyed by the same opcode. */
static int ble_aci_cmd(uint16_t opcode, const uint8_t *payload, uint16_t plen,
                       uint8_t *resp_buf, uint8_t resp_cap)
{
  uint8_t frame[260];
  frame[0] = 0x81;
  frame[1] = (uint8_t)(opcode & 0xFF);
  frame[2] = (uint8_t)(opcode >> 8);
  frame[3] = (uint8_t)(plen & 0xFF);
  frame[4] = (uint8_t)(plen >> 8);
  if (plen > 0 && payload) memcpy(&frame[5], payload, plen);

  if (ble_hci_send(frame, (uint16_t)(5 + plen)) != 0) return -10;

  uint8_t evt[260];
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < 1000) {
    int n = ble_hci_recv(evt, sizeof(evt));
    if (n <= 0) { HAL_Delay(2); continue; }
    if (n < 7) continue;
    if (evt[0] != 0x04) continue;
    if (evt[1] != 0x0E) continue;
    uint16_t evt_op = (uint16_t)(evt[4] | (evt[5] << 8));
    if (evt_op != opcode) continue;
    /* CommandComplete params after opcode: <status> <return_params...>. We
       copy return_params starting at evt[7] (after status byte). */
    if (resp_buf && resp_cap > 0) {
      int copy = n - 7;
      if (copy < 0) copy = 0;
      if (copy > resp_cap) copy = resp_cap;
      memcpy(resp_buf, &evt[7], (size_t)copy);
    }
    return (int)evt[6];
  }
  return -11;
}

/* (Re-)enable advertising. BLE stops advertising the moment a connection
   is established and does NOT auto-resume after the link drops — so this
   must be called both at init and from the Disconnection_Complete handler,
   otherwise the box is invisible after its first connect/disconnect cycle.
   adv_config + adv_data only need to be set once; this is just the enable. */
static int ble_adv_enable(void)
{
  uint8_t p[6] = { 0x01, 0x01, 0x00, 0x00, 0x00, 0x00 };
  return ble_aci_cmd(ACI_OP_GAP_SET_ADV_ENABLE, p, sizeof(p), NULL, 0);
}

/* ----- FileData notify --------------------------------------------------- */

/* Send a notification on the FileData characteristic. val_handle is the
   characteristic VALUE handle (= the add_char declaration handle + 1).
   Returns 0 on success. */
static int ble_notify(uint16_t val_handle, const uint8_t *data, uint16_t len)
{
  uint8_t p[256];
  int i = 0;
  p[i++] = (uint8_t)(g_conn_handle & 0xFF);
  p[i++] = (uint8_t)(g_conn_handle >> 8);
  p[i++] = (uint8_t)(val_handle & 0xFF);
  p[i++] = (uint8_t)(val_handle >> 8);
  p[i++] = 0x00;                                  /* flags: notification */
  p[i++] = (uint8_t)(len & 0xFF);
  p[i++] = (uint8_t)(len >> 8);
  if (len > 0) { memcpy(&p[i], data, len); i += len; }
  return ble_aci_cmd(ACI_OP_GATT_SRV_NOTIFY, p, (uint16_t)i, NULL, 0);
}

/* Notify with congestion handling. The BlueNRG-LP TX buffer holds only a
   handful of pending notifications; once full, aci_gatt_srv_notify returns
   non-zero (INSUFFICIENT_RESOURCES). We back off + retry the SAME payload —
   the radio drains a packet roughly every connection interval. The IWDG is
   kicked each retry so a large transfer doesn't trip the 8 s watchdog.
   Returns 0 on success, -1 if it never drains within the retry budget. */
static int ble_notify_blocking(uint16_t val_handle, const uint8_t *data, uint16_t len)
{
  for (int retry = 0; retry < 400; retry++) {
    int rc = ble_notify(val_handle, data, len);
    if (rc == 0) return 0;
    Watchdog_Kick();
    HAL_Delay(10);   /* let the radio push one packet out */
  }
  return -1;
}

/* ----- FileSync command handling ----------------------------------------- */

/* Captured FileCmd write payload, set by the attribute-modified handler and
   consumed by ble_process_command() at the end of BLE_Tick. */
static uint8_t  g_cmd_buf[64];
static uint8_t  g_cmd_len     = 0;
static uint8_t  g_cmd_pending = 0;

/* LIST: one notification per root-dir entry, "name,size\n", then a lone
   "\n" terminator. Callback runs inside SDFat_ListRoot's walk. */
static int list_emit_cb(const char *name, uint32_t size, void *user)
{
  (void)user;
  char row[40];
  int n = snprintf(row, sizeof(row), "%s,%lu\n", name, (unsigned long)size);
  if (n > 0) {
    ble_notify_blocking(g_filedata_handle + 1, (const uint8_t *)row, (uint16_t)n);
  }
  return 0;   /* continue enumeration */
}

static void ble_process_command(void)
{
  if (!g_cmd_pending) return;
  g_cmd_pending = 0;
  if (g_cmd_len < 1) return;

  uint8_t op = g_cmd_buf[0];
  char buf[64];

  if (op == FSYNC_OP_LIST) {
    snprintf(buf, sizeof(buf), "ble: cmd LIST conn=0x%04x fdh=0x%04x",
             g_conn_handle, (uint16_t)(g_filedata_handle + 1));
    ErrLog_Write(buf);
    SDFat_ListRoot(list_emit_cb, NULL);
    uint8_t term = '\n';
    int rc = ble_notify_blocking(g_filedata_handle + 1, &term, 1);
    snprintf(buf, sizeof(buf), "ble: LIST done term_rc=%d", rc);
    ErrLog_Write(buf);
  } else if (op == FSYNC_OP_READ) {
    /* READ <name>\0[<offset:u32-LE>]: NUL-terminated 8.3 filename, optional
       4-byte little-endian start offset. Offset absent → 0 → whole file.
       A resumed transfer after a dropped link passes offset = bytes the
       host already has. Streams raw file bytes; NOT_FOUND / IO_ERROR /
       BAD_REQUEST come back as a single status byte instead. */
    char     name[16];
    uint32_t offset = 0;

    if (g_cmd_len <= 1) {
      /* TEST SHORTCUT: a bare 0x02 reads a fixed default file at offset 0.
         nRF Connect Mobile caps the hex-write field at 16 chars, too short
         for "02 <name> 00 <4-byte offset>". Real clients send the full
         command. Remove or repurpose in Phase 8. */
      strcpy(name, "BAT000.CSV");
      ErrLog_Write("ble: READ (test shortcut → BAT000.CSV @0)");
    } else {
      /* Locate the NUL terminator inside g_cmd_buf[1 .. g_cmd_len-1]. */
      int nul = -1;
      for (int i = 1; i < g_cmd_len; i++) {
        if (g_cmd_buf[i] == '\0') { nul = i; break; }
      }
      uint8_t nlen;
      if (nul < 0) {
        /* No NUL — whole remainder is the name (offset 0). Lets the nRF
           hex field still drive a basic READ without the NUL+offset tail. */
        nlen = (uint8_t)(g_cmd_len - 1);
      } else {
        nlen = (uint8_t)(nul - 1);
        /* 4-byte LE offset right after the NUL, if the write included it. */
        if (g_cmd_len >= nul + 5) {
          offset = (uint32_t)g_cmd_buf[nul + 1]
                 | ((uint32_t)g_cmd_buf[nul + 2] << 8)
                 | ((uint32_t)g_cmd_buf[nul + 3] << 16)
                 | ((uint32_t)g_cmd_buf[nul + 4] << 24);
        }
      }
      if (nlen == 0 || nlen > sizeof(name) - 1) {
        uint8_t st = FSYNC_ST_BAD_REQ;
        ble_notify(g_filedata_handle + 1, &st, 1);
        ErrLog_Write("ble: READ bad request");
        return;
      }
      memcpy(name, &g_cmd_buf[1], nlen);
      name[nlen] = '\0';
    }

    PL_File f;
    pl_fx_status_t s = SDFat_OpenRead(&f, name);
    if (s != PL_FX_OK) {
      uint8_t st = (s == PL_FX_ERR_NOT_FOUND) ? FSYNC_ST_NOT_FOUND
                                              : FSYNC_ST_IO_ERROR;
      ble_notify(g_filedata_handle + 1, &st, 1);
      snprintf(buf, sizeof(buf), "ble: READ '%s' open fail s=%d", name, s);
      ErrLog_Write(buf);
      return;
    }

    if (offset > 0) SDFat_Seek(&f, offset);

    snprintf(buf, sizeof(buf), "ble: READ '%s' off=%lu size=%lu",
             name, (unsigned long)offset, (unsigned long)f.size);
    ErrLog_Write(buf);

    /* 20-byte chunks — safe under the default 23-byte ATT MTU. A future
       build can negotiate a larger MTU and use bigger chunks. */
    uint8_t  chunk[20];
    uint32_t sent = 0;                             /* bytes streamed this call */
    int      fail = 0;
    for (;;) {
      uint32_t got = 0;
      if (SDFat_Read(&f, chunk, sizeof(chunk), &got) != PL_FX_OK) { fail = 1; break; }
      if (got == 0) break;                         /* EOF */
      if (ble_notify_blocking(g_filedata_handle + 1, chunk, (uint16_t)got) != 0) {
        fail = 1; break;                           /* TX never drained */
      }
      sent += got;
      Watchdog_Kick();                             /* keep IWDG happy on big files */
    }
    SDFat_Close(&f);
    snprintf(buf, sizeof(buf), "ble: READ '%s' done off=%lu sent=%lu size=%lu%s",
             name, (unsigned long)offset, (unsigned long)sent,
             (unsigned long)f.size, fail ? " FAIL" : "");
    ErrLog_Write(buf);
  } else if (op == FSYNC_OP_DELETE) {
    /* DELETE <name>: free the file's clusters + mark the dir entry deleted.
       Reply is a single status byte on FileData. Same 16-char nRF input
       limit applies → a bare 0x03 deletes a fixed test file. */
    char name[16];
    uint8_t nlen = (g_cmd_len > 1) ? (uint8_t)(g_cmd_len - 1) : 0;
    if (nlen == 0) {
      strcpy(name, "GPS000.CSV");          /* TEST SHORTCUT */
      nlen = (uint8_t)strlen(name);
      ErrLog_Write("ble: DELETE (test shortcut → GPS000.CSV)");
    } else if (nlen > sizeof(name) - 1) {
      uint8_t st = FSYNC_ST_BAD_REQ;
      ble_notify_blocking(g_filedata_handle + 1, &st, 1);
      ErrLog_Write("ble: DELETE bad request");
      return;
    } else {
      memcpy(name, &g_cmd_buf[1], nlen);
      name[nlen] = '\0';
    }

    pl_fx_status_t s = SDFat_Delete(name);
    uint8_t st = (s == PL_FX_OK)              ? FSYNC_ST_OK
               : (s == PL_FX_ERR_NOT_FOUND)   ? FSYNC_ST_NOT_FOUND
                                              : FSYNC_ST_IO_ERROR;
    ble_notify_blocking(g_filedata_handle + 1, &st, 1);
    snprintf(buf, sizeof(buf), "ble: DELETE '%s' s=%d st=0x%02x", name, s, st);
    ErrLog_Write(buf);
  } else if (op == FSYNC_OP_STOP_LOG) {
    /* STOP_LOG: gracefully close the active logging session so the host can
       READ a complete file. No FileData reply — the host re-checks via LIST
       (per CLAUDE.md wire protocol). */
    ErrLog_Write("ble: cmd STOP_LOG");
    Logger_Stop();
  } else {
    snprintf(buf, sizeof(buf), "ble: cmd op=0x%02x (not yet handled)", op);
    ErrLog_Write(buf);
  }
}

/* ----- HW init ----------------------------------------------------------- */

static int ble_hw_init(void)
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};

  /* SPI1 SCK + MOSI + MISO. */
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_NOPULL;
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = BLE_SPI_AF;
  g.Pin       = BLE_SPI_SCK_PIN; HAL_GPIO_Init(BLE_SPI_SCK_PORT, &g);
  g.Pin       = BLE_SPI_MISO_PIN; HAL_GPIO_Init(BLE_SPI_MISO_PORT, &g);
  g.Pin       = BLE_SPI_MOSI_PIN; HAL_GPIO_Init(BLE_SPI_MOSI_PORT, &g);

  /* CS — output, idle HIGH. */
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  g.Pin  = BLE_CS_PIN;
  HAL_GPIO_Init(BLE_CS_PORT, &g);
  cs_hi();

  /* RESET — output, idle HIGH (active LOW). */
  g.Pin  = BLE_RST_PIN;
  HAL_GPIO_Init(BLE_RST_PORT, &g);
  HAL_GPIO_WritePin(BLE_RST_PORT, BLE_RST_PIN, GPIO_PIN_SET);

  /* IRQ — input only, no NVIC. */
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_PULLDOWN;
  g.Pin  = BLE_IRQ_PIN;
  HAL_GPIO_Init(BLE_IRQ_PORT, &g);

  /* SPI1 config — copied from BSP_SPI1_Init in BLEDualProgram (the proven
     setup for BlueNRG-LP). KEY: SPI Mode 3 (CPOL=HIGH, CPHA=2EDGE) — Build
     #14 with Mode 0 returned garbage MISO bytes (0x7f 0x8e ...). The
     ReadyMaster fields drive the BlueNRG's hardware-handshake on the IRQ
     line. */
  memset(&g_hspi1, 0, sizeof(g_hspi1));
  g_hspi1.Instance               = SPI1;
  g_hspi1.Init.Mode              = SPI_MODE_MASTER;
  g_hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
  g_hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
  g_hspi1.Init.CLKPolarity       = SPI_POLARITY_HIGH;
  g_hspi1.Init.CLKPhase          = SPI_PHASE_2EDGE;
  g_hspi1.Init.NSS               = SPI_NSS_SOFT;
  g_hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;  /* 160 MHz / 128 = 1.25 MHz */
  g_hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  g_hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
  g_hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  g_hspi1.Init.CRCPolynomial     = 7;
  g_hspi1.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
  g_hspi1.Init.NSSPolarity       = SPI_NSS_POLARITY_LOW;
  g_hspi1.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
  g_hspi1.Init.MasterSSIdleness  = SPI_MASTER_SS_IDLENESS_00CYCLE;
  g_hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  g_hspi1.Init.MasterReceiverAutoSusp  = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  g_hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  g_hspi1.Init.IOSwap            = SPI_IO_SWAP_DISABLE;
  g_hspi1.Init.ReadyMasterManagement = SPI_RDY_MASTER_MANAGEMENT_INTERNALLY;
  g_hspi1.Init.ReadyPolarity     = SPI_RDY_POLARITY_HIGH;
  if (HAL_SPI_Init(&g_hspi1) != HAL_OK) return -1;
  return 0;
}

static void ble_chip_reset(void)
{
  cs_hi();
  HAL_GPIO_WritePin(BLE_RST_PORT, BLE_RST_PIN, GPIO_PIN_RESET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(BLE_RST_PORT, BLE_RST_PIN, GPIO_PIN_SET);
  /* ST middleware does 150 ms here, then sends hci_reset, then waits another
     2000 ms before issuing any other HCI command. Build #13 with only 200 ms
     here returned cc=-10 (send timeout) on hci_reset → chip wasn't actually
     ready. Bumped to 500 ms; the 2-second post-hci-reset settle is in the
     caller. */
  HAL_Delay(500);
}

/* Drain whatever the chip pushed at us during boot (startup-vendor-event,
   etc.) so the first real HCI exchange isn't fighting a backlog. Returns
   how many events were drained. */
static int ble_drain_pending(uint32_t window_ms)
{
  uint8_t buf[260];
  int drained = 0;
  uint32_t start = HAL_GetTick();
  while ((HAL_GetTick() - start) < window_ms) {
    if (!irq_high()) { HAL_Delay(2); continue; }
    cs_lo();
    int bc = ble_spi_recv_header();
    if (bc <= 0) { cs_hi(); HAL_Delay(2); continue; }
    if (bc > (int)sizeof(buf)) bc = sizeof(buf);
    uint8_t zero = 0;
    for (int i = 0; i < bc; i++) {
      if (spi_xfer(&zero, &buf[i], 1) != 0) break;
    }
    cs_hi();
    drained++;
    /* Brief wait for IRQ to fall before peeking again. */
    uint32_t s = HAL_GetTick();
    while (irq_high()) { if ((HAL_GetTick() - s) > 50) break; }
  }
  return drained;
}

/* ----- Public API -------------------------------------------------------- */

int BLE_Init(void)
{
  char buf[96];

  if (ble_hw_init() != 0) {
    ErrLog_Write("ble: hw_init FAIL");
    return -1;
  }
  ErrLog_Write("ble: spi1+gpio ok");

  ble_chip_reset();
  snprintf(buf, sizeof(buf), "ble: chip reset done irq=%d", irq_high());
  ErrLog_Write(buf);

  /* Drain any startup events the chip pushed at us during boot. ST's
     middleware sees these via the EXTI11 IRQ; we just read them with a
     short polling window. */
  int drained = ble_drain_pending(300);
  snprintf(buf, sizeof(buf), "ble: drained=%d irq=%d hdr=%02x%02x%02x%02x%02x",
           drained, irq_high(),
           g_diag_last_recv_hdr[0], g_diag_last_recv_hdr[1],
           g_diag_last_recv_hdr[2], g_diag_last_recv_hdr[3],
           g_diag_last_recv_hdr[4]);
  ErrLog_Write(buf);

  /* HCI_RESET — first command after reset, validates the SPI link. */
  int rc = ble_hci_cmd(HCI_OP_RESET, NULL, 0);
  snprintf(buf, sizeof(buf), "ble: hci_reset cc=%d sndhdr=%02x%02x%02x%02x%02x",
           rc,
           g_diag_last_send_hdr[0], g_diag_last_send_hdr[1],
           g_diag_last_send_hdr[2], g_diag_last_send_hdr[3],
           g_diag_last_send_hdr[4]);
  ErrLog_Write(buf);
  if (rc != 0) return -2;

  /* ST middleware waits 2 s here for the chip to be fully operational
     before issuing any further HCI commands. */
  HAL_Delay(2000);
  ErrLog_Write("ble: 2s settle done");

  /* GATT server must be initialized BEFORE GAP for BlueNRG-LP (per
     ble_manager.c:3548 ordering). */
  rc = ble_aci_cmd(ACI_OP_GATT_SRV_INIT, NULL, 0, NULL, 0);
  snprintf(buf, sizeof(buf), "ble: gatt_srv_init cc=%d", rc);
  ErrLog_Write(buf);
  if (rc != 0) return -6;

  /* aci_gap_init — peripheral role, static random address. Returns 7 bytes
     of handles which we don't use here (just the status byte). */
  {
    uint8_t p[4] = {
      0x01,                       /* role: peripheral */
      0x00,                       /* privacy: disabled */
      BLE_ADV_NAME_LEN,
      0x01,                       /* identity addr type: static random */
    };
    rc = ble_aci_cmd(ACI_OP_GAP_INIT, p, sizeof(p), NULL, 0);
    snprintf(buf, sizeof(buf), "ble: aci_gap_init cc=%d", rc);
    ErrLog_Write(buf);
    if (rc != 0) return -7;
  }

  /* aci_gatt_srv_add_service_nwk — register the BlueST FileSync service
     (128-bit UUID, primary). 3 chars: FileCmd (2 attrs: decl+value),
     FileData (3: decl+value+CCCD), SensorStream (3) + 1 service decl = 9.
     max_attribute_records = 16 leaves headroom for BatteryStatus (Phase 7). */
  {
    uint8_t p[19];
    p[0] = 0x02;                              /* 128-bit UUID */
    memcpy(&p[1], BLE_SVC_UUID, 16);
    p[17] = 0x01;                             /* primary service */
    p[18] = 0x10;                             /* max_attribute_records = 16 */
    uint8_t resp[3];                          /* status + svc_handle(2) */
    rc = ble_aci_cmd(ACI_OP_GATT_SRV_ADD_SERVICE_NWK, p, sizeof(p),
                     resp, sizeof(resp));
    g_svc_handle = (uint16_t)(resp[0] | (resp[1] << 8));
    snprintf(buf, sizeof(buf), "ble: add_svc cc=%d h=0x%04x", rc, g_svc_handle);
    ErrLog_Write(buf);
    if (rc != 0) return -8;
  }

  /* FileCmd char — write w/o response, max 64 bytes (opcode + filename) */
  {
    uint8_t p[26];
    int i = 0;
    p[i++] = (uint8_t)(g_svc_handle & 0xFF);
    p[i++] = (uint8_t)(g_svc_handle >> 8);
    p[i++] = 0x02;                            /* UUID type: 128-bit */
    memcpy(&p[i], BLE_FILECMD_UUID, 16); i += 16;
    p[i++] = 64; p[i++] = 0;                  /* char_value_length = 64 */
    p[i++] = BLE_PROP_WRITE | BLE_PROP_WRITE_NO_RSP;  /* accept both write types */
    p[i++] = 0x00;                            /* security: none */
    p[i++] = 0x01;                            /* gatt_evt_mask: attribute modified */
    p[i++] = 0x10;                            /* enc_key_size = 16 (unused) */
    p[i++] = 0x01;                            /* is_variable = yes */
    uint8_t resp[3];
    rc = ble_aci_cmd(ACI_OP_GATT_SRV_ADD_CHAR_NWK, p, (uint16_t)i,
                     resp, sizeof(resp));
    g_filecmd_handle = (uint16_t)(resp[0] | (resp[1] << 8));
    snprintf(buf, sizeof(buf), "ble: add_filecmd cc=%d h=0x%04x",
             rc, g_filecmd_handle);
    ErrLog_Write(buf);
    if (rc != 0) return -9;
  }

  /* FileData char — notify, max 244 bytes (max MTU - ATT overhead) */
  {
    uint8_t p[26];
    int i = 0;
    p[i++] = (uint8_t)(g_svc_handle & 0xFF);
    p[i++] = (uint8_t)(g_svc_handle >> 8);
    p[i++] = 0x02;
    memcpy(&p[i], BLE_FILEDATA_UUID, 16); i += 16;
    p[i++] = 244; p[i++] = 0;
    p[i++] = BLE_PROP_NOTIFY;
    p[i++] = 0x00;
    p[i++] = 0x00;                            /* no per-write events */
    p[i++] = 0x10;
    p[i++] = 0x01;
    uint8_t resp[3];
    rc = ble_aci_cmd(ACI_OP_GATT_SRV_ADD_CHAR_NWK, p, (uint16_t)i,
                     resp, sizeof(resp));
    g_filedata_handle = (uint16_t)(resp[0] | (resp[1] << 8));
    snprintf(buf, sizeof(buf), "ble: add_filedata cc=%d h=0x%04x",
             rc, g_filedata_handle);
    ErrLog_Write(buf);
    if (rc != 0) return -10;
  }

  /* SensorStream char — notify, 46-byte packed all-sensor snapshot */
  {
    uint8_t p[26];
    int i = 0;
    p[i++] = (uint8_t)(g_svc_handle & 0xFF);
    p[i++] = (uint8_t)(g_svc_handle >> 8);
    p[i++] = 0x02;
    memcpy(&p[i], BLE_SENSORSTREAM_UUID, 16); i += 16;
    p[i++] = STREAM_PACKET_SIZE; p[i++] = 0;
    p[i++] = BLE_PROP_NOTIFY;
    p[i++] = 0x00;
    p[i++] = 0x00;
    p[i++] = 0x10;
    p[i++] = 0x01;
    uint8_t resp[3];
    rc = ble_aci_cmd(ACI_OP_GATT_SRV_ADD_CHAR_NWK, p, (uint16_t)i,
                     resp, sizeof(resp));
    g_stream_handle = (uint16_t)(resp[0] | (resp[1] << 8));
    snprintf(buf, sizeof(buf), "ble: add_stream cc=%d h=0x%04x",
             rc, g_stream_handle);
    ErrLog_Write(buf);
    if (rc != 0) return -14;
  }

  /* ACI_GAP_SET_ADVERTISING_CONFIGURATION — extended-advertising config.
     Lifted from ble_manager.c::set_connectable_ble (BlueNRG-LP path).
     Payload layout (27 bytes total, little-endian):
       advertising_handle (1)      = 0
       discoverable_mode  (1)      = 0x02 (general discoverable)
       event_properties   (2)      = 0x0013 (CONNECTABLE|SCANNABLE|LEGACY)
       interval_min       (4)      = 0xA0 (160 × 0.625 ms = 100 ms)
       interval_max       (4)      = 0xA0
       channel_map        (1)      = 0x07 (all 3 channels)
       peer_address_type  (1)      = 0x00
       peer_address       (6)      = 0
       filter_policy      (1)      = 0x00 (allow any)
       tx_power           (1)      = 0 (0 dBm)
       primary_phy        (1)      = 0x01 (LE_1M)
       secondary_max_skip (1)      = 0
       secondary_phy      (1)      = 0x01 (LE_1M; unused for legacy)
       advertising_sid    (1)      = 0
       scan_req_notif_en  (1)      = 0
  */
  {
    uint8_t p[27] = {0};
    int i = 0;
    p[i++] = 0x00;                                      /* handle */
    p[i++] = 0x02;                                      /* general discoverable */
    p[i++] = 0x13; p[i++] = 0x00;                       /* event_properties LE */
    p[i++] = 0xA0; p[i++] = 0x00; p[i++] = 0x00; p[i++] = 0x00;  /* interval_min */
    p[i++] = 0xA0; p[i++] = 0x00; p[i++] = 0x00; p[i++] = 0x00;  /* interval_max */
    p[i++] = 0x07;                                      /* channel_map */
    p[i++] = 0x00;                                      /* peer_addr_type */
    for (int k = 0; k < 6; k++) p[i++] = 0;             /* peer_addr */
    p[i++] = 0x00;                                      /* filter_policy */
    p[i++] = 0x00;                                      /* tx_power 0 dBm */
    p[i++] = 0x01;                                      /* primary phy 1M */
    p[i++] = 0x00;                                      /* secondary skip */
    p[i++] = 0x01;                                      /* secondary phy 1M */
    p[i++] = 0x00;                                      /* SID */
    p[i++] = 0x00;                                      /* scan_req_notif */
    rc = ble_aci_cmd(ACI_OP_GAP_SET_ADV_CONFIGURATION, p, (uint16_t)i, NULL, 0);
    snprintf(buf, sizeof(buf), "ble: adv_config cc=%d", rc);
    ErrLog_Write(buf);
    if (rc != 0) return -11;
  }

  /* ACI_GAP_SET_ADVERTISING_DATA_NWK
       handle (1) = 0
       operation (1) = 0x03 (complete data)
       adv_data_length (1)
       adv_data (N)
     AD structures inside adv_data:
       AD #1 Flags: [0x02, 0x01, 0x06] (LE general + BR/EDR off)
       AD #2 Name : [name_len+1, 0x09, 'P','u','m','p','T','s','u','e','r','i']
  */
  {
    uint8_t p[64];
    int i = 0;
    p[i++] = 0x00;                                      /* handle */
    p[i++] = 0x03;                                      /* complete data */
    uint8_t name_len = (uint8_t)BLE_ADV_NAME_LEN;
    uint8_t data_len = 3 + (2 + name_len);
    p[i++] = data_len;                                  /* adv_data_length */
    /* Flags */
    p[i++] = 0x02; p[i++] = 0x01; p[i++] = 0x06;
    /* Complete Local Name */
    p[i++] = (uint8_t)(1 + name_len); p[i++] = 0x09;
    memcpy(&p[i], BLE_ADV_NAME, name_len); i += name_len;
    rc = ble_aci_cmd(ACI_OP_GAP_SET_ADV_DATA_NWK, p, (uint16_t)i, NULL, 0);
    snprintf(buf, sizeof(buf), "ble: adv_data cc=%d", rc);
    ErrLog_Write(buf);
    if (rc != 0) return -12;
  }

  /* First advertising enable — adv_config + adv_data above only need to be
     set once; re-enabling later (after a disconnect) just needs this call. */
  rc = ble_adv_enable();
  snprintf(buf, sizeof(buf), "ble: adv_enable cc=%d", rc);
  ErrLog_Write(buf);
  if (rc != 0) return -13;

  g_advertising = 1;
  ErrLog_Write("ble: advertising as " BLE_ADV_NAME);
  return 0;
}

/* Emit one SensorStream snapshot if a client is subscribed and the 2 s
   period has elapsed. Reuses the logger's cached samples — no sensor I/O,
   no mode switch, never touches SD logging. */
static void ble_stream_tick(void)
{
  if (!g_stream_subscribed || g_conn_handle == 0) return;
  uint32_t now = HAL_GetTick();
  if ((now - g_stream_last_ms) < STREAM_PERIOD_MS) return;
  g_stream_last_ms = now;

  PL_Snapshot snap;
  Logger_GetSnapshot(&snap);
  uint8_t pkt[STREAM_PACKET_SIZE];
  Stream_Pack(&snap, (uint8_t)Logger_IsActive(), pkt);
  /* Non-blocking single attempt — if the TX buffer is full we just skip
     this 2 s slot rather than stall the main loop. */
  ble_notify(g_stream_handle + 1, pkt, STREAM_PACKET_SIZE);
}

void BLE_Tick(void)
{
  if (!g_advertising) return;

  /* Stream emit runs every tick regardless of pending events — it's a
     timer, not an event response. */
  ble_stream_tick();

  if (!irq_high()) return;

  uint8_t evt[260];
  int n = ble_hci_recv(evt, sizeof(evt));
  if (n < 3) return;

  char buf[96];

  if (evt[0] == 0x04) {
    /* Standard HCI event: 04 <code> <plen8> <payload>. The chip delivers
       LE Meta (connection) and Disconnection_Complete this way. */
    uint8_t code = evt[1];
    if (code == 0x3E) {
      /* LE Meta — evt[3] = subevent. BT 5.0 extended advertising delivers
         connections as Enhanced Connection Complete (0x0A), legacy = 0x01. */
      if (n >= 7 && (evt[3] == 0x01 || evt[3] == 0x0A)) {
        uint8_t  st = evt[4];
        uint16_t ch = (uint16_t)(evt[5] | (evt[6] << 8));
        g_conn_handle = ch;
        snprintf(buf, sizeof(buf), "ble: connected st=%d conn=0x%04x sub=0x%02x",
                 st, ch, evt[3]);
        ErrLog_Write(buf);
      }
    } else if (code == 0x05) {
      /* Disconnection_Complete: [3]=status [4..5]=conn_handle [6]=reason.
         Clear the stream subscription too — per BLE spec a non-bonded
         client's CCCD resets on disconnect, so the next connection starts
         unsubscribed and the host's app re-subscribes explicitly. Without
         this the box would keep firing notifications the new client never
         asked for. */
      g_conn_handle       = 0;
      g_stream_subscribed = 0;
      /* Re-arm advertising — the chip stopped it on connect and won't
         resume on its own. Without this the box is invisible to any
         further connection attempt. */
      int arc = ble_adv_enable();
      snprintf(buf, sizeof(buf), "ble: disconnected reason=0x%02x re-adv=%d",
               (n >= 7) ? evt[6] : 0, arc);
      ErrLog_Write(buf);
    }
  } else if (evt[0] == 0x82) {
    /* Extended HCI event: 82 <code> <plen16> <payload>. BlueNRG-LP sends
       ALL ACI vendor events this way (Build #21 raw dump confirmed). */
    if (evt[1] == 0xFF && n >= 6) {
      /* Vendor event — evt[4..5] = ecode (little-endian). */
      uint16_t ecode = (uint16_t)(evt[4] | (evt[5] << 8));
      /* ACI_GATT_SRV_ATTRIBUTE_MODIFIED_EVENT = 0x0C01.
         Payload after ecode: conn_handle(2) attr_handle(2)
         attr_data_length(2) attr_data[]:
           evt[6..7]   = conn_handle
           evt[8..9]   = attr_handle
           evt[10..11] = attr_data_length
           evt[12..]   = attr_data */
      if (ecode == 0x0C01 && n >= 12) {
        uint16_t attr = (uint16_t)(evt[8]  | (evt[9]  << 8));
        uint16_t dlen = (uint16_t)(evt[10] | (evt[11] << 8));
        uint8_t  op   = (n >= 13) ? evt[12] : 0;
        snprintf(buf, sizeof(buf), "ble: write attr=0x%04x dlen=%u op=0x%02x",
                 attr, dlen, op);
        ErrLog_Write(buf);
        /* If the write landed on the FileCmd value handle (declaration
           handle + 1), capture the payload for ble_process_command(). */
        if (attr == (uint16_t)(g_filecmd_handle + 1) && dlen > 0) {
          uint16_t cap = dlen;
          if (cap > sizeof(g_cmd_buf)) cap = sizeof(g_cmd_buf);
          if ((int)(12 + cap) <= n) {
            memcpy(g_cmd_buf, &evt[12], cap);
            g_cmd_len     = (uint8_t)cap;
            g_cmd_pending = 1;
          }
        }
        /* SensorStream CCCD (char declaration + 2): host writes 0x0001 to
           subscribe, 0x0000 to unsubscribe. That's the only thing that
           gates the stream — no opcode, no mode. */
        else if (attr == (uint16_t)(g_stream_handle + 2) && dlen >= 1) {
          g_stream_subscribed = (evt[12] & 0x01);
          g_stream_last_ms    = HAL_GetTick();   /* first packet after the period */
          snprintf(buf, sizeof(buf), "ble: stream subscribed=%d",
                   g_stream_subscribed);
          ErrLog_Write(buf);
        }
      } else {
        snprintf(buf, sizeof(buf), "ble: vendor ecode=0x%04x n=%d", ecode, n);
        ErrLog_Write(buf);
      }
    }
  }

  /* Run any FileSync command captured above. Done outside the event-parse
     block so the SD reads + notifications aren't tangled with event decode. */
  ble_process_command();
}

uint8_t BLE_IsAdvertising(void) { return g_advertising; }
