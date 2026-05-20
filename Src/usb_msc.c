/**
  ******************************************************************************
  * @file    usb_msc.c
  * @brief   USB Mass Storage Class device for PumpLogger.
  *
  *          Phase 9 / Issue #5. Brings up the STM32U585's OTG_FS in device
  *          mode and presents the microSD as a USB bulk-only MSC drive,
  *          bridging SCSI READ(10)/WRITE(10) directly to HAL_SD_*Blocks.
  *
  *          The host PC handles FatFs itself — we only ferry raw 512-byte
  *          blocks. This sidesteps any "two filesystem drivers on one
  *          card" problem AT THE PROTOCOL LEVEL, but the PumpLogger
  *          firmware must still pause its own FatFs writes while the host
  *          has MSC mounted — see tud_mount_cb below.
  *
  *          Bring-up reference: fp-sns-stbox1/SDDataLogFileX/Core/Src/
  *          usb_cdc.c. All STM32U5 USB gotchas (VDDUSB power island,
  *          HSI48 + CRS routing, manual PHYSEL select) are reproduced
  *          here verbatim.
  ******************************************************************************
  */
#include <stdbool.h>
#include <string.h>

#include "stm32u5xx_hal.h"
#include "tusb.h"

#include "usb_msc.h"
#include "sd_fatfs.h"
#include "errlog.h"
#include "logger.h"

/* IRQ counter — incremented in stm32u5xx_it.c::OTG_FS_IRQHandler. Useful
   for the "is the USB bus alive?" diagnostic LED pattern. */
volatile uint32_t g_otg_fs_irq_count = 0;

static volatile bool s_mounted      = false;
static volatile bool s_ever_mounted = false;

/* ---------------------------------------------------------------------------
 * USB peripheral hardware bring-up
 *
 * Reproduces the proven SDDataLogFileX/usb_cdc.c sequence — every step is
 * load-bearing on STM32U5:
 *   1. PWR clock + VDDUSB rail (without this, transceiver line floats)
 *   2. HSI48 → ICLK for USB (HSI48 is already ON from SystemClock_Config)
 *   3. CRS auto-trim of HSI48 from USB SOF (HSI48 alone is too loose for FS)
 *   4. OTG_FS peripheral clock
 *   5. GPIO PA11/PA12 on AF10 (USB_OTG_FS)
 *   6. NVIC priority for OTG_FS_IRQn
 *   7. GUSBCFG.PHYSEL = 1 (manual; TinyUSB's dwc2 init doesn't set this)
 * --------------------------------------------------------------------------*/
static void usb_msc_hw_init(void)
{
  /* Step 1. VDDUSB power island. STM32U5 puts the USB transceiver behind
     a separate supply rail — without this, OTG_FS register writes succeed
     but the lines stay floating and no host enumerates. HAL_PWREx_* writes
     to PWR->SVMCR, which requires the PWR peripheral clock — without
     __HAL_RCC_PWR_CLK_ENABLE() the SVMCR write silently no-ops and the
     symptom is "host sees zero bus events even though tud_rhport_init
     returns OK." */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddUSB();

  /* Step 2. Route HSI48 to the USB / SDMMC / RNG intermediate clock (ICLK).
     STM32U5 routes all three through one selector — SDMMC is unaffected
     because it has its own clock setup in sd_fatfs.c. HSI48 is already
     ON from SystemClock_Config (RCC_OSCILLATORTYPE_HSI48 set there). */
  {
    RCC_PeriphCLKInitTypeDef pc = {0};
    pc.PeriphClockSelection = RCC_PERIPHCLK_ICLK;
    pc.IclkClockSelection   = RCC_ICLK_CLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&pc) != HAL_OK) {
      ErrLog_Write("usb: ICLK clk config FAIL");
    }
  }

  /* Step 3. CRS auto-trim of HSI48 from USB SOF. HSI48 alone is ±1-2 %
     out of reset, too loose for USB FS spec (±0.25 % during enumeration).
     CRS locks HSI48 to the host's 1 kHz USB SOF token so the line rate
     stays in spec the moment a host plugs in. Without CRS the host may
     not see SE0/J/K transitions cleanly and enumeration silently fails. */
  __HAL_RCC_CRS_CLK_ENABLE();
  {
    RCC_CRSInitTypeDef crs = {0};
    crs.Prescaler             = RCC_CRS_SYNC_DIV1;
    crs.Source                = RCC_CRS_SYNC_SOURCE_USB;
    crs.Polarity              = RCC_CRS_SYNC_POLARITY_RISING;
    crs.ReloadValue           = RCC_CRS_RELOADVALUE_DEFAULT;
    crs.ErrorLimitValue       = RCC_CRS_ERRORLIMIT_DEFAULT;
    crs.HSI48CalibrationValue = RCC_CRS_HSI48CALIBRATION_DEFAULT;
    HAL_RCCEx_CRSConfig(&crs);
  }

  /* Step 4. OTG_FS peripheral clock. STM32U5 has both an FS and an HS
     path; we use FS (HS PHY needs an external transceiver). */
  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

  /* Step 5. GPIO PA11 (D-) / PA12 (D+) on AF10 (USB_OTG_FS). */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  {
    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_11 | GPIO_PIN_12;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF10_USB;
    HAL_GPIO_Init(GPIOA, &g);
  }

  /* Step 6. Arm OTG_FS interrupt. Priority 13 — between SDMMC1 (14, the
     polled HAL_SD_*Blocks calls actually run with IRQs masked, but we
     keep the slot reserved for future DMA work) and UART4-GPS (6) so a
     USB burst can't drop NMEA bytes. tud_int_handler bound in
     stm32u5xx_it.c. */
  HAL_NVIC_SetPriority(OTG_FS_IRQn, 13, 0);
  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);

  /* Step 7. CRUCIAL: select the FS embedded PHY via GUSBCFG.PHYSEL.
     STM32U585's OTG_FS needs this bit set BEFORE the core reset so the
     controller knows which PHY interface to bring up; without it the
     data lines stay floating and no host enumerates. ST's HAL_PCD_Init
     does this in USB_CoreInit, but TinyUSB's dwc2_phy_init doesn't, so
     we set it manually. Set AFTER the OTG_FS RCC clock is enabled (we
     did that above) so the register is actually writable. */
  USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;
}

bool UsbMsc_Init(void)
{
  usb_msc_hw_init();

  /* Non-deprecated form of tud_init(0). Returns false if the dwc2 core
     fails its synopsys-id check (= clock/power not enabled correctly). */
  const tusb_rhport_init_t rh_init = {
    .role  = TUSB_ROLE_DEVICE,
    .speed = TUSB_SPEED_FULL,
  };
  bool ok = tud_rhport_init(0, &rh_init);
  if (!ok) {
    ErrLog_Write("usb: tud_rhport_init FAIL");
  }
  return ok;
}

void UsbMsc_Tick(void)
{
  tud_task();
}

bool UsbMsc_IsMounted(void)   { return s_mounted; }
bool UsbMsc_EverMounted(void) { return s_ever_mounted; }

/* ===========================================================================
 * TinyUSB device-state callbacks
 * ==========================================================================*/

/* Host has set the device configuration. The MSC bulk endpoints become
   addressable and the host's MSC class driver will start polling for
   READY status. From the logger's perspective this is the moment to
   stop writing to the SD card — we now have a competing block-level
   user. */
void tud_mount_cb(void)
{
  s_mounted = true;
  s_ever_mounted = true;
  ErrLog_Write("usb: mounted — pausing logger");
  /* Flush + close any open session, then mark inactive. Logger_Stop is
     idempotent so it's safe to call even if no session is open. */
  Logger_Stop();
}

void tud_umount_cb(void)
{
  s_mounted = false;
  ErrLog_Write("usb: unmounted — resuming logger");
  /* Resume logging. In AUTO mode this opens a fresh session immediately.
     In MANUAL mode it stays idle until the next START_LOG. */
  if (Logger_GetMode() != LOGGER_MODE_MANUAL) {
    if (Logger_Init() != 0) {
      ErrLog_Write("usb: post-unmount logger init FAIL");
    }
  }
}

/* Bus suspended (no SOF for 3 ms). Treat as soft-disconnect for safety —
   the host may be entering selective suspend. We do NOT resume logging
   here: only an explicit unmount or a full bus reset transitions us
   back. This is conservative; a re-mount on resume re-fires tud_mount_cb
   without ill effect. */
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
}

void tud_resume_cb(void)
{
  /* No-op — s_mounted reflects host SCSI activity, not bus power. */
}

/* ===========================================================================
 * MSC SCSI callbacks (bulk-only transport)
 *
 * Spec: USB Mass Storage Class - Bulk Only Transport rev 1.0 + SCSI-2
 * READ(10)/WRITE(10)/INQUIRY/READ_CAPACITY(10)/TEST_UNIT_READY. TinyUSB
 * handles BBB framing and SCSI command parsing; we implement the
 * vendor-side data fetch.
 * ==========================================================================*/

/* SCSI INQUIRY response — three short ASCII fields. Padding-with-spaces
   is required by the spec (no NUL terminator). */
void tud_msc_inquiry_cb(uint8_t lun, uint8_t vendor_id[8],
                        uint8_t product_id[16], uint8_t product_rev[4])
{
  (void) lun;
  const char vid[]  = "ywesee  ";          /* exactly 8 */
  const char pid[]  = "PumpLogger SD   ";   /* exactly 16 */
  const char rev[]  = "0001";               /* exactly 4 */
  memcpy(vendor_id,  vid, 8);
  memcpy(product_id, pid, 16);
  memcpy(product_rev, rev, 4);
}

/* TEST UNIT READY — the host polls this once per second. We're ready
   whenever the SD card is mounted in our own FatFs sense (= SDMMC up).
   If the card is missing the host should see "not ready" and refrain
   from issuing READ/WRITE — though in practice on this board the card
   absence is fatal at boot so we never see that branch. */
bool tud_msc_test_unit_ready_cb(uint8_t lun)
{
  (void) lun;
  return SDFat_IsMounted();
}

/* READ CAPACITY(10) — host learns the block count and block size. */
void tud_msc_capacity_cb(uint8_t lun, uint32_t *block_count, uint16_t *block_size)
{
  (void) lun;
  SD_HandleTypeDef *hsd = SDFat_RawHandle();
  if (hsd == NULL) {
    *block_count = 0;
    *block_size  = 512;
    return;
  }
  HAL_SD_CardInfoTypeDef info;
  if (HAL_SD_GetCardInfo(hsd, &info) != HAL_OK) {
    *block_count = 0;
    *block_size  = 512;
    return;
  }
  *block_count = info.LogBlockNbr;
  *block_size  = (uint16_t) info.LogBlockSize;
}

/* START STOP UNIT — the host issues this on eject. start=false means
   "spin down / eject." We treat it as a hint that the host is about to
   unmount; the actual unmount callback (tud_umount_cb) is what really
   drives logger resume, so this is mostly informational. Return true to
   acknowledge — returning false would stall the host. */
bool tud_msc_start_stop_cb(uint8_t lun, uint8_t power_condition,
                           bool start, bool load_eject)
{
  (void) lun; (void) power_condition; (void) load_eject;
  if (!start) {
    ErrLog_Write("usb: host eject");
  }
  return true;
}

/* READ(10) — host wants `bufsize` bytes starting at LBA `lba + offset/512`.
   TinyUSB chunks long transfers and calls us per chunk; `offset` is the
   byte offset WITHIN the current LBA's data payload that this chunk
   starts at. In practice offset is always a multiple of 512 because we
   advertise 512-byte logical blocks, and `bufsize` is always a multiple
   of 512. */
int32_t tud_msc_read10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                          void *buffer, uint32_t bufsize)
{
  (void) lun;
  SD_HandleTypeDef *hsd = SDFat_RawHandle();
  if (hsd == NULL) return -1;

  /* Compute the actual starting LBA. offset is in bytes, not blocks. */
  uint32_t start_lba   = lba + (offset / 512u);
  uint32_t block_count = bufsize / 512u;
  if (block_count == 0) return 0;

  /* Wait for any prior write to complete. HAL_SD_*Blocks is synchronous
     but the card itself takes time to leave the PROGRAMMING state. */
  uint32_t guard = 0;
  while (HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER) {
    if (++guard > 1000000u) return -1;
  }

  if (HAL_SD_ReadBlocks(hsd, (uint8_t *) buffer, start_lba, block_count, 1000) != HAL_OK) {
    return -1;
  }

  /* Wait for the read to fully complete before reporting bytes back. */
  guard = 0;
  while (HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER) {
    if (++guard > 1000000u) return -1;
  }

  return (int32_t)(block_count * 512u);
}

/* WRITE(10) — host wants to write `bufsize` bytes to LBA `lba + offset/512`. */
int32_t tud_msc_write10_cb(uint8_t lun, uint32_t lba, uint32_t offset,
                           uint8_t *buffer, uint32_t bufsize)
{
  (void) lun;
  SD_HandleTypeDef *hsd = SDFat_RawHandle();
  if (hsd == NULL) return -1;

  uint32_t start_lba   = lba + (offset / 512u);
  uint32_t block_count = bufsize / 512u;
  if (block_count == 0) return 0;

  uint32_t guard = 0;
  while (HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER) {
    if (++guard > 1000000u) return -1;
  }

  if (HAL_SD_WriteBlocks(hsd, buffer, start_lba, block_count, 1000) != HAL_OK) {
    return -1;
  }

  guard = 0;
  while (HAL_SD_GetCardState(hsd) != HAL_SD_CARD_TRANSFER) {
    if (++guard > 1000000u) return -1;
  }

  return (int32_t)(block_count * 512u);
}

/* Writable? — host calls this on WRITE_PROTECT / MODE_SENSE. We always
   allow writes; the SD card itself has no write-protect tab on this
   board form factor. */
bool tud_msc_is_writable_cb(uint8_t lun)
{
  (void) lun;
  return true;
}

/* Vendor-specific SCSI commands — return -1 so TinyUSB sets the standard
   "INVALID COMMAND" sense data. We don't speak any vendor extensions. */
int32_t tud_msc_scsi_cb(uint8_t lun, uint8_t const scsi_cmd[16],
                        void *buffer, uint16_t bufsize)
{
  (void) lun; (void) scsi_cmd; (void) buffer; (void) bufsize;
  return -1;
}
