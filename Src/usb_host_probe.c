/**
  ******************************************************************************
  * @file    usb_host_probe.c
  * @brief   USB host-mode connect probe — diagnostic build (USB_HOST_PROBE=1).
  *
  *          See usb_host_probe.h for the why. In short: force OTG_FS to host
  *          mode and poll HPRT.PCSTS to learn whether the box's USB
  *          controller sees a plugged-in device on the bus.
  *
  *          The clock / PHY bring-up (VDDUSB island, HSI48 -> ICLK, CRS,
  *          OTG_FS clock, PA11/PA12 AF10, GUSBCFG.PHYSEL) is the SAME proven
  *          STM32U5 sequence used by usb_msc.c::usb_msc_hw_init — reproduced
  *          here (rather than shared) so the production MSC path stays byte-
  *          for-byte untouched and this file is a self-contained diagnostic.
  *          From there it diverges: instead of tud_rhport_init (device), it
  *          does the host-mode core init (FHMOD, GCCFG, HCFG, port power).
  ******************************************************************************
  */
#include <stdbool.h>

#include "stm32u5xx_hal.h"

#include "usb_host_probe.h"
#include "errlog.h"
#include "buzzer.h"

/* Host register access. USB_OTG_FS is the global core base (CMSIS). The host
   sub-block and the single host-port register sit at fixed offsets. */
#define USBx            USB_OTG_FS
#define USBx_HOST       ((USB_OTG_HostTypeDef *)((uint32_t)USBx + USB_OTG_HOST_BASE))
#define USBx_HPRT0      (*(__IO uint32_t *)((uint32_t)USBx + USB_OTG_HOST_PORT_BASE))

/* HPRT carries write-1-to-clear "change" bits and the write-1-to-disable
   PENA bit alongside the read/write control bits. Any read-modify-write of
   HPRT must mask these off, or we'd accidentally clear a connect-detect or
   disable the port. */
#define HPRT_WC1   (USB_OTG_HPRT_PENA | USB_OTG_HPRT_PCDET | \
                    USB_OTG_HPRT_PENCHNG | USB_OTG_HPRT_POCCHNG)

static bool     s_connected     = false;   /* last sampled PCSTS */
static bool     s_oca_latched   = false;   /* overcurrent reported once */
static bool     s_ever_seen     = false;
static uint32_t s_t_init        = 0;
static uint32_t s_last_dump     = 0;

/* Bounded spin on a core register bit so a misconfigured clock can't hang
   the boot forever (the 8 s IWDG would catch it, but bounded is cleaner).
   Returns false on timeout. */
static bool wait_bit_clear(volatile uint32_t *reg, uint32_t mask, uint32_t ms)
{
  uint32_t t0 = HAL_GetTick();
  while ((*reg & mask) != 0U) {
    if ((HAL_GetTick() - t0) > ms) return false;
  }
  return true;
}
static bool wait_bit_set(volatile uint32_t *reg, uint32_t mask, uint32_t ms)
{
  uint32_t t0 = HAL_GetTick();
  while ((*reg & mask) == 0U) {
    if ((HAL_GetTick() - t0) > ms) return false;
  }
  return true;
}

/* ---- clock / PHY bring-up: identical to usb_msc.c steps 1-5 + 7 ---------- */
static void usb_clock_phy_init(void)
{
  /* 1. VDDUSB power island (needs PWR clock first, or SVMCR write no-ops). */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddUSB();

  /* 2. HSI48 -> ICLK (USB intermediate clock). */
  {
    RCC_PeriphCLKInitTypeDef pc = {0};
    pc.PeriphClockSelection = RCC_PERIPHCLK_ICLK;
    pc.IclkClockSelection   = RCC_ICLK_CLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&pc) != HAL_OK) {
      ErrLog_Write("usbh: ICLK clk config FAIL");
    }
  }

  /* 3. CRS auto-trim of HSI48 (off USB SOF once a host/SOF is present;
     harmless in host mode — we generate SOF, CRS just stays idle). */
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

  /* 4. OTG_FS peripheral clock. */
  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

  /* 5. PA11 (D-) / PA12 (D+) on AF10. */
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

  /* 7. Select the FS embedded PHY (must precede the core reset). */
  USBx->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;

  /* NOTE: deliberately NO NVIC enable here. The probe is polled — the
     OTG_FS interrupt stays masked so the stm32u5xx_it.c handler (which
     forwards to TinyUSB device) never fires in this build. */
}

bool UsbHostProbe_Init(void)
{
  usb_clock_phy_init();

  /* Wait for AHB master idle, then soft-reset the core. */
  if (!wait_bit_set(&USBx->GRSTCTL, USB_OTG_GRSTCTL_AHBIDL, 200)) {
    ErrLog_Write("*** usbh: core not idle (clock/power?)");
    ErrLog_Flush();
    return false;
  }
  USBx->GRSTCTL |= USB_OTG_GRSTCTL_CSRST;
  if (!wait_bit_clear(&USBx->GRSTCTL, USB_OTG_GRSTCTL_CSRST, 200)) {
    ErrLog_Write("*** usbh: core soft-reset stuck");
    ErrLog_Flush();
    return false;
  }
  /* Settling time after reset before touching the PHY. */
  HAL_Delay(2);

  /* Activate the transceiver (PWRDWN=1 means "powered up" on this core) and
     DISABLE VBUS sensing. The board has no VBUS-sense pin wired to the host
     path, so with VBDEN set the core would think VBUS is invalid and refuse
     to bring the port up. Clearing VBDEN makes the core treat VBUS as
     permanently valid. */
  USBx->GCCFG |= USB_OTG_GCCFG_PWRDWN;
  USBx->GCCFG &= ~USB_OTG_GCCFG_VBDEN;

  /* Force host mode. Datasheet requires >=25 ms for the role swap to take
     (the core re-runs its mode-dependent reset internally). */
  USBx->GUSBCFG &= ~USB_OTG_GUSBCFG_FDMOD;
  USBx->GUSBCFG |=  USB_OTG_GUSBCFG_FHMOD;
  HAL_Delay(50);

  /* Host clock: FS/LS PHY clock select = 48 MHz (FSLSPCS = 0b01). */
  USBx_HOST->HCFG = (USBx_HOST->HCFG & ~USB_OTG_HCFG_FSLSPCS) | 1U;

  /* Drive port power. On a board with a VBUS load-switch this asserts it;
     on this board there is no switch, so it is a harmless no-op electrically
     — but it is the correct host-mode step and lets PPWR read back set. */
  {
    uint32_t hprt = USBx_HPRT0 & ~HPRT_WC1;
    hprt |= USB_OTG_HPRT_PPWR;
    USBx_HPRT0 = hprt;
  }

  s_t_init    = HAL_GetTick();
  s_last_dump = s_t_init;
  s_connected = (USBx_HPRT0 & USB_OTG_HPRT_PCSTS) ? true : false;

  ErrLog_Writef("usbh: probe armed (host mode) hprt=%08lx", (unsigned long)USBx_HPRT0);
  ErrLog_Flush();

  /* Audible "this is the HOST-PROBE build" signature: 4 quick low beeps. */
  for (int i = 0; i < 4; i++) { Buzzer_Beep(1000U, 50U); HAL_Delay(40); }

  return true;
}

void UsbHostProbe_Tick(void)
{
  uint32_t now  = HAL_GetTick();
  uint32_t hprt = USBx_HPRT0;

  bool conn = (hprt & USB_OTG_HPRT_PCSTS) != 0U;
  uint8_t lsts = (uint8_t)((hprt & USB_OTG_HPRT_PLSTS) >> USB_OTG_HPRT_PLSTS_Pos);
  bool oca  = (hprt & USB_OTG_HPRT_POCA) != 0U;

  /* Connect edge — the answer we built this for. */
  if (conn && !s_connected) {
    s_ever_seen = true;
    ErrLog_Writef("*** usbh: DEVICE DETECTED hprt=%08lx plsts=%u (D%c)",
                  (unsigned long)hprt, lsts,
                  (lsts & 0x1) ? '+' : '-');   /* PLSTS b0=D+, b1=D- line hi */
    ErrLog_Flush();
    for (int i = 0; i < 3; i++) { Buzzer_Beep(2600U, 70U); HAL_Delay(45); }
  } else if (!conn && s_connected) {
    ErrLog_Writef("usbh: device removed hprt=%08lx", (unsigned long)hprt);
    ErrLog_Flush();
    Buzzer_Beep(1200U, 200U);
  }
  s_connected = conn;

  /* Overcurrent (would mean the board DID try to source power and tripped). */
  if (oca && !s_oca_latched) {
    s_oca_latched = true;
    ErrLog_Writef("*** usbh: VBUS OVERCURRENT hprt=%08lx", (unsigned long)hprt);
    ErrLog_Flush();
  } else if (!oca) {
    s_oca_latched = false;
  }

  /* Periodic HPRT trace into the errlog so the full port evolution is on the
     SD card to read back: 1 Hz for the first 30 s, then every 5 s. */
  uint32_t period = ((now - s_t_init) < 30000U) ? 1000U : 5000U;
  if ((now - s_last_dump) >= period) {
    s_last_dump = now;
    ErrLog_Writef("usbh: hprt=%08lx pcsts=%u plsts=%u ppwr=%u poca=%u",
                  (unsigned long)hprt,
                  conn ? 1U : 0U, lsts,
                  (hprt & USB_OTG_HPRT_PPWR) ? 1U : 0U,
                  oca ? 1U : 0U);
  }
}

bool UsbHostProbe_EverSawDevice(void)
{
  return s_ever_seen;
}
