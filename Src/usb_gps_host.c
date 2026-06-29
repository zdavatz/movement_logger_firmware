/**
  ******************************************************************************
  * @file    usb_gps_host.c
  * @brief   USB-host CDC GNSS receiver (Phase 10, Increment 1).
  *
  *          OTG_FS in host mode + TinyUSB host CDC-ACM. Enumerates a USB-CDC
  *          GNSS dongle and forwards its NMEA stream into gps.c::GPS_FeedBytes.
  *
  *          Clock / PHY bring-up is the proven STM32U5 sequence shared with
  *          usb_msc.c and usb_host_probe.c (VDDUSB island, HSI48 -> ICLK,
  *          CRS, OTG_FS clock, PA11/PA12 AF10, GUSBCFG.PHYSEL). The host-
  *          specific VBUS bring-up (disable VBUS sensing so the port comes up
  *          without a VBUS-sense pin, then power the port) is the same set of
  *          register writes the host-probe used to successfully detect and
  *          power the dongle on real hardware.
  ******************************************************************************
  */
#include <stdbool.h>
#include <stdint.h>

#include "stm32u5xx_hal.h"
#include "tusb.h"
#include "host/hcd.h"   /* hcd_event_device_attach() inline — missed-edge rescue */

#include "usb_gps_host.h"
#include "gps.h"
#include "errlog.h"
#include "buzzer.h"

#define USBx        USB_OTG_FS
#define USBx_HOST   ((USB_OTG_HostTypeDef *)((uint32_t)USBx + USB_OTG_HOST_BASE))
#define USBx_HPRT0  (*(__IO uint32_t *)((uint32_t)USBx + USB_OTG_HOST_PORT_BASE))
#define HPRT_WC1    (USB_OTG_HPRT_PENA | USB_OTG_HPRT_PCDET | \
                     USB_OTG_HPRT_PENCHNG | USB_OTG_HPRT_POCCHNG)

/* TinyUSB host (OPT_OS_NONE) requires the application to provide a 1 ms
   time base for enumeration timeouts/delays — the device stack never called
   this, which is why the MSC build linked without it. SysTick's HAL_GetTick
   is exactly the ms counter TinyUSB wants. */
uint32_t tusb_time_millis_api(void) { return HAL_GetTick(); }

/* Incremented in stm32u5xx_it.c::OTG_FS_IRQHandler (host build). Lets the
   plug-in tracer show whether plugging the dongle in fires ANY OTG interrupt
   — i.e. whether the core sees an electrical bus event at all. */
volatile uint32_t g_otg_fs_irq_count = 0;

static volatile bool s_mounted = false;
static volatile uint32_t s_bytes = 0;
static uint8_t  s_cdc_idx = 0xFF;
static uint32_t s_t_init = 0;
static uint32_t s_last_dump = 0;

/* ---- clock / PHY bring-up (same proven sequence as usb_msc.c) ------------ */
static void usb_clock_phy_init(void)
{
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_EnableVddUSB();

  {
    RCC_PeriphCLKInitTypeDef pc = {0};
    pc.PeriphClockSelection = RCC_PERIPHCLK_ICLK;
    pc.IclkClockSelection   = RCC_ICLK_CLKSOURCE_HSI48;
    if (HAL_RCCEx_PeriphCLKConfig(&pc) != HAL_OK) {
      ErrLog_Write("usbg: ICLK clk config FAIL");
    }
  }

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

  __HAL_RCC_USB_OTG_FS_CLK_ENABLE();

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

  /* Host enumeration needs prompt IRQ response (same as the MSC device).
     OTG_FS_IRQHandler forwards to tuh_int_handler in this build. Priority 13
     — below SDMMC, above GPS UART. */
  HAL_NVIC_SetPriority(OTG_FS_IRQn, 13, 0);
  HAL_NVIC_EnableIRQ(OTG_FS_IRQn);

  /* Select the FS embedded PHY (TinyUSB's dwc2 init doesn't). */
  USBx->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL;
}

/* Disable VBUS sensing + assert port power. The board has no VBUS-sense pin
   on the host path; with VBDEN set the core would refuse to bring the port
   up. This is the exact set of writes the host-probe used to detect + power
   the dongle. Applied AFTER tuh_rhport_init so TinyUSB's core reset doesn't
   clobber it. */
static void usb_host_vbus_enable(void)
{
  USBx->GCCFG |= USB_OTG_GCCFG_PWRDWN;     /* transceiver active */
  USBx->GCCFG &= ~USB_OTG_GCCFG_VBDEN;     /* VBUS internally valid */

  /* Force A-session + VBUS valid via GOTGCTL software override. With VBUS
     sensing off (VBDEN=0) the dwc2 HOST path leaves ASVLD=0 and won't run the
     port unless real VBUS exceeds the ~4.4 V A-session threshold. If the
     board's marginal leakage sits BELOW 4.4 V but ABOVE the dongle's 3.3 V
     minimum, the device has enough power yet the core rejects it. Overriding
     A-valid + VBUS-valid lets the host operate regardless of VBUS level.
     TinyUSB's hcd does NOT set this (only its device path forces B-valid),
     so we add it here. Trace will show whether the line then comes alive. */
  USBx->GOTGCTL |= USB_OTG_GOTGCTL_VBVALOEN | USB_OTG_GOTGCTL_VBVALOVAL
                 | USB_OTG_GOTGCTL_AVALOEN  | USB_OTG_GOTGCTL_AVALOVAL;

  uint32_t hprt = USBx_HPRT0 & ~HPRT_WC1;
  hprt |= USB_OTG_HPRT_PPWR;               /* port power on */
  USBx_HPRT0 = hprt;
}

bool UsbGpsHost_Init(void)
{
  usb_clock_phy_init();

  const tusb_rhport_init_t rh_init = {
    .role  = TUSB_ROLE_HOST,
    .speed = TUSB_SPEED_FULL,
  };
  bool ok = tuh_rhport_init(0, &rh_init);
  if (!ok) {
    ErrLog_Write("*** usbg: tuh_rhport_init FAIL");
    ErrLog_Flush();
    return false;
  }

  usb_host_vbus_enable();

  s_t_init    = HAL_GetTick();
  s_last_dump = s_t_init;
  ErrLog_Writef("usbg: host up, awaiting CDC GNSS hprt=%08lx",
                (unsigned long)USBx_HPRT0);
  ErrLog_Flush();
  return true;
}

void UsbGpsHost_Tick(void)
{
  tuh_task();

  /* PLUG-IN EVENT TRACER. Logs a line every time the OTG host/session state
     changes, so the exact electrical sequence when the dongle is plugged in
     is captured (or its absence is). Change-detection is on the STABLE HPRT
     bits (PCSTS/PENA/POCA/PPWR) + GOTGCTL + the OTG IRQ count, so a floating
     D-line's PLSTS flicker doesn't spam — but the current PLSTS (raw D+/D-
     voltage) is included in every logged line. Read GOTGCTL for VBUS/session
     valid; PLSTS for whether the dongle powered up and pulled a data line;
     irq for whether plug-in fired any interrupt at all. */
  {
    static uint32_t s_tr_hprt = 0xFFFFFFFFu, s_tr_gotg = 0xFFFFFFFFu, s_tr_irq = 0xFFFFFFFFu;
    uint32_t hprt = USBx_HPRT0;
    uint32_t gotg = USBx->GOTGCTL;
    uint32_t irq  = g_otg_fs_irq_count;
    uint32_t hprt_stable = hprt & (USB_OTG_HPRT_PCSTS | USB_OTG_HPRT_PENA |
                                   USB_OTG_HPRT_POCA  | USB_OTG_HPRT_PPWR);
    if (hprt_stable != s_tr_hprt || gotg != s_tr_gotg || irq != s_tr_irq) {
      s_tr_hprt = hprt_stable; s_tr_gotg = gotg; s_tr_irq = irq;
      uint8_t plsts = (uint8_t)((hprt & USB_OTG_HPRT_PLSTS) >> USB_OTG_HPRT_PLSTS_Pos);
      ErrLog_Writef("usbg: TR t=%lu hprt=%08lx plsts=%u(D%c%c) gotg=%08lx gccfg=%08lx irq=%lu",
                    (unsigned long)HAL_GetTick(), (unsigned long)hprt, plsts,
                    (plsts & 0x1) ? '+' : '-', (plsts & 0x2) ? '+' : '-',
                    (unsigned long)gotg, (unsigned long)USBx->GCCFG, (unsigned long)irq);
      ErrLog_Flush();
    }
  }

  /* Register-level port-connect detector, independent of TinyUSB. Same
     HPRT.PCSTS poll the host-probe proved reliable. Splits the failure
     into layers: PCSTS asserting = the device is powered + electrically
     detected at the port (VBUS/clock OK); if that fires but tuh never
     mounts, the bug is in enumeration, not power. 1 long beep on the
     PCSTS rising edge. */
  static bool     s_port_seen = false;
  static uint32_t s_pcsts_ms = 0;
  static uint32_t s_last_inject = 0;
  static uint8_t  s_inject_n = 0;
  bool pcsts = (USBx_HPRT0 & USB_OTG_HPRT_PCSTS) != 0U;

  /* VBUS is asserted once in UsbGpsHost_Init and left STEADY — never
     power-cycled. A USB-CDC GNSS dongle (e.g. ELT0380 / NEO-M8N) carries a
     supercap that needs sustained VBUS to charge and boot; interrupting
     power resets that and prevents it from ever asserting its pull-up. So we
     just keep the port powered and wait for the device to come up. (External
     powered hub recommended: the box sourcing VBUS itself is an undesigned,
     intermittent capability — see usb_gps_host.h.) */

  if (pcsts && !s_port_seen) {
    s_port_seen   = true;
    s_pcsts_ms    = HAL_GetTick();
    s_last_inject = 0;
    s_inject_n    = 0;
    ErrLog_Writef("usbg: PORT detect (pcsts=1) hprt=%08lx", (unsigned long)USBx_HPRT0);
    ErrLog_Flush();
    Buzzer_Beep(1500U, 250U);   /* 1 long beep = electrical/port detect */
  } else if (!pcsts && s_port_seen) {
    s_port_seen = false;
    ErrLog_Writef("usbg: port device gone hprt=%08lx", (unsigned long)USBx_HPRT0);
  }

  /* Rescue for a MISSED / FLAKY CONNECT. The dwc2 host only begins
     enumeration from the PCDET interrupt -> hcd_event_device_attach(). When
     the dongle is already attached as the port powers up at boot, that
     rising edge slips past, so PCSTS reads 1 but enumeration never starts —
     and a single re-injection sometimes doesn't take. So once a device is
     detected at the port but not yet mounted, RETRY the attach event every
     ~1.5 s (up to 8 times = ~12 s) until it enumerates. Silent (no beep) —
     success is signalled by the mount (2 beeps) / NMEA (3 beeps) callbacks.
     The !s_mounted gate stops retries the instant it enumerates. */
  if (s_port_seen && !s_mounted &&
      (HAL_GetTick() - s_pcsts_ms) > 600U &&
      (HAL_GetTick() - s_last_inject) > 1500U &&
      s_inject_n < 8U) {
    s_last_inject = HAL_GetTick();
    s_inject_n++;
    ErrLog_Writef("usbg: attach inject #%u (kick enum)", s_inject_n);
    ErrLog_Flush();
    hcd_event_device_attach(0, false);
  }

  /* Low-rate liveness trace every 5 s (errlog only — no beep). The one-shot
     event beeps (mount, first-NMEA) are enough confirmation; the continuous
     heartbeat was a bring-up diagnostic and is removed now that enumeration
     is confirmed working. */
  uint32_t now = HAL_GetTick();
  if ((now - s_last_dump) >= 5000U) {
    s_last_dump = now;
    uint8_t st = !s_port_seen ? 1U : (!s_mounted ? 2U : (s_bytes == 0U ? 3U : 4U));
    ErrLog_Writef("usbg: state=%u mounted=%u bytes=%lu hprt=%08lx",
                  st, s_mounted ? 1U : 0U,
                  (unsigned long)s_bytes, (unsigned long)USBx_HPRT0);
  }
}

bool     UsbGpsHost_IsMounted(void)       { return s_mounted; }
uint32_t UsbGpsHost_BytesForwarded(void)  { return s_bytes; }

/* ---- TinyUSB host CDC callbacks ----------------------------------------- */

void tuh_cdc_mount_cb(uint8_t idx)
{
  s_cdc_idx = idx;
  s_mounted = true;

  uint16_t vid = 0, pid = 0;
  tuh_itf_info_t info;
  if (tuh_cdc_itf_get_info(idx, &info)) {
    tuh_vid_pid_get(info.daddr, &vid, &pid);
  }
  ErrLog_Writef("usbg: CDC GNSS mounted idx=%u %04x:%04x", idx, vid, pid);
  ErrLog_Flush();

  /* Audible "GNSS enumerated" — 2 rising beeps. Immediate field confirmation
     that the host saw + mounted the dongle, no BLE/SD read needed. */
  Buzzer_Beep(1800U, 80U); HAL_Delay(50); Buzzer_Beep(2600U, 80U);

  /* Assert DTR+RTS (line-state bitmask b0=DTR, b1=RTS). The u-blox streams
     regardless, but some CDC stacks gate TX on DTR — harmless to assert. */
  tuh_cdc_set_control_line_state(idx, 0x03, NULL, 0);
}

void tuh_cdc_umount_cb(uint8_t idx)
{
  if (idx == s_cdc_idx) {
    s_mounted = false;
    s_cdc_idx = 0xFF;
  }
  ErrLog_Writef("usbg: CDC GNSS unmounted idx=%u", idx);
  ErrLog_Flush();
}

void tuh_cdc_rx_cb(uint8_t idx)
{
  static bool s_first_data = false;
  uint8_t buf[64];
  uint32_t avail;
  while ((avail = tuh_cdc_read_available(idx)) > 0) {
    uint32_t n = tuh_cdc_read(idx, buf, (avail > sizeof(buf)) ? sizeof(buf) : avail);
    if (n == 0) break;
    GPS_FeedBytes(buf, (uint16_t)n);
    s_bytes += n;
  }
  /* One-shot "NMEA is flowing" — 3 quick high beeps the first time bytes
     actually arrive from the dongle (distinct from the 2-beep mount). This
     is the success signal: enumeration + a live data pipe. */
  if (!s_first_data && s_bytes > 0) {
    s_first_data = true;
    ErrLog_Writef("usbg: first NMEA bytes (%lu)", (unsigned long)s_bytes);
    ErrLog_Flush();
    for (int i = 0; i < 3; i++) { Buzzer_Beep(2800U, 60U); HAL_Delay(40); }
  }
}
