/**
  ******************************************************************************
  * @file    tusb_config.h
  * @brief   TinyUSB project configuration for PumpLogger on STM32U585.
  *
  *          USB FS device class = MSC (single-LUN microSD bridge). CDC is
  *          disabled. Phase 9.
  ******************************************************************************
  */
#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* MCU and family — DWC2 OTG_FS on STM32U5. */
#define CFG_TUSB_MCU              OPT_MCU_STM32U5

/* Role select. PL_USB_GPS_HOST (Makefile USB_GPS_HOST=1, Phase 10) builds
   the OTG_FS in HOST mode to read a USB-CDC GNSS receiver. Default 0 =
   DEVICE mode (the Phase 9 USB-MSC SD bridge). One OTG core = one role per
   build for now; runtime dual-role (UCPD-gated) is Increment 2. */
#ifndef PL_USB_GPS_HOST
#define PL_USB_GPS_HOST 0
#endif

#if PL_USB_GPS_HOST
#define CFG_TUSB_RHPORT0_MODE     (OPT_MODE_HOST | OPT_MODE_FULL_SPEED)
#else
#define CFG_TUSB_RHPORT0_MODE     (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#endif

/* OS abstraction. tud_task()/tuh_task() is driven from the cooperative main
   loop (no RTOS), polled at ~1 kHz. TinyUSB's none-OS variant uses simple
   atomic flags between the OTG_FS ISR and the task — exactly the
   producer/consumer shape we want. */
#define CFG_TUSB_OS               OPT_OS_NONE

/* Quiet by default — TinyUSB's debug printf would pull in newlib stdio
   and we have no console here. Flip to 1/2/3 only when debugging USB
   itself and only with a separate SWO/SWD tap. */
#define CFG_TUSB_DEBUG            0

/* Memory placement attributes (defaults are fine for STM32U5; the SRAM
   that TinyUSB allocates is normal contents-preserved memory). */
#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN        __attribute__((aligned(4)))

#define CFG_TUSB_MEM_DMA          1

#if PL_USB_GPS_HOST
/* Host classes (Phase 10) -------------------------------------------------- */
#define CFG_TUH_ENABLED           1
#define CFG_TUD_ENABLED           0
#define CFG_TUH_ENUMERATION_BUFSIZE 256

/* No hub: the GNSS dongle is the one and only downstream device, wired
   straight to the box's single USB-C port. */
#define CFG_TUH_HUB               0
#define CFG_TUH_DEVICE_MAX        1

/* CDC-ACM host. The u-blox 8 presents a native USB-CDC ACM interface
   (bDeviceClass=2), so the stock ACM path applies — no FTDI/CP210x/CH34x
   vendor-serial drivers needed. Push the host-set line coding on enum
   (harmless for the u-blox, which ignores baud over USB). RX FIFO sized to
   absorb a burst of 10 Hz multi-constellation NMEA between task polls. */
#define CFG_TUH_CDC               1
#define CFG_TUH_CDC_FTDI          0
#define CFG_TUH_CDC_CP210X        0
#define CFG_TUH_CDC_CH34X         0
#define CFG_TUH_CDC_LINE_CODING_ON_ENUM { 38400, CDC_LINE_CODING_STOP_BITS_1, CDC_LINE_CODING_PARITY_NONE, 8 }
#define CFG_TUH_CDC_RX_BUFSIZE    512
#define CFG_TUH_CDC_TX_BUFSIZE    64

#else
/* Device classes (Phase 9) ------------------------------------------------- */
#define CFG_TUD_ENABLED           1
#define CFG_TUD_ENDPOINT0_SIZE    64

#define CFG_TUD_CDC               0
#define CFG_TUD_MSC               1
#define CFG_TUD_HID               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_VENDOR            0

/* MSC bulk endpoint buffer. SCSI READ(10) and WRITE(10) ferry 512-byte
   blocks; TinyUSB needs at least one block of FIFO to keep the host
   from stalling. We set it to one full SD block. */
#define CFG_TUD_MSC_EP_BUFSIZE    512
#endif

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
