/**
  ******************************************************************************
  * @file    usb_gps_host.h
  * @brief   USB-host CDC GNSS receiver (Phase 10, Increment 1).
  *
  *          Brings up OTG_FS in HOST mode and enumerates a USB-CDC GNSS
  *          dongle (u-blox 8 / NEO-M8N class, VID 0x1546, native CDC-ACM).
  *          Streamed NMEA is pushed into gps.c via GPS_FeedBytes(), so the
  *          existing parser, fix logic, SD logging and BLE stream all work
  *          unchanged — the byte source just moved from UART4 to USB.
  *
  *          Build:  make USB_GPS_HOST=1   (fixed host mode; the default
  *          build is the Phase 9 USB-MSC device). One OTG core = one role
  *          per build for now — runtime dual-role (UCPD-gated) is Increment 2.
  *
  *          F-ARCH-7: like the MSC device, host enumeration needs the
  *          OTG_FS interrupt (forwarded to tuh_int_handler) — the same
  *          sanctioned exception. tuh_task() runs cooperatively from the
  *          main loop.
  ******************************************************************************
  */
#ifndef PL_USB_GPS_HOST_H
#define PL_USB_GPS_HOST_H

#include <stdbool.h>
#include <stdint.h>

/* Bring up OTG_FS host mode + TinyUSB host, power the port. Returns false if
   the dwc2 core fails its id check (clock/power not configured). */
bool UsbGpsHost_Init(void);

/* Pump the host stack. Call every main-loop iteration. */
void UsbGpsHost_Tick(void);

/* True while a CDC GNSS device is enumerated and mounted. */
bool UsbGpsHost_IsMounted(void);

/* Running total of NMEA bytes forwarded into gps.c since boot. */
uint32_t UsbGpsHost_BytesForwarded(void);

#endif /* PL_USB_GPS_HOST_H */
