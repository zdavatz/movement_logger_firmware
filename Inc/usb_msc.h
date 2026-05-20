/**
  ******************************************************************************
  * @file    usb_msc.h
  * @brief   USB Mass Storage Class device — exposes the microSD as a USB
  *          drive when the box is plugged into a host over USB-C.
  *
  *          Phase 9 deliverable. See Issue #5 + DESIGN.md "USB MSC" section.
  *
  *          Architecture:
  *            - UsbMsc_Init() in main() after SystemClock_Config + SDFat_Mount.
  *            - UsbMsc_Tick() called from the main loop; drives tud_task().
  *            - OTG_FS_IRQHandler in stm32u5xx_it.c forwards to tud_int_handler(0).
  *            - SCSI READ(10)/WRITE(10) bridge to HAL_SD_*Blocks via the raw
  *              hsd handle exposed by sd_fatfs.c.
  *            - On tud_mount_cb the logger is signaled to suspend SD writes;
  *              on tud_umount_cb it resumes on the next tick.
  ******************************************************************************
  */
#ifndef PUMPLOGGER_USB_MSC_H
#define PUMPLOGGER_USB_MSC_H

#include <stdbool.h>
#include <stdint.h>

/* Bring up VDDUSB, OTG_FS clocks, GPIO, NVIC, and tud_rhport_init.
   Returns true on success. Must be called after SystemClock_Config (HSI48
   on) and after SDFat_Mount (we read block size + count for capacity). */
bool UsbMsc_Init(void);

/* Pump the TinyUSB device task. Cheap; call every main-loop tick. */
void UsbMsc_Tick(void);

/* True iff a USB host has the MSC interface enumerated AND mounted (i.e.
   it has begun issuing SCSI READ/WRITE traffic). The logger uses this to
   gate SD writes. */
bool UsbMsc_IsMounted(void);

/* True iff at any point this boot the host has mounted us. Useful for
   the boot banner / errlog. */
bool UsbMsc_EverMounted(void);

#endif /* PUMPLOGGER_USB_MSC_H */
