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
#define CFG_TUSB_RHPORT0_MODE     (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

/* OS abstraction. tud_task() is driven from the cooperative main loop
   (no RTOS), polled at ~1 kHz. TinyUSB's none-OS variant uses simple
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

/* Device classes ---------------------------------------------------------- */
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

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
