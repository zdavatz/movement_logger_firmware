/**
  ******************************************************************************
  * @file    stm32u5xx_it.c
  * @brief   Cortex + STM32 system interrupt handlers.
  *
  *          SysTick is the system time base. UART4_IRQHandler is the only
  *          application-level ISR — a deliberate exception to F-ARCH-6 for
  *          GPS byte-by-byte RX. The DMA-based GPS path proved too fragile
  *          (one framing error halted the channel; Build #8/#9 ERRLOG showed
  *          bytes=1 over 6 minutes). The IRQ pattern is lifted from the
  *          original SDDataLogFileX::gps_nmea.c — proven to work, IRQ load
  *          is bounded (<10 µs/byte at 38400 baud = <0.5% CPU).
  ******************************************************************************
  */

#include "main.h"
#include "tusb.h"

void NMI_Handler(void)            { while (1) {} }
void HardFault_Handler(void)      { while (1) {} }
void MemManage_Handler(void)      { while (1) {} }
void BusFault_Handler(void)       { while (1) {} }
void UsageFault_Handler(void)     { while (1) {} }
void SVC_Handler(void)            { }
void DebugMon_Handler(void)       { }
void PendSV_Handler(void)         { }

void SysTick_Handler(void)
{
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

/* UART4 RX IRQ — single source for the GPS byte stream. Delegates to HAL
   which drives the RxCplt/Error callbacks in gps.c. */
void GPS_UART_IRQHandler(void);
void UART4_IRQHandler(void)
{
  GPS_UART_IRQHandler();
}

/* OTG_FS global IRQ — forwarded to TinyUSB.
   F-ARCH-7 sanctioned exception #2 (Phase 9 / Issue #5): USB enumeration
   requires fast (<100 µs) response to bus events; polling the task at the
   1 ms main-loop cadence is not fast enough for the bus reset / set-address
   handshake. NVIC priority set in the USB module's hw init.
   Device build (Phase 9 MSC) → tud_int_handler; host build (Phase 10
   USB-GPS, PL_USB_GPS_HOST) → tuh_int_handler. The host-probe diagnostic
   polls and never arms this IRQ, so the device path is harmless there. */
#if PL_USB_GPS_HOST
extern volatile uint32_t g_otg_fs_irq_count;  /* lives in usb_gps_host.c */
void OTG_FS_IRQHandler(void)
{
  g_otg_fs_irq_count++;
  tuh_int_handler(0);
}
#else
extern volatile uint32_t g_otg_fs_irq_count;  /* lives in usb_msc.c */
void OTG_FS_IRQHandler(void)
{
  g_otg_fs_irq_count++;
  tud_int_handler(0);
}
#endif
