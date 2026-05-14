/**
  ******************************************************************************
  * @file    main.h
  * @brief   Public hooks needed across modules. Kept tiny on purpose —
  *          modules expose their own headers, this one only has the
  *          cross-cutting bits (clock + error handler + the HAL include).
  ******************************************************************************
  */

#ifndef PUMPLOGGER_MAIN_H
#define PUMPLOGGER_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32u5xx_hal.h"
#include "config.h"

void SystemClock_Config(void);
void Error_Handler(const char *file, int line);

/* Snapshotted RCC->CSR captured by main() right after HAL_Init(), used by
   ErrLog_Init() to decode the reset reason. */
extern uint32_t BootResetCsr;

#ifdef __cplusplus
}
#endif

#endif /* PUMPLOGGER_MAIN_H */
