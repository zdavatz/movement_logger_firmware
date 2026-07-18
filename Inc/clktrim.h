/**
  ******************************************************************************
  * @file    clktrim.h
  * @brief   SysTick trim against the 32.768 kHz LSE crystal (v0.0.54).
  *          The system clock is HSI16 (RC, ±1 %); measured drift on real
  *          hardware is ~+4400 ppm — the logged tick gains ~0.26 s/min vs
  *          wall clock. When the board's LSE crystal starts, a free-running
  *          LPTIM1 clocked from it becomes the wall-time reference and the
  *          SysTick reload is slewed until 1 tick == 1 real millisecond.
  *          No LSE (crystal absent / dead): everything stays as before,
  *          one plain errlog line records the fact.
  ******************************************************************************
  */

#ifndef PUMPLOGGER_CLKTRIM_H
#define PUMPLOGGER_CLKTRIM_H

#include <stdint.h>

/* Start the LSE oscillator (non-blocking — crystal needs ~200-400 ms).
   Call once right after SystemClock_Config(); must NOT log (errlog is not
   up yet at that point in boot). */
void ClkTrim_Init(void);

/* Superloop task @ PL_CADENCE_CLKTRIM. Drives the LSE-ready state machine,
   samples LPTIM1 vs HAL_GetTick, and once per CLK_TRIM_WINDOW_MS applies a
   SysTick reload correction. All errlog output happens here (plain lines,
   never `***`). */
void ClkTrim_Tick(void);

/* Cumulative correction currently applied, in ppm (0 until first trim;
   positive = SysTick was running fast and has been slowed). Diagnostics. */
int32_t ClkTrim_AppliedPpm(void);

#endif /* PUMPLOGGER_CLKTRIM_H */
