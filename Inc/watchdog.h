/**
  ******************************************************************************
  * @file    watchdog.h
  * @brief   IWDG hardware watchdog — nothing else.
  *
  *          Deliberately minimal. An earlier "sensor-plausibility" software
  *          watchdog (reset the box if every sensor froze for N seconds) was
  *          removed: it caught only a hypothetical failure mode never seen in
  *          bring-up, while the IWDG already catches every observed hang, and
  *          a safety mechanism that can itself misfire is worse than none.
  ******************************************************************************
  */
#ifndef PL_WATCHDOG_H
#define PL_WATCHDOG_H

/* One-shot init: starts the IWDG. Call once after clocks are up. */
void Watchdog_Init(void);

/* Kick the IWDG. Call every main-loop iteration. */
void Watchdog_Tick(void);

/* Kick the IWDG from inside a long-running blocking operation (e.g. a BLE
   file transfer) that would otherwise overrun the 8 s window without
   returning to the main loop. Same effect as Watchdog_Tick(). */
void Watchdog_Kick(void);

#endif
