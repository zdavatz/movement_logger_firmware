/**
  ******************************************************************************
  * @file    sched.h
  * @brief   Cooperative tick scheduler. SysTick fires at PL_TICK_HZ.
  *          Modules gate their work on `sched_due(slot, cadence)` which fires
  *          after at least `cadence` ticks since the previous fire — robust
  *          against missed tick boundaries when handler work overruns 1 ms.
  ******************************************************************************
  */

#ifndef PUMPLOGGER_SCHED_H
#define PUMPLOGGER_SCHED_H

#include <stdint.h>
#include <stdbool.h>

uint32_t sched_tick(void);
void     sched_wait_next_tick(void);

/* Per-slot edge-triggered scheduling. Each call site gets its own slot so
   "due since last fire" is tracked independently. Earlier sched_should_run()
   used `tick % cadence == 0` which silently dropped fires whenever a Logger_Tick
   iteration overran a tick boundary (e.g. during an SD sector flush). */
typedef enum {
  PL_SCHED_SENSOR = 0,    /* IMU + MAG @ PL_CADENCE_SENSOR (100 Hz) */
  PL_SCHED_BARO,          /* LPS22DF  @ PL_CADENCE_BARO    (25 Hz)  */
  PL_SCHED_BATTERY,       /* STC3115  @ PL_CADENCE_BATTERY (1 Hz)   */
  PL_SCHED_FLUSH,         /* SD flush @ PL_CADENCE_FLUSH   (1 Hz)   */
  PL_SCHED_LED,           /* Green LED toggle              (2 Hz)   */
  PL_SCHED_PLAUSIBLE,     /* Sensor-plausibility watchdog          */
  PL_SCHED_SLOT_COUNT
} pl_sched_slot_t;

bool sched_due(pl_sched_slot_t slot, uint32_t cadence_ticks);

#endif /* PUMPLOGGER_SCHED_H */
