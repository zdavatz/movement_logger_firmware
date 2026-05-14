/**
  ******************************************************************************
  * @file    logger.h
  * @brief   Top-level sensor logger. Owns the Sens/Gps/Bat CSVs.
  ******************************************************************************
  */
#ifndef PL_LOGGER_H
#define PL_LOGGER_H

#include <stdint.h>
#include "sensors_imu.h"
#include "sensors_mag.h"
#include "sensors_baro.h"
#include "sensors_fuel.h"
#include "gps.h"

/* Latest-sample cache, refreshed by Logger_Tick as it polls each sensor.
   The BLE SensorStream emitter reads this — it never triggers its own
   sensor reads (DESIGN.md Section 3). */
typedef struct {
  PL_ImuSample  imu;
  PL_MagSample  mag;
  PL_BaroSample baro;
  PL_FuelSample fuel;
  PL_GpsFix     gps;
} PL_Snapshot;

int  Logger_Init(void);            /* opens session files; returns 0 on success */
void Logger_Tick(void);            /* called every 1 ms from main loop */
int  Logger_IsActive(void);
void Logger_FlushAll(void);

/* Copy the most-recent cached sensor samples. Cheap; no I/O. */
void Logger_GetSnapshot(PL_Snapshot *out);

/* Gracefully close the active session: flush + close all CSVs, mark the
   logger inactive so Logger_Tick stops writing. After this the SD files are
   complete and safe to read/delete over BLE FileSync. Idempotent. */
void Logger_Stop(void);

#endif
