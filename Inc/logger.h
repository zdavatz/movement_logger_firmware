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

/* Log-mode: AUTO = the box opens a session automatically on cold boot
   (the original always-on behaviour, default). MANUAL = the box stays
   idle on boot and only logs after a BLE START_LOG, for a fixed
   duration. Persisted on the SD card (LOGMODE.CFG); changed from the
   host app via the SET_MODE opcode. */
#define LOGGER_MODE_AUTO    0
#define LOGGER_MODE_MANUAL  1

int  Logger_Init(void);            /* opens session files; returns 0 on success */
void Logger_Tick(void);            /* called every 1 ms from main loop */
int  Logger_IsActive(void);
void Logger_FlushAll(void);

/* Read the persisted log-mode from the SD config (cached after first
   call). Returns LOGGER_MODE_AUTO / LOGGER_MODE_MANUAL. Missing or
   unreadable config → AUTO (safe default = always records). */
int  Logger_GetMode(void);

/* Persist a new log-mode to the SD config and apply it: switching to
   AUTO while idle starts a session immediately; switching to MANUAL
   never kills an already-running session (no silent data loss).
   Returns 0 on success, <0 on SD error. `manual` is 0 / 1. */
int  Logger_SetMode(int manual);

/* MANUAL-mode session start: opens the session files if idle and arms
   an auto-stop `duration_s` seconds out (0 = run until STOP_LOG / power
   loss). OK-no-op if a session is already active. Returns 0 / <0. */
int  Logger_StartSession(uint32_t duration_s);

/* Copy the most-recent cached sensor samples. Cheap; no I/O. */
void Logger_GetSnapshot(PL_Snapshot *out);

/* Stamp a host-clock sync anchor into the open Sens/Gps CSVs. The host
   pushes its wall-clock `epoch_ms` over BLE (SET_TIME, on every connect);
   we pair it with the box's free-running HAL_GetTick() ms — the same
   counter as the `ms` column — and append a `# SYNC epoch_ms=.. tick_ms=..`
   comment line to both files, then flush. Lets the replay tools resolve
   absolute wall-clock without an RTC or a GPS fix. Returns 1 if the marker
   was written, 0 if no session is open (best-effort no-op). Safe to call
   from the BLE command handler — single-threaded superloop, same thread as
   Logger_Tick, so no lock is needed. */
int  Logger_WriteSyncMarker(uint64_t epoch_ms);

/* Gracefully close the active session: flush + close all CSVs, mark the
   logger inactive so Logger_Tick stops writing. After this the SD files are
   complete and safe to read/delete over BLE FileSync. Idempotent. */
void Logger_Stop(void);

#endif
