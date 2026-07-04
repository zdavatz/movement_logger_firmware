/**
  ******************************************************************************
  * @file    logger.c
  * @brief   100 Hz sensors → SensNNN.csv, 10 Hz GPS → GpsNNN.csv,
  *          1 Hz fuel gauge → BatNNN.csv, periodic SD flushes.
  ******************************************************************************
  */
#include "main.h"
#include "logger.h"
#include "sched.h"
#include "errlog.h"
#include "sd_fatfs.h"
#include "sensors_imu.h"
#include "sensors_mag.h"
#include "sensors_baro.h"
#include "sensors_fuel.h"
#include "gps.h"
#include "buzzer.h"
#include <stdio.h>
#include <string.h>

static PL_File g_sens;
static PL_File g_gps;
static PL_File g_bat;
static int     g_active = 0;
static int     g_session = -1;

/* Persisted log-mode. SD config is a tiny text file on the root; the
   first byte is all that matters ('m'/'M' = manual, anything else =
   auto). g_mode is the cache: -1 = not read yet. */
#define LOGMODE_CFG_NAME "LOGMODE.CFG"
static int      g_mode = -1;

/* MANUAL-mode auto-stop deadline (HAL_GetTick() ms). 0 = no deadline
   (AUTO mode, or a MANUAL session started with duration 0). */
static uint32_t g_stop_at_ms = 0;

/* Last-read baro sample is held between baro polls (25 Hz) so each 100 Hz
   sensor row gets the most recent pressure/temperature without re-reading. */
static PL_BaroSample g_baro_cache;

/* Latest-sample cache for the BLE SensorStream emitter (see Logger_GetSnapshot).
   Refreshed in Logger_Tick as each sensor is polled. */
static PL_ImuSample  g_last_imu;
static PL_MagSample  g_last_mag;
static PL_FuelSample g_last_fuel;
static PL_GpsFix     g_last_gps;

int Logger_Init(void)
{
  if (!SDFat_IsMounted()) return -1;
  g_session = SDFat_NextSessionNumber();
  if (g_session < 0) return -2;

  char name[20];
  char hdr[140];

  snprintf(name, sizeof(name), "SENS%03d.CSV", g_session);
  if (SDFat_OpenAppend(&g_sens, name) != PL_FX_OK) return -3;
  snprintf(hdr, sizeof(hdr),
           "ms,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps,mx_mg,my_mg,mz_mg,p_hPa,t_C\n");
  SDFat_Append(&g_sens, hdr, (uint32_t)strlen(hdr));

  snprintf(name, sizeof(name), "GPS%03d.CSV", g_session);
  if (SDFat_OpenAppend(&g_gps, name) != PL_FX_OK) return -4;
  snprintf(hdr, sizeof(hdr),
           "ms,utc,lat,lon,alt_m,speed_kmh,course_deg,fix_q,nsat,hdop,cn0_max,sats_in_view\n");
  SDFat_Append(&g_gps, hdr, (uint32_t)strlen(hdr));

  snprintf(name, sizeof(name), "BAT%03d.CSV", g_session);
  if (SDFat_OpenAppend(&g_bat, name) != PL_FX_OK) return -5;
  snprintf(hdr, sizeof(hdr), "ms,v_mV,soc_x10,i_x100uA\n");
  SDFat_Append(&g_bat, hdr, (uint32_t)strlen(hdr));

  SDFat_Flush(&g_sens);
  SDFat_Flush(&g_gps);
  SDFat_Flush(&g_bat);

  g_active = 1;
  ErrLog_Writef("logger: session %03d open", g_session);
  return 0;
}

int Logger_IsActive(void) { return g_active; }

int Logger_GetMode(void)
{
  if (g_mode >= 0) return g_mode;            /* cached */

  g_mode = LOGGER_MODE_AUTO;                 /* safe default */
  if (SDFat_IsMounted()) {
    PL_File f;
    if (SDFat_OpenRead(&f, LOGMODE_CFG_NAME) == PL_FX_OK) {
      char c = 0;
      uint32_t got = 0;
      if (SDFat_Read(&f, &c, 1, &got) == PL_FX_OK && got == 1 &&
          (c == 'm' || c == 'M')) {
        g_mode = LOGGER_MODE_MANUAL;
      }
      SDFat_Close(&f);
    }
  }
  ErrLog_Writef("logger: mode = %s",
                g_mode == LOGGER_MODE_MANUAL ? "manual" : "auto");
  return g_mode;
}

int Logger_SetMode(int manual)
{
  manual = manual ? 1 : 0;

  /* Overwrite by delete + recreate — the SD layer is append-only with
     no truncate. NOT_FOUND on delete is fine (first time). */
  SDFat_Delete(LOGMODE_CFG_NAME);
  PL_File f;
  if (SDFat_OpenAppend(&f, LOGMODE_CFG_NAME) != PL_FX_OK) {
    ErrLog_Write("logger: SetMode open fail");
    return -1;
  }
  const char *txt = manual ? "manual\n" : "auto\n";
  pl_fx_status_t s = SDFat_Append(&f, txt, (uint32_t)strlen(txt));
  SDFat_Flush(&f);
  SDFat_Close(&f);
  if (s != PL_FX_OK) {
    ErrLog_Write("logger: SetMode write fail");
    return -1;
  }

  g_mode = manual ? LOGGER_MODE_MANUAL : LOGGER_MODE_AUTO;
  ErrLog_Writef("logger: mode set to %s", manual ? "manual" : "auto");

  /* Apply now. AUTO + idle → start recording immediately so the user
     doesn't have to power-cycle. MANUAL never stops a running session
     (would be silent data loss); it just takes effect on the next boot
     / next session boundary. */
  if (!manual && !g_active) {
    g_stop_at_ms = 0;
    if (Logger_Init() != 0) ErrLog_Write("logger: SetMode auto-start fail");
  }
  return 0;
}

int Logger_StartSession(uint32_t duration_s)
{
  if (g_active) {
    /* Already recording — re-arm the deadline so a repeated START_LOG
       extends rather than errors. */
    g_stop_at_ms = duration_s ? (HAL_GetTick() + duration_s * 1000U) : 0;
    ErrLog_Writef("logger: StartSession (already active) dur=%lus",
                  (unsigned long)duration_s);
    return 0;
  }
  if (Logger_Init() != 0) {
    ErrLog_Write("logger: StartSession init fail");
    return -1;
  }
  g_stop_at_ms = duration_s ? (HAL_GetTick() + duration_s * 1000U) : 0;
  ErrLog_Writef("logger: StartSession dur=%lus", (unsigned long)duration_s);
  return 0;
}

static void emit_sensor_row(const PL_ImuSample *imu,
                            const PL_MagSample *mag,
                            const PL_BaroSample *baro)
{
  /* Scale to engineering units inline (cheap; avoids floats almost-everywhere). */
  int32_t ax_mg = ((int32_t)imu->ax * 122) / 1000;  /* 0.122 mg/LSB at ±4 g */
  int32_t ay_mg = ((int32_t)imu->ay * 122) / 1000;
  int32_t az_mg = ((int32_t)imu->az * 122) / 1000;
  /* Gyro ±2000 dps → 70 mdps/LSB (v0.0.28; was ±500 / 17.5). Output is
     still mdps — same column/units, 4x coarser, negligible for pumpfoil.
     Keep in lock-step with CTRL6_G (sensors_imu.c) + Stream_Pack. */
  int32_t gx_mdps = ((int32_t)imu->gx * 700) / 10;
  int32_t gy_mdps = ((int32_t)imu->gy * 700) / 10;
  int32_t gz_mdps = ((int32_t)imu->gz * 700) / 10;
  /* Mag ±50 gauss / 1.5 mgauss/LSB */
  int32_t mx_mg = ((int32_t)mag->mx * 15) / 10;
  int32_t my_mg = ((int32_t)mag->my * 15) / 10;
  int32_t mz_mg = ((int32_t)mag->mz * 15) / 10;

  /* Pressure in hPa (one decimal) — convert pa/100 → hPa, integer + 1 decimal. */
  int32_t p_int  = baro->pressure_pa / 100;
  int32_t p_frac = (baro->pressure_pa - p_int * 100) / 10;
  if (p_frac < 0) p_frac = -p_frac;
  int32_t t_int  = baro->temperature_cC / 100;
  int32_t t_frac = (baro->temperature_cC - t_int * 100) / 10;
  if (t_frac < 0) t_frac = -t_frac;

  char row[160];
  int n = snprintf(row, sizeof(row),
                   "%lu,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld,%ld.%01ld,%ld.%01ld\n",
                   (unsigned long)imu->tick_ms,
                   (long)ax_mg, (long)ay_mg, (long)az_mg,
                   (long)gx_mdps, (long)gy_mdps, (long)gz_mdps,
                   (long)mx_mg, (long)my_mg, (long)mz_mg,
                   (long)p_int, (long)p_frac,
                   (long)t_int, (long)t_frac);
  if (n > 0) SDFat_Append(&g_sens, row, (uint32_t)n);
}

static void emit_gps_row(const PL_GpsFix *fix)
{
  /* Skip when no valid fix has come in yet. */
  if (!fix->valid) return;

  /* Decimal lat/lon to 6 places with simple integer + fractional split. */
  long lat_i = (long)fix->lat;
  long lat_f = (long)((fix->lat - lat_i) * 1e6);
  if (lat_f < 0) lat_f = -lat_f;
  long lon_i = (long)fix->lon;
  long lon_f = (long)((fix->lon - lon_i) * 1e6);
  if (lon_f < 0) lon_f = -lon_f;

  long alt_i = (long)fix->alt_m;
  long spd_i = (long)(fix->speed_kmh * 10);
  long cog_i = (long)(fix->course * 10);
  long hdop_i = (long)(fix->hdop * 10);

  char row[180];
  int n = snprintf(row, sizeof(row),
                   "%lu,%s,%ld.%06ld,%ld.%06ld,%ld,%ld.%01ld,%ld.%01ld,%u,%u,%ld.%01ld,%u,%u\n",
                   (unsigned long)fix->tick_ms, fix->utc,
                   lat_i, lat_f, lon_i, lon_f, alt_i,
                   spd_i / 10, spd_i % 10,
                   cog_i / 10, cog_i % 10,
                   (unsigned)fix->fix_q, (unsigned)fix->num_sat,
                   hdop_i / 10, hdop_i % 10,
                   (unsigned)fix->cn0_max, (unsigned)fix->sats_in_view);
  if (n > 0) SDFat_Append(&g_gps, row, (uint32_t)n);
}

static void emit_bat_row(const PL_FuelSample *fuel)
{
  char row[80];
  int n = snprintf(row, sizeof(row), "%lu,%u,%u,%d\n",
                   (unsigned long)fuel->tick_ms,
                   (unsigned)fuel->voltage_mV,
                   (unsigned)fuel->soc_x10,
                   (int)fuel->current_x100uA);
  if (n > 0) SDFat_Append(&g_bat, row, (uint32_t)n);
}

void Logger_Tick(void)
{
  /* Live-snapshot acquisition runs every tick even when no SD session
     is active, so the BLE SensorStream stays alive while idle and
     between MANUAL sessions. The old `if (!g_active) return;` here
     froze g_last_* whenever g_active==0 (MANUAL mode, after STOP_LOG,
     or any idle state introduced by the AUTO/MANUAL log-mode split),
     so every BLE client — Android and desktop alike — saw a stale /
     zero live feed. SD writes, flush and the duration deadline below
     stay gated on g_active: g_sens/g_gps/g_bat are closed when idle
     and must never be touched then. */

  /* MANUAL-mode fixed-duration session: close + go idle when the
     deadline passes. AUTO sessions never set g_stop_at_ms. Compare as
     signed delta so a HAL_GetTick() wrap (49.7 days) can't strand a
     session open forever. Only meaningful for a live session; fall
     through afterwards so this same tick still refreshes the snapshot
     (no early return — that was the bug). */
  if (g_active && g_stop_at_ms != 0 &&
      (int32_t)(HAL_GetTick() - g_stop_at_ms) >= 0) {
    g_stop_at_ms = 0;
    ErrLog_Write("logger: session duration reached — stopping");
    Logger_Stop();
  }

  if (sched_due(PL_SCHED_BARO, PL_CADENCE_BARO)) {
    BARO_Read(&g_baro_cache);
  }

  if (sched_due(PL_SCHED_SENSOR, PL_CADENCE_SENSOR)) {
    PL_ImuSample imu; PL_MagSample mag;
    if (IMU_Read(&imu) == 0 && MAG_Read(&mag) == 0 && g_baro_cache.valid) {
      g_last_imu = imu;
      g_last_mag = mag;
      if (g_active) emit_sensor_row(&imu, &mag, &g_baro_cache);
    }
  }

  if (GPS_FixUpdated()) {
    PL_GpsFix f;
    if (GPS_GetLatestFix(&f) == 0) {
      g_last_gps = f;
      /* Only write to GpsNNN.csv when the fix is actually valid. GGA
         alone bumps the update flag (parse_gga sets g_updated=1 even
         with fix_q=0), so without this filter every disconnected
         antenna or open-sky-blocked moment writes "ghost" rows with
         the last-known lat/lon and nsat=0 / hdop=99.9 — confusing the
         visualisation tools that interpret each row as a real fix. */
      if (g_active && f.fix_q > 0) emit_gps_row(&f);
    }
  }

#if PL_GPS_ANTENNA_BEEP
  /* Antenna-test mode: every 10 s beep N times where N = satellites in
     fix. Direct audio feedback while swapping antennas — more sats =
     better antenna. Build with `make GPS_ANTENNA_BEEP=1`. Blocks
     Logger_Tick for ~N*140ms during the beep burst, which is fine for
     a transient test build (we're not chasing 100 Hz IMU here, we're
     listening to the buzzer). Default off so production logging
     stays silent. */
  static uint32_t s_last_ant_beep_ms = 0;
  uint32_t now_ant = HAL_GetTick();
  if (now_ant - s_last_ant_beep_ms >= 10000) {
    s_last_ant_beep_ms = now_ant;
    uint8_t n = g_last_gps.num_sat;
    if (n > 16) n = 16;                  /* cap so worst case stays ~2 s */
    if (g_last_gps.fix_q > 0 && n > 0) {
      /* 100 ms beep + 250 ms gap = 350 ms per "count tick" → 10 sats =
         3.5 s burst. Slow enough for the ear to count. */
      for (uint8_t i = 0; i < n; i++) {
        Buzzer_Beep(2000, 100);
        HAL_Delay(250);
      }
    }
  }
#endif

  if (sched_due(PL_SCHED_BATTERY, PL_CADENCE_BATTERY)) {
    PL_FuelSample fuel;
    if (FUEL_Read(&fuel) == 0) {
      g_last_fuel = fuel;
      if (g_active) emit_bat_row(&fuel);
    }
  }

  if (g_active && sched_due(PL_SCHED_FLUSH, PL_CADENCE_FLUSH)) {
    SDFat_Flush(&g_sens);
    SDFat_Flush(&g_gps);
    SDFat_Flush(&g_bat);
    ErrLog_Flush();
  }

  /* GPS diagnostic dump every 5 s — Build #8 bring-up only. Tells us
     instantly whether DMA is alive (bytes>0), framing is OK (lines_good>0),
     and whether the module has a fix (rmc>0, gga>0). */
  static uint32_t s_last_gps_diag = 0;
  uint32_t now_diag = HAL_GetTick();
  if (now_diag - s_last_gps_diag >= 5000) {
    s_last_gps_diag = now_diag;
    extern void ErrLog_Writef(const char *fmt, ...);
    uint32_t b, lg, lb, rmc, gga, err, rxd, ubxd;
    GPS_GetStats(&b, &lg, &lb, &rmc, &gga, &err, &rxd, &ubxd);
    ErrLog_Writef("gps_diag: bytes=%lu lines_good=%lu lines_bad=%lu rmc=%lu gga=%lu errors=%lu rx_drop=%lu ubx_drop=%lu",
                  (unsigned long)b, (unsigned long)lg, (unsigned long)lb,
                  (unsigned long)rmc, (unsigned long)gga, (unsigned long)err,
                  (unsigned long)rxd, (unsigned long)ubxd);

    /* One-shot loud marker if GPS communication is fundamentally broken.
       Fires once per boot at 30 s if lines_good is still 0 — the
       deliberate hand-signal that "the GPS file will be empty, go look
       at the module / wiring / baud rate, the firmware can't see NMEA".
       No retry logic on purpose: a retry path is hard to test reliably
       and a loud error marker is enough to drive the human to fix the
       root cause (factory-reset the module, check wires, etc.). */
    static uint8_t s_gps_broken_announced = 0;
    if (!s_gps_broken_announced && now_diag >= 30000 && lg == 0) {
      s_gps_broken_announced = 1;
      ErrLog_Writef("*** GPS NO NMEA: %lus, 0 lines parsed (bytes=%lu errors=%lu) ***",
                    (unsigned long)(now_diag / 1000),
                    (unsigned long)b, (unsigned long)err);
      ErrLog_Writef("*** module is sending, we can't decode — check UART baud/wiring ***");
    }
  }
}

void Logger_FlushAll(void)
{
  if (!g_active) return;
  SDFat_Flush(&g_sens);
  SDFat_Flush(&g_gps);
  SDFat_Flush(&g_bat);
  ErrLog_Flush();
}

int Logger_WriteSyncMarker(uint64_t epoch_ms)
{
  /* No open session → nothing to anchor. Host still gets an OK reply. */
  if (!g_active) return 0;

  /* Format the 64-bit epoch by hand: newlib-nano's printf is built without
     %llu support (errlog.c sidesteps it the same way, casting to %lu after
     a divide), so we can't rely on snprintf for a u64. */
  char dec[24];
  int  p = (int)sizeof(dec);
  dec[--p] = '\0';
  if (epoch_ms == 0u) {
    dec[--p] = '0';
  } else {
    while (epoch_ms > 0u && p > 0) {
      dec[--p] = (char)('0' + (int)(epoch_ms % 10u));
      epoch_ms /= 10u;
    }
  }

  /* tick_ms is HAL_GetTick() — the SAME free-running ms counter as the CSV
     `ms` column (imu/fix tick_ms). Pairing it with the host epoch lets the
     replay tools map every row to absolute wall-clock. A leading '#' makes
     the line a comment all CSV parsers skip as a data row. Written as one
     complete \n-terminated Append on the logging thread, so it can never
     interleave mid-row. */
  char line[64];
  int n = snprintf(line, sizeof(line),
                   "# SYNC epoch_ms=%s tick_ms=%lu\n",
                   &dec[p], (unsigned long)HAL_GetTick());
  if (n <= 0) return 0;

  SDFat_Append(&g_sens, line, (uint32_t)n);
  SDFat_Append(&g_gps,  line, (uint32_t)n);
  SDFat_Flush(&g_sens);
  SDFat_Flush(&g_gps);
  ErrLog_Writef("logger: sync marker tick=%lu", (unsigned long)HAL_GetTick());
  return 1;
}

void Logger_Stop(void)
{
  if (!g_active) return;
  g_active = 0;                 /* stop Logger_Tick writing first */
  SDFat_Close(&g_sens);
  SDFat_Close(&g_gps);
  SDFat_Close(&g_bat);
  ErrLog_Write("logger: session stopped (BLE STOP_LOG)");
  ErrLog_Flush();
}

void Logger_GetSnapshot(PL_Snapshot *out)
{
  if (!out) return;
  out->imu  = g_last_imu;
  out->mag  = g_last_mag;
  out->baro = g_baro_cache;
  out->fuel = g_last_fuel;
  out->gps  = g_last_gps;
}
