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
#include <stdio.h>
#include <string.h>

static PL_File g_sens;
static PL_File g_gps;
static PL_File g_bat;
static int     g_active = 0;
static int     g_session = -1;

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
           "ms,utc,lat,lon,alt_m,speed_kmh,course_deg,fix_q,nsat,hdop\n");
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

static void emit_sensor_row(const PL_ImuSample *imu,
                            const PL_MagSample *mag,
                            const PL_BaroSample *baro)
{
  /* Scale to engineering units inline (cheap; avoids floats almost-everywhere). */
  int32_t ax_mg = ((int32_t)imu->ax * 122) / 1000;  /* 0.122 mg/LSB at ±4 g */
  int32_t ay_mg = ((int32_t)imu->ay * 122) / 1000;
  int32_t az_mg = ((int32_t)imu->az * 122) / 1000;
  /* Gyro ±500 dps → 17.5 mdps/LSB */
  int32_t gx_mdps = ((int32_t)imu->gx * 175) / 10;
  int32_t gy_mdps = ((int32_t)imu->gy * 175) / 10;
  int32_t gz_mdps = ((int32_t)imu->gz * 175) / 10;
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
                   "%lu,%s,%ld.%06ld,%ld.%06ld,%ld,%ld.%01ld,%ld.%01ld,%u,%u,%ld.%01ld\n",
                   (unsigned long)fix->tick_ms, fix->utc,
                   lat_i, lat_f, lon_i, lon_f, alt_i,
                   spd_i / 10, spd_i % 10,
                   cog_i / 10, cog_i % 10,
                   (unsigned)fix->fix_q, (unsigned)fix->num_sat,
                   hdop_i / 10, hdop_i % 10);
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
  if (!g_active) return;

  if (sched_due(PL_SCHED_BARO, PL_CADENCE_BARO)) {
    BARO_Read(&g_baro_cache);
  }

  if (sched_due(PL_SCHED_SENSOR, PL_CADENCE_SENSOR)) {
    PL_ImuSample imu; PL_MagSample mag;
    if (IMU_Read(&imu) == 0 && MAG_Read(&mag) == 0 && g_baro_cache.valid) {
      g_last_imu = imu;
      g_last_mag = mag;
      emit_sensor_row(&imu, &mag, &g_baro_cache);
    }
  }

  if (GPS_FixUpdated()) {
    PL_GpsFix f;
    if (GPS_GetLatestFix(&f) == 0) {
      g_last_gps = f;
      emit_gps_row(&f);
    }
  }

  if (sched_due(PL_SCHED_BATTERY, PL_CADENCE_BATTERY)) {
    PL_FuelSample fuel;
    if (FUEL_Read(&fuel) == 0) {
      g_last_fuel = fuel;
      emit_bat_row(&fuel);
    }
  }

  if (sched_due(PL_SCHED_FLUSH, PL_CADENCE_FLUSH)) {
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
    uint32_t b, lg, lb, rmc, gga, err;
    GPS_GetStats(&b, &lg, &lb, &rmc, &gga, &err);
    ErrLog_Writef("gps_diag: bytes=%lu lines_good=%lu lines_bad=%lu rmc=%lu gga=%lu errors=%lu",
                  (unsigned long)b, (unsigned long)lg, (unsigned long)lb,
                  (unsigned long)rmc, (unsigned long)gga, (unsigned long)err);
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
