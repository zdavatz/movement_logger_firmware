/**
  ******************************************************************************
  * @file    stream.c
  * @brief   SensorStream 46-byte packer. See DESIGN.md Section 3 for the
  *          exact wire layout. Little-endian throughout.
  ******************************************************************************
  */
#include "stream.h"
#include <string.h>

static inline void wr16(uint8_t *p, int16_t v)
{
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
}

/* Saturate a 32-bit value into int16 range instead of letting the cast
   WRAP. Belt-and-suspenders for the stream fields; with the deci-dps gyro
   scale below it no longer trips in practice (the full ±500 dps FS fits),
   but it still guards against a stray out-of-range sample. */
static inline int16_t sat16(int32_t v)
{
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

static inline void wr32(uint8_t *p, int32_t v)
{
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

void Stream_Pack(const PL_Snapshot *s, uint8_t logging_active,
                 uint8_t out[STREAM_PACKET_SIZE])
{
  memset(out, 0, STREAM_PACKET_SIZE);
  if (!s) return;

  /* Scale raw sensor counts to engineering units, matching the CSV path:
       acc:  0.122 mg/LSB  → mg        = raw * 122 / 1000
       gyro: 17.5 mdps/LSB → centi-dps = raw * 175 / 100
       mag:  1.5 mgauss/LSB → mgauss   = raw * 15  / 10                  */
  wr32(&out[0], (int32_t)s->imu.tick_ms);

  wr16(&out[4],  (int16_t)(((int32_t)s->imu.ax * 122) / 1000));
  wr16(&out[6],  (int16_t)(((int32_t)s->imu.ay * 122) / 1000));
  wr16(&out[8],  (int16_t)(((int32_t)s->imu.az * 122) / 1000));

  /* gyro: DECI-dps (raw * 17.5 mdps/LSB, ÷10) — v0.0.27. Was centi-dps
     (÷100), which overflowed int16 above 327.67 dps and clamped/wrapped, so
     a normal hand rotation (peaks well past 327 dps) undercounted and the
     host's gyro-integrated live heading lost ~12%/turn. Deci-dps covers the
     full ±500 dps gyro FS in int16 (max ≈ 5000). Host divides by 10 for dps
     (was /100). CSV path (emit_sensor_row) is unchanged — it packs mdps and
     is unaffected. */
  wr16(&out[10], sat16(((int32_t)s->imu.gx * 175) / 1000));
  wr16(&out[12], sat16(((int32_t)s->imu.gy * 175) / 1000));
  wr16(&out[14], sat16(((int32_t)s->imu.gz * 175) / 1000));

  wr16(&out[16], (int16_t)(((int32_t)s->mag.mx * 15) / 10));
  wr16(&out[18], (int16_t)(((int32_t)s->mag.my * 15) / 10));
  wr16(&out[20], (int16_t)(((int32_t)s->mag.mz * 15) / 10));

  wr32(&out[22], s->baro.pressure_pa);
  wr16(&out[26], s->baro.temperature_cC);

  /* GPS — lat/lon are doubles in PL_GpsFix; pack as degrees × 1e7. */
  wr32(&out[28], (int32_t)(s->gps.lat * 1e7));
  wr32(&out[32], (int32_t)(s->gps.lon * 1e7));
  wr16(&out[36], (int16_t)s->gps.alt_m);
  wr16(&out[38], (int16_t)(s->gps.speed_kmh * 100.0f));   /* cm/h × 10 ≈ km/h × 100 */
  wr16(&out[40], (int16_t)(s->gps.course * 100.0f));      /* centi-degrees */
  out[42] = s->gps.fix_q;
  out[43] = s->gps.num_sat;

  uint8_t flags = 0;
  if (s->gps.valid)      flags |= 0x01;   /* bit 0: gps_valid */
  /* bit 1 (low_battery) is owned by the battery module — Phase 7 */
  if (logging_active)    flags |= 0x04;   /* bit 2: logging_active */
  out[44] = flags;
  out[45] = s->gps.cn0_max;                /* GPS strongest C/N0 (dB-Hz), 0 = no data */
}
