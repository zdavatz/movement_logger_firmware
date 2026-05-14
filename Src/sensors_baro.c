/**
  ******************************************************************************
  * @file    sensors_baro.c
  * @brief   LPS22DF on I²C1. ODR 25 Hz, ±1260 hPa.
  ******************************************************************************
  */
#include "main.h"
#include "sensors_baro.h"
#include "sensors_mag.h"   /* for I2C1_ReadReg / WriteReg */
#include <stdio.h>

extern void ErrLog_Write(const char *msg);

#define LPS_WHO_AM_I        0x0F
#define LPS_CTRL_REG1       0x10
#define LPS_CTRL_REG2       0x11
#define LPS_CTRL_REG3       0x12
#define LPS_PRESS_OUT_XL    0x28
#define LPS_TEMP_OUT_L      0x2B
#define LPS_WHO_AM_I_VAL    0xB4

static int g_ok = 0;

int BARO_Init(void)
{
  if (I2C1_Init() != 0) return -1;
  uint8_t who = 0xFF;
  int rc = I2C1_ReadReg(PL_LPS22DF_I2C_ADDR, LPS_WHO_AM_I, &who, 1);
  {
    char m[64];
    snprintf(m, sizeof m, "baro: who@0x%02x rc=%d val=0x%02x",
             (unsigned)PL_LPS22DF_I2C_ADDR, rc, who);
    ErrLog_Write(m);
  }
  if (rc != 0) return -1;
  if (who != LPS_WHO_AM_I_VAL) return -2;

  /* CTRL_REG1: ODR[6:3] = 0011 (25 Hz), AVG[2:0] = 011 (16 avg) */
  uint8_t v = (3 << 3) | 3;
  I2C1_WriteReg(PL_LPS22DF_I2C_ADDR, LPS_CTRL_REG1, &v, 1);
  /* CTRL_REG2: BDU=1, LFPF_CFG=0, EN_LPFP=0, BOOT=0, SWRESET=0 */
  v = 0x08;
  I2C1_WriteReg(PL_LPS22DF_I2C_ADDR, LPS_CTRL_REG2, &v, 1);
  /* CTRL_REG3: IF_ADD_INC=1 (auto-increment for multi-byte reads) */
  v = 0x01;
  I2C1_WriteReg(PL_LPS22DF_I2C_ADDR, LPS_CTRL_REG3, &v, 1);

  HAL_Delay(40);   /* let the first conversion settle */
  g_ok = 1;
  return 0;
}

int BARO_Read(PL_BaroSample *out)
{
  if (!g_ok || !out) return -1;
  uint8_t b[5];
  if (I2C1_ReadReg(PL_LPS22DF_I2C_ADDR, LPS_PRESS_OUT_XL, b, 5) != 0) return -1;
  /* Pressure is a 24-bit unsigned (LSB at PRESS_OUT_XL), 4096 LSB / hPa
     → 40960 LSB / kPa → 0.244 Pa per LSB. Convert raw to Pa via raw/4096*100. */
  uint32_t raw_p = ((uint32_t)b[2] << 16) | ((uint32_t)b[1] << 8) | b[0];
  /* sign-extend 24->32 */
  if (raw_p & 0x00800000u) raw_p |= 0xFF000000u;
  int32_t p_pa = (int32_t)((int64_t)(int32_t)raw_p * 100 / 4096);
  int16_t t_cC = (int16_t)((b[4] << 8) | b[3]);   /* already 0.01 °C/LSB */

  out->pressure_pa    = p_pa;
  out->temperature_cC = t_cC;
  out->tick_ms        = HAL_GetTick();
  out->valid          = 1;
  return 0;
}
