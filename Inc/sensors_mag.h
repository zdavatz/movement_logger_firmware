/**
  ******************************************************************************
  * @file    sensors_mag.h
  * @brief   LIS2MDL magnetometer on I²C2.
  ******************************************************************************
  */
#ifndef PL_SENSORS_MAG_H
#define PL_SENSORS_MAG_H
#include <stdint.h>

typedef struct {
  int16_t  mx, my, mz;
  uint32_t tick_ms;
  uint8_t  valid;
} PL_MagSample;

/* Shared I²C2 init (idempotent). */
int  I2C1_Init(void);
int  MAG_Init(void);
int  MAG_Read(PL_MagSample *out);

/* Public helpers other I²C2 sensors reuse — keeps everyone on one handle. */
int  I2C1_ReadReg(uint16_t dev8, uint8_t reg, uint8_t *buf, uint16_t len);
int  I2C1_WriteReg(uint16_t dev8, uint8_t reg, const uint8_t *buf, uint16_t len);

#endif
