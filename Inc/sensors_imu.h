/**
  ******************************************************************************
  * @file    sensors_imu.h
  * @brief   LSM6DSV16X accelerometer + gyroscope on SPI2 (polled).
  ******************************************************************************
  */
#ifndef PL_SENSORS_IMU_H
#define PL_SENSORS_IMU_H

#include <stdint.h>

typedef struct {
  int16_t  ax, ay, az;     /* raw counts (sign-extended) */
  int16_t  gx, gy, gz;
  uint32_t tick_ms;
  uint8_t  valid;
} PL_ImuSample;

int  IMU_Init(void);
int  IMU_Read(PL_ImuSample *out);

#endif
