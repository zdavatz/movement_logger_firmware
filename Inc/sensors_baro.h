/**
  ******************************************************************************
  * @file    sensors_baro.h
  * @brief   LPS22DF barometer/temperature on I²C2 @ 25 Hz.
  ******************************************************************************
  */
#ifndef PL_SENSORS_BARO_H
#define PL_SENSORS_BARO_H
#include <stdint.h>

typedef struct {
  int32_t  pressure_pa;     /* Pa */
  int16_t  temperature_cC;  /* 0.01 °C */
  uint32_t tick_ms;
  uint8_t  valid;
} PL_BaroSample;

int BARO_Init(void);
int BARO_Read(PL_BaroSample *out);

#endif
