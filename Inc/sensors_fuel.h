/**
  ******************************************************************************
  * @file    sensors_fuel.h
  * @brief   STC3115 fuel gauge on I²C4 @ 1 Hz.
  ******************************************************************************
  */
#ifndef PL_SENSORS_FUEL_H
#define PL_SENSORS_FUEL_H
#include <stdint.h>

typedef struct {
  uint16_t voltage_mV;
  int16_t  current_x100uA;
  uint16_t soc_x10;          /* 0..1000 = 0..100.0 % */
  uint32_t tick_ms;
  uint8_t  valid;
} PL_FuelSample;

int FUEL_Init(void);
int FUEL_Read(PL_FuelSample *out);

#endif
