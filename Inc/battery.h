/**
  ******************************************************************************
  * @file    battery.h
  * @brief   BatteryStatus packer — builds the 8-byte STC3115 snapshot
  *          (DESIGN.md Section 4). Pure function, no I/O, no state: ble.c
  *          owns the 1/min notify timer, the low-battery transition check,
  *          the CCCD-subscription tracking and the stored-value update.
  ******************************************************************************
  */
#ifndef PL_BATTERY_H
#define PL_BATTERY_H

#include <stdint.h>
#include "sensors_fuel.h"

#define BATTERY_PACKET_SIZE  8
#define BATTERY_LOW_SOC_X10  100   /* SOC < 10.0 % → low-battery warning */

/* Pack the latest fuel-gauge sample into the 8-byte wire format.
   `low_batt` feeds flags bit 0, `logging_active` feeds flags bit 1. */
void Battery_Pack(const PL_FuelSample *fuel, uint8_t low_batt,
                  uint8_t logging_active, uint8_t out[BATTERY_PACKET_SIZE]);

#endif /* PL_BATTERY_H */
