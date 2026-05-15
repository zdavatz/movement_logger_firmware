/**
  ******************************************************************************
  * @file    battery.c
  * @brief   BatteryStatus 8-byte packer. See DESIGN.md Section 4 for the
  *          exact wire layout. Little-endian throughout.
  ******************************************************************************
  */
#include "battery.h"
#include <string.h>

void Battery_Pack(const PL_FuelSample *f, uint8_t low_batt,
                  uint8_t logging_active, uint8_t out[BATTERY_PACKET_SIZE])
{
  memset(out, 0, BATTERY_PACKET_SIZE);
  if (!f) return;

  /* 0..1: voltage_mV   2..3: soc_x10   4..5: current_x100uA (signed) */
  out[0] = (uint8_t)(f->voltage_mV & 0xFF);
  out[1] = (uint8_t)((f->voltage_mV >> 8) & 0xFF);
  out[2] = (uint8_t)(f->soc_x10 & 0xFF);
  out[3] = (uint8_t)((f->soc_x10 >> 8) & 0xFF);
  out[4] = (uint8_t)(f->current_x100uA & 0xFF);
  out[5] = (uint8_t)((f->current_x100uA >> 8) & 0xFF);

  uint8_t flags = 0;
  if (low_batt)       flags |= 0x01;   /* bit 0: low_battery_warning_active */
  if (logging_active) flags |= 0x02;   /* bit 1: logging_active */
  out[6] = flags;
  out[7] = 0x00;                        /* reserved */
}
