/**
  ******************************************************************************
  * @file    gps.h
  * @brief   u-blox MAX-M10S on UART4 — DMA-circular RX, polling parse.
  ******************************************************************************
  */
#ifndef PL_GPS_H
#define PL_GPS_H

#include <stdint.h>

typedef struct {
  double   lat;
  double   lon;
  float    alt_m;
  float    speed_kmh;
  float    course;
  char     utc[16];          /* "hhmmss.ss" */
  uint8_t  fix_q;            /* 0=none, 1=GPS, 2=DGPS */
  uint8_t  num_sat;
  float    hdop;
  uint32_t tick_ms;
  uint8_t  valid;
} PL_GpsFix;

int  GPS_Init(void);
void GPS_Tick(void);
int  GPS_GetLatestFix(PL_GpsFix *out);
/* True if a fresh fix was published since last Get-call. Auto-cleared. */
uint8_t GPS_FixUpdated(void);
/* Diagnostic counters for Build #8 GPS bring-up. Pass NULL for fields you
   don't need. Values are running totals since boot. */
void GPS_GetStats(uint32_t *bytes, uint32_t *lines_good, uint32_t *lines_bad,
                  uint32_t *rmc, uint32_t *gga, uint32_t *errors);

#endif
