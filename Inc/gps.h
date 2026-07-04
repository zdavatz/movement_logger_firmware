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
  uint8_t  cn0_max;          /* strongest satellite C/N0 (dB-Hz) from GSV; 0 = no data */
  uint8_t  sats_in_view;     /* satellites reporting a C/N0 in the last GSV burst */
} PL_GpsFix;

int  GPS_Init(void);
void GPS_Tick(void);
int  GPS_GetLatestFix(PL_GpsFix *out);
/* True if a fresh fix was published since last Get-call. Auto-cleared. */
uint8_t GPS_FixUpdated(void);
/* Diagnostic counters for Build #8 GPS bring-up. Pass NULL for fields you
   don't need. Values are running totals since boot. */
void GPS_GetStats(uint32_t *bytes, uint32_t *lines_good, uint32_t *lines_bad,
                  uint32_t *rmc, uint32_t *gga, uint32_t *errors,
                  uint32_t *rx_dropped, uint32_t *ubx_dropped);

/* GPS fix-quality summary, used to drive the green-LED pattern (single /
   double / triple flash). Returns NONE if the latest valid-status RMC
   is older than ~3 s — i.e. signal lost mid-session goes back to
   "no fix" pattern instead of latching the previous good state. */
typedef enum {
  PL_GPS_QUALITY_NONE = 0,
  PL_GPS_QUALITY_WEAK = 1,        /* 4–6 sats */
  PL_GPS_QUALITY_GOOD = 2,        /* ≥7 sats */
} pl_gps_quality_t;
pl_gps_quality_t GPS_LastFixQuality(void);

/* ---------- BLE GPS bridge (GPS Debug tab over BLE) ----------------------
   Lets the desktop run its u-blox UBX survey (antenna/mounting diagnostics)
   over the box's BLE link instead of a serial cable — no opening the box.

   The host polls the receiver directly: UBX poll frames arrive over BLE and
   are written straight to the module UART by GPS_BridgeTx; complete UBX reply
   frames are captured out of the RX stream into a relay buffer that the BLE
   layer drains with GPS_BridgeRead and notifies back. NMEA keeps flowing to
   the SD logger the whole time — enabling the bridge only *adds* UBX output
   on the port (CFG via $PUBX,41), it never disables NMEA.

   The bridge is volatile (never persisted): a power-cycle leaves the module
   back at NMEA-only, and the BLE layer also disables it on disconnect. */
void     GPS_BridgeSet(uint8_t on);                        /* enable/disable */
uint8_t  GPS_BridgeActive(void);
void     GPS_BridgeTx(const uint8_t *data, uint16_t len);  /* host → module UART */
uint16_t GPS_BridgeRead(uint8_t *out, uint16_t max);       /* captured UBX → out */

#endif
