/**
  ******************************************************************************
  * @file    stream.h
  * @brief   SensorStream packer — builds the 46-byte all-sensor snapshot
  *          (DESIGN.md Section 3). Pure function, no I/O, no state: ble.c
  *          owns the timing, the CCCD-subscription tracking and the notify.
  *          There is no LOG-vs-STREAM mode — SD logging is always on; the
  *          stream is just a notify side-channel when a client subscribes.
  ******************************************************************************
  */
#ifndef PL_STREAM_H
#define PL_STREAM_H

#include <stdint.h>
#include "logger.h"
#include "gps.h"

/* v0.0.55: 46-byte legacy layout + 12-byte GPS RF extension (bytes
   46..57, see DESIGN.md Section 3). Hosts accept both 46 (legacy
   firmware) and 58; flags bit 3 marks the extension's MON-RF fields as
   fresh. */
#define STREAM_PACKET_SIZE  58

/* Pack the latest sensor snapshot into the wire format.
   `logging_active` feeds flags bit 2; `rf` (may be NULL) fills the RF
   extension bytes and flags bit 3. */
void Stream_Pack(const PL_Snapshot *snap, uint8_t logging_active,
                 const PL_GpsRfLive *rf,
                 uint8_t out[STREAM_PACKET_SIZE]);

#endif /* PL_STREAM_H */
