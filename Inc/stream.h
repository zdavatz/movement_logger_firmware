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

#define STREAM_PACKET_SIZE  46

/* Pack the latest sensor snapshot into the 46-byte wire format.
   `logging_active` feeds flags bit 2. */
void Stream_Pack(const PL_Snapshot *snap, uint8_t logging_active,
                 uint8_t out[STREAM_PACKET_SIZE]);

#endif /* PL_STREAM_H */
