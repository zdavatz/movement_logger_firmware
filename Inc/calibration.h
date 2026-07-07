/**
 ******************************************************************************
 * @file    calibration.h
 * @brief   Box-persisted board-orientation calibration (v0.0.37+).
 *
 *          The box holds a small 32-byte calibration blob on the SD root
 *          (`CAL.CFG`) so that a "Zero here" / "USB-C south" / "nose = +Y"
 *          set on ANY host (iPhone, Android, Desktop) is seen by every
 *          other host that later connects to the same box. Historically
 *          each app stored these in its own local UserDefaults / config,
 *          which meant the user had to re-calibrate on every device.
 *
 *          Wire protocol + blob layout: see DESIGN.md
 *          "Box-persisted calibration (CAL_GET / CAL_SET)".
 *
 *          Public surface is deliberately opaque: BLE_Tick reads the RAM
 *          copy verbatim into a FileData notify (CAL_GET), and merges an
 *          incoming write into it (CAL_SET). The blob layout is the ONLY
 *          spec — no per-field getters — so the layout can grow into the
 *          reserved bytes without churning ble.c.
 ******************************************************************************
 */

#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <stdint.h>

#define CAL_BLOB_SIZE       32u
#define CAL_LAYOUT_VERSION  0x01u

/* Bits in blob[1] (valid_mask). See DESIGN.md. */
#define CAL_MASK_NOSE_PLUS_Y   0x01u
#define CAL_MASK_MAG_OFFSET    0x02u
#define CAL_MASK_ANGLE_ZERO    0x04u
#define CAL_MASK_HEADING_BIAS  0x08u

/* Load CAL.CFG from SD into the RAM copy. Missing / corrupt / wrong-version
   file → RAM zeroed (valid_mask == 0, i.e. "not calibrated"). Called once
   from main() after SDFat_Mount succeeds. */
void Calibration_Init(void);

/* Copy the RAM blob out to the caller. Always CAL_BLOB_SIZE bytes. */
void Calibration_GetBlob(uint8_t out[CAL_BLOB_SIZE]);

/* Per-field merge from `in` into the RAM copy, then persist to CAL.CFG.
   Only fields whose bit is set in `in[1]` overwrite the corresponding
   bytes in RAM; unset bits leave RAM untouched. The stored valid_mask
   becomes the union of the old and new masks (no way to un-set — the
   host wipes via a SET_CAL that omits the bit AND overwrites the field
   with zeros, plus its own local wipe).

   Returns  0 on OK,
           -1 on IO error writing CAL.CFG,
           -2 on BAD_REQUEST (wrong-size caller, wrong version byte). */
int Calibration_SetFromBlob(const uint8_t *in, uint16_t len);

#endif /* CALIBRATION_H */
