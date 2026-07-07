/**
 ******************************************************************************
 * @file    calibration.c
 * @brief   Box-persisted board-orientation calibration (v0.0.37+).
 *
 *          Backing store: 32-byte `CAL.CFG` on the SD root.
 *          Wire format + semantics: DESIGN.md
 *          "Box-persisted calibration (CAL_GET / CAL_SET)".
 ******************************************************************************
 */

#include "calibration.h"
#include "sd_fatfs.h"
#include "errlog.h"

#include <string.h>

#define CAL_CFG_NAME "CAL.CFG"

/* RAM shadow of CAL.CFG. Populated by Calibration_Init at boot; mutated
   by Calibration_SetFromBlob per-field per SET_CAL merge. Zero-init means
   "not calibrated" (valid_mask == 0), the same state as a fresh SD card. */
static uint8_t g_cal[CAL_BLOB_SIZE];

/* Non-zero iff Calibration_Init has run. Guards a stray Calibration_GetBlob
   before SD mount (would return zeros anyway — the guard is there to log
   the ordering bug instead of silently pretending "not calibrated"). */
static uint8_t g_ready;

/* Overwrite CAL.CFG with the current RAM copy. The SD layer is
   append-only with no truncate; delete + recreate — the same pattern
   `Logger_SetMode` uses for LOGMODE.CFG. NOT_FOUND on delete is fine
   (first time). Returns 0 on OK, -1 on IO error. */
static int cal_persist(void)
{
  if (!SDFat_IsMounted()) return -1;
  SDFat_Delete(CAL_CFG_NAME);
  PL_File f;
  if (SDFat_OpenAppend(&f, CAL_CFG_NAME) != PL_FX_OK) {
    ErrLog_Write("*** cal: persist open fail");
    return -1;
  }
  pl_fx_status_t s = SDFat_Append(&f, g_cal, CAL_BLOB_SIZE);
  SDFat_Flush(&f);
  SDFat_Close(&f);
  if (s != PL_FX_OK) {
    ErrLog_Write("*** cal: persist write fail");
    return -1;
  }
  return 0;
}

void Calibration_Init(void)
{
  memset(g_cal, 0, sizeof(g_cal));
  g_ready = 1;

  if (!SDFat_IsMounted()) {
    ErrLog_Write("cal: SD not mounted, using zeroed blob (valid_mask=0)");
    return;
  }
  PL_File f;
  if (SDFat_OpenRead(&f, CAL_CFG_NAME) != PL_FX_OK) {
    ErrLog_Write("cal: CAL.CFG missing → valid_mask=0 (first boot / never calibrated)");
    return;
  }
  uint8_t buf[CAL_BLOB_SIZE];
  uint32_t got = 0;
  pl_fx_status_t s = SDFat_Read(&f, buf, CAL_BLOB_SIZE, &got);
  SDFat_Close(&f);
  if (s != PL_FX_OK || got != CAL_BLOB_SIZE) {
    ErrLog_Writef("*** cal: CAL.CFG short read (got=%lu s=%d) → treating as unset",
                  (unsigned long)got, (int)s);
    return;
  }
  if (buf[0] != CAL_LAYOUT_VERSION) {
    ErrLog_Writef("*** cal: CAL.CFG unknown version 0x%02X (expected 0x%02X) → treating as unset",
                  buf[0], CAL_LAYOUT_VERSION);
    return;
  }
  memcpy(g_cal, buf, CAL_BLOB_SIZE);
  ErrLog_Writef("cal: loaded (mask=0x%02X)", g_cal[1]);
}

void Calibration_GetBlob(uint8_t out[CAL_BLOB_SIZE])
{
  if (!g_ready) {
    ErrLog_Write("*** cal: GetBlob before Init — returning zeros");
    memset(out, 0, CAL_BLOB_SIZE);
    return;
  }
  /* Always report the current layout version to legacy hosts even when
     the blob is still all-zeros (they read valid_mask=0 → "no cal"). */
  g_cal[0] = CAL_LAYOUT_VERSION;
  memcpy(out, g_cal, CAL_BLOB_SIZE);
}

int Calibration_SetFromBlob(const uint8_t *in, uint16_t len)
{
  if (!g_ready) return -2;
  if (len != CAL_BLOB_SIZE) {
    ErrLog_Writef("*** cal: SET bad length %u (expected %u)",
                  (unsigned)len, (unsigned)CAL_BLOB_SIZE);
    return -2;
  }
  if (in[0] != CAL_LAYOUT_VERSION) {
    ErrLog_Writef("*** cal: SET unknown version 0x%02X (expected 0x%02X)",
                  in[0], CAL_LAYOUT_VERSION);
    return -2;
  }

  /* Per-field merge: only fields whose valid_mask bit is set in the
     incoming blob overwrite the corresponding RAM bytes. The stored
     valid_mask becomes (old | new) so a partial CAL_SET (e.g. only
     nosePlusY) doesn't wipe fields the caller didn't touch. */
  uint8_t mask = in[1];

  if (mask & CAL_MASK_NOSE_PLUS_Y) {
    g_cal[2] = in[2];               /* nosePlusY */
    /* byte 3 stays reserved/zero — do NOT copy padding */
  }
  if (mask & CAL_MASK_MAG_OFFSET) {
    memcpy(&g_cal[4], &in[4], 6);   /* magOffsetMg[3] */
  }
  if (mask & CAL_MASK_ANGLE_ZERO) {
    memcpy(&g_cal[10], &in[10], 6); /* angleZeroRef[3] */
    memcpy(&g_cal[16], &in[16], 8); /* angleZeroAtEpoch */
  }
  if (mask & CAL_MASK_HEADING_BIAS) {
    memcpy(&g_cal[24], &in[24], 2); /* headingBiasDeg */
  }

  g_cal[0]  = CAL_LAYOUT_VERSION;
  g_cal[1] |= mask;

  ErrLog_Writef("cal: SET mask=0x%02X → stored mask=0x%02X",
                mask, g_cal[1]);

  if (cal_persist() != 0) {
    /* RAM copy is updated; SD write failed. Report IO_ERROR so the host
       knows the setting won't survive a reboot; the current session
       still uses the in-RAM values so the immediate render doesn't
       regress. */
    return -1;
  }
  return 0;
}
