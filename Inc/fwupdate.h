/**
  ******************************************************************************
  * @file    fwupdate.h
  * @brief   Brick-safe dual-bank (A/B) firmware-over-BLE receiver.
  *
  *          The STM32U585 has 2 MB flash as two 1 MB banks with a SWAP_BANK
  *          option byte: whichever bank the boot-ROM maps to 0x08000000 is the
  *          "active" one, the other is always readable/writable at 0x08100000.
  *          The running app (≤ 1 MB, linker confined to bank 1) receives a new
  *          image into the INACTIVE bank while it keeps executing/logging from
  *          the active one (read-while-write). Only after the whole image is
  *          programmed AND its SHA-256 matches the host-declared digest does the
  *          module toggle SWAP_BANK + reset, so a corrupt / interrupted upload
  *          can never brick the box — the previous image stays intact in the
  *          other bank as an automatic rollback.
  *
  *          Wire protocol (FileCmd opcodes, host → box; replies on FileData):
  *            0x09 FW_BEGIN  <image_len:u32-LE><sha256:32>  → 1-byte status
  *            0x0A FW_DATA   <offset:u32-LE><bytes…>        → 4-byte next-off ACK
  *                                                            or 1-byte error
  *            0x0B FW_COMMIT  (none)        → 1-byte 0xA0 then reset, or error
  *            0x0C FW_ABORT   (none)        → 1-byte 0x00
  *
  *          Runtime-hang rollback (belt-and-suspenders beyond verify-before-
  *          activate): COMMIT drops a pending marker on the SD card; the freshly
  *          swapped image bumps a boot-attempt counter in FwUpdate_BootCheck()
  *          and, once healthy, clears it via FwUpdate_ConfirmBoot(). An image
  *          that verifies but hangs never confirms, so after FW_MAX_BOOT_ATTEMPTS
  *          resets the boot-check flips SWAP_BANK back to the known-good image.
  *
  *          See DESIGN.md §"Firmware update over BLE (FOTA)" and REQUIREMENTS.md
  *          F-FWU / OOS-4.
  ******************************************************************************
  */
#ifndef PL_FWUPDATE_H
#define PL_FWUPDATE_H

#include <stdint.h>

typedef enum {
  FWUP_OK = 0,
  FWUP_ERR_BUSY,        /* a session is already active / wrong state          */
  FWUP_ERR_TOO_BIG,     /* image_len exceeds a bank                           */
  FWUP_ERR_BAD_SEQ,     /* FW_DATA offset is past the expected write cursor   */
  FWUP_ERR_FLASH,       /* erase or program failed                            */
  FWUP_ERR_NOT_READY,   /* COMMIT with no active session                      */
  FWUP_ERR_HASH         /* image SHA-256 did not match the declared digest    */
} fwup_status_t;

/* Largest image we accept: leave the top 8 KB page of the bank as headroom
   (never written by an image confined to bank 1 by the linker). */
#define FW_MAX_IMAGE_BYTES   (1024u * 1024u - 8u * 1024u)

/* How many un-confirmed boots of a freshly swapped image we tolerate before
   reverting to the previous bank. Generous so a few quick power-cycles of a
   genuinely-healthy image (before it confirms) don't trigger a false revert. */
#define FW_MAX_BOOT_ATTEMPTS 5

/* Uptime (ms) after which a running image is considered healthy and clears the
   pending marker. Long enough to prove the superloop runs stably. */
#define FW_CONFIRM_UPTIME_MS 15000u

/* Begin a session: validate image_len, remember the expected digest, and erase
   exactly the inactive-bank pages the image will occupy. Blocks ~tens of ms per
   8 KB page (IWDG is fed per page); a brief sensor-logging gap at update time is
   accepted. Returns FWUP_OK (ready to receive) or an error. */
fwup_status_t FwUpdate_Begin(uint32_t image_len, const uint8_t sha256[32]);

/* Program a contiguous segment at byte `offset` (must equal the current write
   cursor; a strictly-smaller offset is treated as an idempotent retransmit and
   ACKed without reprogramming). Buffers a partial 16-byte quadword internally.
   On return *next_offset holds the new write cursor for the host's ACK. */
fwup_status_t FwUpdate_Data(uint32_t offset, const uint8_t *data, uint32_t len,
                            uint32_t *next_offset);

/* Flush the final partial quadword (0xFF-padded) and verify SHA-256 over the
   declared image length. Returns FWUP_OK when the staged image is complete and
   its digest matches (the caller should then notify the host and call
   FwUpdate_Activate), or an error. The active bank is never touched here; on
   any error path the previous image stays intact. */
fwup_status_t FwUpdate_Commit(void);

/* Activate a just-verified image: drop the SD pending marker, toggle SWAP_BANK,
   and reset into the new bank. Only valid right after FwUpdate_Commit() returned
   FWUP_OK (no-op otherwise). DOES NOT RETURN on success. Split from Commit so
   the caller can send the host a "verified, activating" ack before the reset. */
void FwUpdate_Activate(void);

/* Discard the current session. The partially-written inactive bank is left
   stale (never activated); the next FW_BEGIN re-erases it. Safe anytime. */
void FwUpdate_Abort(void);

/* True while a session is open (BEGIN done, COMMIT/ABORT not yet). */
int FwUpdate_Active(void);

/* ---- Boot-time rollback (called from main.c, not from BLE) -------------- */

/* Call once right after SDFat_Mount() succeeds. If a freshly-swapped image is
   on trial, bump its attempt counter; if it has failed to confirm too many
   times, revert SWAP_BANK and reset back to the previous image. No-op when no
   update is pending. */
void FwUpdate_BootCheck(void);

/* Call once from the main loop after the box has run stably (uptime past
   FW_CONFIRM_UPTIME_MS). Clears the SD pending marker so no revert happens.
   Idempotent / cheap no-op after the first successful call. */
void FwUpdate_ConfirmBoot(void);

#endif /* PL_FWUPDATE_H */
