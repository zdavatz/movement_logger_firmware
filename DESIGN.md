# Design — SensorTile.box PRO firmware rewrite

**Status:** **LOCKED v0.2** · 2026-05-11
**Reference:** [REQUIREMENTS.md](./REQUIREMENTS.md) v0.3 (locked)

This document fills in the concrete numbers, formats, and module
boundaries the requirements doc left open. Once reviewed it locks
together with the requirements; from there we go to Phase 2 (skeleton).

## Contents

1. [GATT service layout (BLE)](#1-gatt-service-layout-ble)
2. [FileSync wire protocol](#2-filesync-wire-protocol)
3. [Live-stream sample format](#3-live-stream-sample-format)
4. [Battery status format](#4-battery-status-format)
5. [HCI command sequence (host → BlueNRG-LP)](#5-hci-command-sequence-host--bluenrg-lp)
6. [SPI transport](#6-spi-transport)
7. [On-SD file formats](#7-on-sd-file-formats)
8. [Cooperative scheduler](#8-cooperative-scheduler)
9. [Module boundaries](#9-module-boundaries)
10. [Memory budget](#10-memory-budget)
11. [State-machine robustness — design standard](#11-state-machine-robustness--design-standard)
12. [Watchdog details](#12-watchdog-details)
13. [Boot sequence](#13-boot-sequence)
14. [Decisions left for implementation phase](#14-decisions-left-for-implementation-phase)

---

## 1. GATT service layout (BLE)

One custom **primary service**, four **characteristics**. We keep the
two existing FileCmd / FileData UUIDs so the existing
MovementLogger GUI works without changes; SensorStream and BatteryStatus
are new in the same BlueST-style namespace.

**Service UUID (primary):**

    00000000-0001-11e1-9ab4-0002a5d5c51b   (already used in current FW)

**Characteristics:**

| Name | UUID | Properties | Max size | Description |
|---|---|---|---|---|
| FileCmd | `00000080-0010-11e1-ac36-0002a5d5c51b` | write w/o response | 20 B | host → box. Opcode + args. |
| FileData | `00000040-0010-11e1-ac36-0002a5d5c51b` | notify | 20 B | box → host. Notification chunks: file listing rows, file body chunks, single-byte status replies. |
| SensorStream | `00000100-0010-11e1-ac36-0002a5d5c51b` | notify | 46 B (single notify w/ MTU upgrade) or 3 × ~20 B chunks (default MTU fallback) | box → host. One packed all-sensor snapshot every 2 s (Section 3). Emitted automatically while a client is subscribed to this CCCD — **concurrent with SD logging, never instead of it**. |
| BatteryStatus | `00000200-0010-11e1-ac36-0002a5d5c51b` | read + notify | 8 B | box → host. SOC / mV / current. Notify once per minute when connected (Section 4). |

Each notify-capable characteristic carries an implicit CCCD descriptor
(handle = char value handle + 1). The host must write `0x01 0x00` to
the CCCD to subscribe before any notifications fire.

20 B chunks match the BLE 4.0 default ATT MTU minus 3 bytes of header.
That sizing also matches what the BlueNRG-LP accepts as a single
notify payload without negotiating a higher MTU.

---

## 2. FileSync wire protocol

### Host → box (FileCmd writes)

Each write starts with one opcode byte, followed by an opcode-specific
payload.

| Opcode | Name | Payload | Notes |
|---|---|---|---|
| `0x01` | `LIST` | none | Request a listing of the SD root. |
| `0x02` | `READ` | `<name>\0[<offset:u32-LE>]` | Read `<name>` from byte `<offset>` to EOF. `<name>` is a NUL-terminated ASCII 8.3 filename. The 4-byte little-endian offset is optional — absent (write ends at the NUL) means `offset = 0` = whole file. A resumed transfer after a dropped link sends `offset = bytes already received`. Works on the *active* session's files too — returns a consistent snapshot up to the last 1 Hz flush. |
| `0x03` | `DELETE` | `<name>\0` | Delete the file. `<name>` NUL-terminated. Returns single-byte status. |
| `0x04` | `STOP_LOG` | none | Flush + close the current session. No FileData reply (host re-checks via LIST). |
| `0x05` | `START_LOG` | `[<dur:u32-LE>]` | MANUAL-mode session start. Opens the next `SensNNN.csv` set; if `dur` (seconds) is given and non-zero, the box auto-closes the session after it and goes idle again. `dur` absent / 0 = run until `STOP_LOG` or power loss. OK-no-op if a session is already active (re-arms the deadline). Returns single-byte status. |
| `0x06` | `SET_MODE` | `<mode:u8>` | `0` = AUTO, `1` = MANUAL. Persisted to `LOGMODE.CFG` on the SD root and applied immediately (AUTO + idle → start a session now; MANUAL never stops a running session). Returns single-byte status. |
| `0x07` | `GET_MODE` | none | Box replies one byte on FileData: `0` = AUTO, `1` = MANUAL. |
| `0x08` | `SET_TIME` | `<epoch_ms:u64-LE>` | Host pushes its current wall-clock millis (the box has no RTC). The box appends a `# SYNC epoch_ms=<...> tick_ms=<HAL_GetTick()>` comment line into the open `SensNNN.csv` and `GpsNNN.csv`, pairing the phone epoch with the free-running `ms` counter so replay tools can resolve absolute wall-clock with zero drift and without a GPS fix. Sent on every connect. Returns single-byte status (`0x00` even when idle — marker is a best-effort no-op then; `0xE3` if fewer than 8 epoch bytes). |
| `0x09` | `FW_BEGIN` | `<image_len:u32-LE><sha256:32>` | Start a firmware-over-BLE update (FOTA, F-FWU). Validates `image_len` ≤ a bank and erases the inactive flash bank. Returns single-byte status: `0x00` ready, `0xB0` busy (another op in flight), `0xE6` image too big, `0xE5` flash-erase failed, `0xE3` malformed (< 37 bytes). See "Firmware update over BLE" below. |
| `0x0A` | `FW_DATA` | `<offset:u32-LE><bytes…>` | Program a contiguous segment into the inactive bank at `offset`. `offset` must equal the box's current write cursor (a strictly smaller offset is treated as an idempotent retransmit after a lost ACK). On success the box replies a **4-byte** little-endian next-expected-offset (the host's flow-control + resume signal); on error a single byte `0xE7` bad-sequence or `0xE5` flash-program-failed. |
| `0x0B` | `FW_COMMIT` | none | Flush the final quadword, verify SHA-256 over the staged image, and — on match — activate it (toggle `SWAP_BANK` + reset). Replies `0xA0 FW_READY` then disconnects (the host sees the reset as success and reconnects to the new firmware). On mismatch replies `0xE4`; the running image is untouched. |
| `0x0C` | `FW_ABORT` | none | Discard the current FOTA session. The partially-written inactive bank is left stale (never activated) and re-erased by the next `FW_BEGIN`. Returns `0x00`. |
| `0x0D` | `GPS_BRIDGE` | `<on:u8>` | `1` = start, `0` = stop relaying raw u-blox UBX over BLE for the desktop GPS antenna survey. Since v0.0.41 the port is UBX-native anyway; while ON the box only throttles its own periodic NAV-PVT (10 Hz → 1 Hz, logging continues) so the survey's poll replies don't compete on the relay, and captures complete UBX reply frames out of the RX stream, notifying them back on **FileData** in ≤160-byte chunks while the FileSync FSM is idle. **Fire-and-forget — no reply.** Auto-disabled on disconnect; never persisted. Legacy hosts/firmware ignore it. The host must not run a LIST/READ while the bridge is on (both ride FileData). |
| `0x0E` | `GPS_TX` | `<raw bytes…>` | Forward the bytes straight to the u-blox UART. The desktop survey sends UBX poll frames (`NAV-PVT`/`NAV-DOP`/`NAV-SAT`/`NAV-SIG`/`MON-RF`) this way; replies come back as `GPS_BRIDGE` notifies. **Fire-and-forget — no reply.** No-op unless a bridge is active. |
| `0x0F` | `DISCONNECT` | none | Fire-and-forget host-requested link teardown. Routes to `ble_recover_lost_peer("host disconnect")` → real HCI_Disconnect from the box side + re-advertise. Sent by the macOS desktop before `cancelPeripheralConnection` (which only detaches the *app's* view; `bluetoothd` keeps the ACL alive with LL keepalives otherwise, leaving the box connected-in-limbo). iOS/Android tear the link down natively, so they never send this. |
| `0x11` | `GPS_POWER` | `<on:u8>` | Turn the u-blox MAX-M10S receiver on (1) or off (0 = UBX-RXM-PMREQ backup mode, ~tens of µA). Persisted to `GPSPWR.CFG` on the SD root and re-applied on every boot (with a wake-pulse at the start of `GPS_Init` so a module left asleep across an MCU-only reset gets detected). IMU + baro logging is unaffected; GPS rows naturally stop while off (the `# SYNC` anchor keeps replay time-aligned with no fix). Returns single-byte status. |
| `0x12` | `GPS_GET_POWER` | none | Box replies one byte on FileData: `0` = off, `1` = on. Legacy firmware (< v0.0.35) never replies — the host treats a timeout as "unknown, assume on". |
| `0x13` | `CAL_GET` | none | Box replies the 32-byte **calibration blob** (layout below). All hosts that connect to the same box see the same calibration — a per-box source of truth so a calibration set on the iPhone survives on the Desktop / Android without a re-tap. Legacy firmware (< v0.0.37) never replies — the host treats a timeout as "unknown, use local UserDefaults / config.toml as before". |
| `0x14` | `CAL_SET` | 32-byte calibration blob | Merge the incoming blob into the box's persisted `CAL.CFG`. **Merge is per-field, not blob-replace**: only fields whose valid-mask bit is set in the incoming blob overwrite the stored ones; unset bits leave the corresponding field untouched. Lets a host push a single new field (e.g. just `nosePlusY`) without knowing the box's current `magOffsetMg`. Returns single-byte status. |
| any other | reserved | — | Box replies `0xE3 BAD_REQUEST` on FileData. |

**LOG mode: AUTO (default) vs MANUAL.** There is still no STREAM_START /
STREAM_STOP and no LOG-vs-STREAM mode — live streaming (Section 3) is a
side effect of a client subscribing to the SensorStream CCCD and runs
concurrently with logging. What *is* configurable is **when SD logging
starts**:

- **AUTO** (default, and the behaviour when `LOGMODE.CFG` is absent or
  unreadable): a session opens automatically on every cold boot. There
  is no state the box can be left in where it silently stops recording —
  this is the data-safe default and matches the original always-on
  design (the mutually-exclusive LOG/STREAM mode inherited from the
  ThreadX firmware is gone; the polling architecture has no BLE↔SDMMC
  constraint).
- **MANUAL**: the box stays idle on cold boot and only records after a
  host `START_LOG`, for the requested fixed duration. **Deliberate
  tradeoff:** in MANUAL the box can be powered yet not recording, so a
  forgotten `START_LOG` (or no host in range) silently loses the run.
  This is opt-in, set by the host app via `SET_MODE`, never the default.

The mode is persisted on the SD card (`LOGMODE.CFG`, a one-line text
file — first byte `m`/`M` ⇒ manual, anything else ⇒ auto) so it survives
the hard power-cycle (F-PWR-5). Changing it needs no power-cycle:
`SET_MODE` applies at once.

### Box-persisted calibration (`CAL_GET` / `CAL_SET`, v0.0.37+)

Historically each host app (iPhone, Android, Desktop) kept its own copy
of the calibration in its own local store (UserDefaults /
SharedPreferences / `config.toml`). A `nosePlusY` toggle set on the
iPhone didn't reach the Desktop — three of the four calibration fields
are physical facts of the specific box (which end is the nose, hard-iron
of *its* magnetometer, the pose the user picked as "level" for *this*
board), so the box is the natural place to store them.

The `CAL_GET` (`0x13`) / `CAL_SET` (`0x14`) opcodes move that state onto
the box, persisted as a fixed-size blob in `CAL.CFG` on the SD root
(parallel to `LOGMODE.CFG` + `GPSPWR.CFG`). On connect the host issues
`CAL_GET`, seeds its local mirror from the reply, and thereafter reuses
its cache offline. On any local user-driven update (nosePlusY toggle,
"USB-C south" tap, "Zero here" tap) the host issues `CAL_SET` with just
that field's bit set, and the merge on the box updates only that field
in `CAL.CFG` — a second host can't clobber untouched fields.

**Blob layout — 32 bytes, little-endian.** Fits in one BLE notify.

| Offset | Size | Field | Encoding |
|---|---|---|---|
| 0 | 1 | `version` | Layout version, starts at `0x01`. Reserved values ≥ `0x02` let a future revision extend the blob without breaking older readers. |
| 1 | 1 | `valid_mask` | Bit `0` = nosePlusY, `1` = magOffsetMg, `2` = angleZeroRef, `3` = headingBiasDeg. In a `CAL_GET` reply, unset bits mean "not calibrated yet — host falls back to its own local value or default". In a `CAL_SET` write, unset bits mean "leave that field alone" so a host can update one field without knowing the others. |
| 2 | 1 | `nosePlusY` | `0` = nose is `-Y`, `1` = `+Y`. |
| 3 | 1 | reserved | Zero-fill. |
| 4 | 6 | `magOffsetMg[3]` | Hard-iron offset from the continuous compass auto-cal: 3 × `i16` (X, Y, Z in milli-gauss). Written back on a "wipe compass cal" tap; hosts should push updates conservatively (rate-limit or on-disconnect) to avoid SD-write churn. |
| 10 | 6 | `angleZeroRef[3]` | "Zero here" board-angle reference: 3 × `i16` in **tenths of a degree** ([pitch, roll, yaw]; range ±3276.7°). |
| 16 | 8 | `angleZeroAtEpoch` | Wall-clock epoch (ms) when "Zero here" was captured. `u64` LE. `0` = never zeroed. |
| 24 | 2 | `headingBiasDeg` | "USB-C points SOUTH" yaw bias: `i16` in tenths of a degree. |
| 26 | 6 | reserved | Zero-fill; extension room for future single-`i16`/`u32` fields under new mask bits. |

The `CAL_GET` reply is exactly this 32-byte blob (one notify), regardless
of how many mask bits are set — legacy hosts can tell "box has no cal
yet" from `valid_mask == 0`. The `CAL_SET` reply is one status byte
(`0x00` OK / `0xE2` IO_ERROR if SD write failed / `0xE3` BAD_REQUEST for
a wrong-size or bad-version payload).

`CAL.CFG` is loaded once on boot into a static RAM copy; `CAL_GET` reads
from RAM, `CAL_SET` mutates RAM and re-writes the whole 32-byte file
(cheap — one SD block, no cluster rewrite unless the file didn't exist
yet). Missing / corrupt / wrong-version `CAL.CFG` → RAM copy is zeroed
(`valid_mask = 0`), same as "unset".

### Box → host (FileData notifies)

Each opcode produces a sequence of FileData notifies. The host
correlates them with the most recent FileCmd write (the protocol is
strictly request-then-response, single-outstanding-request).

**LIST**

- Zero or more rows. Each row is one notify with payload
  `<name>,<size_decimal>\n` (ASCII, ≤ 20 B).
- Terminated by a single notify with payload `\n` (one byte).

**READ**

- Zero or more notifies. Each notify carries up to 20 raw bytes from
  the file, starting at `<offset>` (per the request) and continuing in
  file order until EOF.
- Terminated by a single notify with payload `0x00` (one byte = OK
  status). The host knows the expected total length from the matching
  LIST row (or its own high-water mark) so it can detect a short read.

**DELETE**

- A single notify carrying the status byte (table below).

**STOP_LOG**

- No FileData notify. The host observes the rotation via a subsequent
  LIST (the new `SensNNN+1.csv` appears).

### Status bytes

| Byte | Meaning | When |
|---|---|---|
| `0x00` | OK | DELETE succeeded; end-of-READ marker. |
| `0xB0` | BUSY | Opcode rejected because a LIST/READ is in flight, or because logging would conflict (logger ↔ host concurrency). |
| `0xE1` | NOT_FOUND | Filename not present on SD. |
| `0xE2` | IO_ERROR | SD read/write failed. |
| `0xE3` | BAD_REQUEST | Malformed payload (zero-length name, invalid offset, unknown opcode). |
| `0xA0` | FW_READY | (FOTA) Image verified — box is about to swap banks + reset. **Not an error.** |
| `0xE4` | FW_HASH_FAIL | (FOTA) Staged image SHA-256 did not match the host-declared digest. Image NOT activated. |
| `0xE5` | FW_FLASH_FAIL | (FOTA) Inactive-bank erase or program failed. |
| `0xE6` | FW_TOO_BIG | (FOTA) `image_len` exceeds one flash bank. |
| `0xE7` | FW_BAD_SEQ | (FOTA) `FW_DATA` offset gap, or `FW_COMMIT` with a short image. |

A 1-byte READ payload is **never** a status byte by coincidence — the
host distinguishes by checking whether the byte fits the file's
expected continuation (any value 0–255 is valid file content). The
"end of stream" marker is recognized by `0x00` arriving *after* the
expected total length has been received. (A 1-byte file containing
`0x00` and a 0-byte file with terminator are indistinguishable —
acceptable corner case.)

### Firmware update over BLE (FOTA) — F-FWU

Implements REQUIREMENTS.md F-FWU (moves the former OOS-4 in-scope; see the
governance note there). Lets the iOS / Android / desktop apps push a new
`firmware.bin` to the box over the existing FileSync link instead of opening
the case for USB-C DFU. Implemented in `fwupdate.c`; wired into `ble.c`'s
FileSync dispatch as opcodes `0x09`–`0x0C`.

**Dual-bank A/B, brick-safe by construction.** The STM32U585 is a 2 × 1 MB
dual-bank part with a `SWAP_BANK` option byte: whichever physical bank the
boot-ROM maps to `0x08000000` is "active", the other is always at `0x08100000`.
The app is linked to a single bank (`LENGTH=1024K`) and runs/logs from the
active bank while a new image streams into the **inactive** bank
(read-while-write, no execution stall). `SWAP_BANK` is toggled **only after**
the whole image is programmed and its SHA-256 matches the host-declared digest,
so a corrupt or interrupted upload can never brick the box — the previous image
stays intact in the other bank as an automatic rollback. There is **no
bootloader**: the boot-ROM performs the swap.

> **Brick gotcha (do not regress).** `HAL_FLASHEx_Erase` bank numbers follow the
> live `SWAP_BANK` state, so a hardcoded `FLASH_BANK_2` erases the *running*
> bank once swapped. `fwupdate.c` computes the inactive bank from `FLASH->OPTR`
> on every operation (`swapped ? FLASH_BANK_1 : FLASH_BANK_2`).

**Wire protocol.** Host → box (FileCmd `0x09`–`0x0C`, table above):

1. `FW_BEGIN <image_len:u32><sha256:32>` — box erases the inactive-bank pages the
   image will occupy (one 8 KB page per HAL call, IWDG fed between pages; ≤ ~1 s,
   a brief logging gap accepted), stores the expected digest, replies `0x00`.
2. `FW_DATA <offset:u32><bytes…>` — box buffers into 128-bit quadwords and
   programs them; replies the 4-byte next-expected-offset. **ACK-gated**: the
   host sends the next chunk only after this ACK, which serialises with the
   single FileCmd buffer and prevents overrun. A lost ACK → host resends the
   same offset → box re-ACKs the current cursor idempotently (cheap resume; a
   dropped *link* aborts the session via the stall watchdog and the host
   restarts from `FW_BEGIN`). Chunk size is the host's `min(char_value_length
   244, MTU−3) − 5` — `char_value_length` was raised 64 → 244 so a phone that
   negotiates a large MTU can push big chunks.
3. `FW_COMMIT` — box pads the final quadword, recomputes SHA-256 over exactly
   `image_len` bytes of the inactive bank, and on match notifies `0xA0` then
   toggles `SWAP_BANK` + resets. The host observes the disconnect as success and
   reconnects to verify the new build. On mismatch: `0xE4`, session ends, active
   bank untouched.

**Integrity, not authenticity.** SHA-256 guarantees the activated image is
exactly what the host sent (no corruption/truncation), but does not
authenticate the sender — consistent with OOS-7 (physical-possession-equals-
trust) and the fixed BLE PIN. Signed images (Ed25519) are a future option if the
trust model tightens; the digest field would become a signature.

**Runtime-hang rollback.** Verify-before-activate covers corruption; a verified
image that *hangs at runtime* is caught by a boot-attempt counter. `FW_COMMIT`
drops an SD marker `FWPEND.STA`; `FwUpdate_BootCheck()` (called right after
`SDFat_Mount()` in `main.c`, before sensor init) bumps the counter each boot and,
after `FW_MAX_BOOT_ATTEMPTS` (5) unconfirmed boots, flips `SWAP_BANK` back to the
previous image and resets. A healthy image clears the marker via
`FwUpdate_ConfirmBoot()` once it has run past `FW_CONFIRM_UPTIME_MS` (15 s). The
only residual brick path — a verified image that hangs *before* SD mount — is
the same USB-C DFU recovery that is the sole update path today, so it is never
worse than the status quo.

**State-machine integration (Section 11).** FOTA reuses the FileSync FSM: a
session is `FSM_FW_RECV`, rejects other ops with BUSY, rides the same READ
deadline (600 s) + stall (15 s) + disconnect guards, and `fsm_emergency_exit`
calls `FwUpdate_Abort()` to drop a partial session (the inactive bank is left
stale, never activated).

---

## 3. Live-stream sample format

`SensorStream` carries one packed binary snapshot per notify covering
**all** sensors (IMU + mag + baro + GPS). Fixed layout, little-endian
throughout, **46 bytes**. Notify cadence: **0.5 Hz** (one packet every
2 seconds — sufficient for live calibration and debugging; higher rates
are a future feature if needed).

Emitted only while a client has subscribed to the SensorStream CCCD;
no subscriber → no stream → zero cost. The emitter reuses the latest
sensor samples the logger already polled — it does not add sensor
reads, and it never pauses or replaces SD logging. While a FileSync
READ transfer is in flight the 0.5 Hz stream is skipped for that
window so the two don't contend for BLE bandwidth.

```
offset  size  field                   units             source
------  ----  ----------------------  ----------------  -----------------
  0       4   timestamp_ms            ms since boot     free counter
  4       2   acc_x                   mg                LSM6DSV16X
  6       2   acc_y                   mg
  8       2   acc_z                   mg
 10       2   gyro_x                  centi-dps         LSM6DSV16X (× 1.75)
 12       2   gyro_y                  centi-dps
 14       2   gyro_z                  centi-dps
 16       2   mag_x                   mgauss            LIS2MDL
 18       2   mag_y                   mgauss
 20       2   mag_z                   mgauss
 22       4   pressure_pa             Pa                LPS22DF (full int32)
 26       2   temperature_cC          0.01 °C           LPS22DF
 28       4   gps_lat_e7              degrees × 10^7    u-blox MAX-M10S
 32       4   gps_lon_e7              degrees × 10^7
 36       2   gps_alt_m               metres (signed)
 38       2   gps_speed_cmh           cm/h × 10         u-blox (≈ km/h × 100)
 40       2   gps_course_cdeg         centi-degrees     0..35999
 42       1   gps_fix_q               0=none 1=fix      GGA scale; since v0.0.41 from UBX NAV-PVT (gnssFixOK → 1)
 43       1   gps_nsat                # satellites      used in the solution
 44       1   flags                   bit field         see below
 45       1   gps_cn0_max             dB-Hz (u8)        strongest satellite C/N0 (NAV-SAT; GSV in NMEA fallback); 0 = no data
```

**flags** (bit 0 = LSB):
- bit 0: gps_valid (set when the most-recent GPS fix is fresh — within the last 5 seconds).
- bit 1: low_battery_warning_active.
- bit 2: logging_active (1 = the box is recording this snapshot to SD as well; with the no-mode design this is normally always 1).
- bits 3-7: reserved, must be 0.

**Gyro scaling**: the LSM6DSV16X at ±500 dps with 17.5 mdps/LSB has a
±9 deg/s range across an int16 — we'd lose precision. So we send
centi-degrees-per-second = raw_LSB × 1.75 truncated to int16. Range
±327.67 dps. Extreme spins are clipped (and logged in the error log if
it ever happens).

**Pressure**: LPS22DF gives 24-bit pressure in Pa (range ~30000-110000
Pa for normal altitudes). Fits comfortably in int32. We send the raw
Pa value, not hPa — host divides by 100 for display.

**GPS handling when no fix**: when the GPS module hasn't acquired a fix
yet (or has lost it), `gps_lat_e7` / `gps_lon_e7` are set to `0x7FFFFFFF`,
`gps_alt_m` / `gps_speed_cmh` to `0`, `gps_fix_q` to `0`, and the
`gps_valid` flag is cleared. Host detects "no fix" via the flag.

### MTU negotiation

46 bytes exceeds the default BLE 4.0 ATT MTU (23 bytes → 20 byte
notify payload). The box requests an MTU upgrade right after connect:

- HCI command `ACI_GATT_EXCHANGE_CONFIG` sent at connect with MTU 100.
- Most modern hosts (macOS, recent Android) accept this; the BlueNRG-LP
  reports the negotiated MTU in the response.
- If the host refuses or the negotiated MTU is too small, the box falls
  back to **chunked mode**: the same 46-byte snapshot is split into
  three sequential notifies (`0x00 <bytes 0-18>`, `0x01 <bytes 19-37>`,
  `0x02 <bytes 38-45>`). First byte of each chunk = sequence index.
  Host reassembles. At 0.5 Hz this adds negligible latency.

---

## 4. Battery status format

`BatteryStatus` packet, 8 bytes little-endian:

```
offset  size  field         units               source
------  ----  ------------  ------------------  ---------
  0       2   voltage_mV    mV                  STC3115
  2       2   soc_x10       0.1 %               STC3115
  4       2   current_x100  100 µA, signed      STC3115
  6       1   flags         bit field           see below
  7       1   reserved      0x00
```

**flags**:
- bit 0: low_battery_warning_active (SOC < 10 %).
- bit 1: logging_active (normally always 1 — see the no-mode design in Section 2).
- bits 2-7: reserved.

Notify cadence: once per minute while connected. Also notified
opportunistically whenever the low-battery flag transitions, so the GUI
sees the warning immediately on first crossing 10 %.

**Gauge configuration (v0.0.32+).** The STC3115 is programmed at boot with
the real battery model — 480 mAh HiMax 752535, Ri 160 mΩ, behind the
board's R83 = 50 mΩ shunt (`CC_CNF=484`, `VM_CNF=79`, ST's board-tuned
OCVTAB offsets, relax threshold 24 mA; constants in `Inc/config.h`
`PL_BATT_*`, values from ST's `STC3115_Battery_Conf.h` for
STEVAL-MKBOXPRO). Startup discriminates warm (gauge kept running through
the Hall cut — state untouched, SOC continuous), restore (gauge reset but
its 16-byte RAM survived — SOC reloaded from the 1 Hz backup that
`FUEL_Read` writes), and cold (true first power-up — SOC preset from OCV
through the corrected curve). Before v0.0.32 the gauge ran on POR defaults
(1957 mAh @ 10 mΩ model) with no state handling, so SOC re-preset from the
generic OCV curve at every boot — a plateau-resting Li-Po read "~55 %"
forever — and `current_x100` was scaled ~200× too low (µA/mA slip plus the
wrong shunt value).

Host can also `read` the characteristic anytime for an instantaneous
value (e.g. right after connecting, before the first periodic notify).

---

## 5. HCI command sequence (host → BlueNRG-LP)

The cold-boot sequence the firmware drives over polled SPI. All commands
are sent in order, each waiting for its Command Complete event before
proceeding (with a 1 s timeout). Failure at any step → box logs the
error, beeps the watchdog pattern, and parks the BLE task without
crashing the logger.

| # | Command | Opcode | Parameters / purpose |
|---|---|---|---|
| 1 | `HCI_RESET` | `0x0C03` | Reset chip state. Wait ≥ 150 ms after reset for chip to be ready. |
| 2 | `HCI_READ_LOCAL_VERSION_INFORMATION` | `0x1001` | Sanity check the chip is alive, logged. |
| 3 | `HCI_LE_SET_RANDOM_ADDRESS` | `0x2005` | Set a random static address derived from the STM32 96-bit UID (last 6 bytes, top two bits forced to `11` per BT spec for "static random"). |
| 4 | `ACI_GAP_INIT` | `0xFC8A` | Role = peripheral (0x01), privacy = disabled (0x00), name length = 8. |
| 5 | `ACI_GATT_ADD_SERVICE` | `0xFD01` | Primary service, UUID from Section 1. Reserve handles for 4 chars × (value + CCCD-where-applicable). |
| 6 | `ACI_GATT_ADD_CHAR` × 4 | `0xFD04` | One per characteristic. Properties + permissions per Section 1. |
| 7 | `ACI_GAP_SET_ADVERTISING_CONFIGURATION` | `0xFC97` | Interval 100-200 ms, connectable + scannable, channel map all 3. |
| 8 | `ACI_GAP_SET_ADVERTISING_DATA` | `0xFC98` | Flags (LE General Discoverable + BR/EDR not supported), complete local name "STBoxFs", service UUID list with our primary UUID. |
| 9 | `ACI_GAP_SET_SCAN_RESPONSE_DATA` | `0xFC9F` | Same name, optional manufacturer data with firmware version. |
| 10 | `ACI_GAP_SET_ADVERTISING_ENABLE` | `0xFC99` | Single advertising set (handle 0), no duration limit. |

After advertising is enabled, the BLE task enters its steady-state loop:
poll for chip events, dispatch to the GATT handlers, service active
FileSync / SensorStream / BatteryStatus operations.

**On connect**: immediately after the chip reports
`HCI_LE_CONNECTION_COMPLETE`, the box sends
`ACI_GATT_EXCHANGE_CONFIG` (`0xFD03`) requesting MTU = 100. Result is
remembered so the SensorStream emitter chooses single-notify vs
chunked mode (Section 3).

**On disconnect**: the chip generates a `HCI_DISCONNECTION_COMPLETE`
event. The firmware re-enables advertising (`ACI_GAP_SET_ADVERTISING_ENABLE`)
and clears any per-connection state (subscriptions, in-flight LIST/READ).
Bond data is not stored — every new connect goes through the PIN flow
fresh, eliminating the bond-state class of bugs we hit in the current
firmware.

---

## 6. SPI transport

Same hardware framing as today (we know it works from BLEDualProgram).
What changes is who drives it: pure polling, no ISR.

**Wire format** (BlueNRG-LP datasheet):

1. To send a command: master pulls CS low, writes the 5-byte master
   header `0x0a 0x00 0x00 0x00 0x00`, reads the 5-byte slave header back.
2. The slave header's bytes 1-2 (LE) tell how many bytes the master may
   write; bytes 3-4 tell how many bytes the slave has ready to send.
3. If the slave's "write buffer" is ≥ our command length, master writes
   the payload. Master pulls CS high.
4. To receive: master pulls CS low, writes `0x0b 0x00 0x00 0x00 0x00`,
   reads slave header. Bytes 3-4 = bytes pending. Master clocks out
   that many bytes (slave sends them). Master pulls CS high.

**IRQ pin**: BlueNRG-LP raises its IRQ pin (PB11) HIGH while it has
pending bytes. Master polls this pin every main-loop iteration; when
HIGH, drives a receive transaction and consumes bytes until the pin
goes LOW.

**No EXTI configuration.** We don't arm EXTI11. The IRQ pin is read as
a plain GPIO input. (This is the architectural fix that took 43
versions to find in the old firmware.)

**Reset pulse**: at boot, master drives the chip's RST pin LOW for
≥ 10 ms, then HIGH. Wait ≥ 150 ms for chip to be ready (per datasheet).
Then start the HCI sequence above.

---

## 7. On-SD file formats

### Filesystem

- FAT32, MBR or bare-BPB, single-volume mount. **Hand-rolled minimal
  append-only writer** (`sd_fatfs.c`) on top of polled `HAL_SD` — *not*
  FatFs/FileX. Just enough FAT32 to create / append / flush / read /
  delete 8.3 files in the root directory.
- Long filenames disabled (8.3 only) — smaller code, no Unicode tables.
  The writer never creates *or* cleans up LFN entries, so a card
  previously used on macOS/Windows can carry orphaned LFNs whose stale
  checksums make a host file browser show the wrong name↔content pairing.
  Cosmetic on-box (the firmware walks 8.3 short entries only); **reformat
  a reused card FAT32 before a field run** to clear host/old-firmware junk.
- Cluster cache 4 KB (single-sector FAT page, write-through, mirrored to
  FAT #2 when the volume has ≥2 FATs).

### Crash-consistency (F-PWR-5)

The box loses power abruptly at any instant (Hall-sensor supply
interrupter, no graceful shutdown), so every on-disk metadata update must
fail safe. **Invariant: commit the resource before the reference, and
remove the reference before the resource.**

- **Create** (`create_file_entry`): allocate the cluster, persist
  `FAT[fc]=EOC` *and* zero the body, *then* write the referencing
  directory entry. A cut in between leaves a lost cluster (FAT≠0,
  unreferenced), never a directory entry pointing at a cluster the FAT
  still calls free.
- **Delete** (`SDFat_Delete`): mark the directory entry `0xE5` and persist
  it *first*, *then* free the FAT chain. A cut in between leaves orphaned
  clusters, never a freed cluster still referenced by a live entry.

This prevents the **cross-link** class — a directory entry resolving into
another file's data — that the previous ordering produced under abrupt
power cuts (a field box's `GpsNNN.csv` read back `SensNNN` bytes). The
residual failure mode is now **lost clusters** that slowly shrink free
space; reclaiming them (and self-healing a card that already cross-linked)
needs a mount-time consistency pass — **TODO, tracked as a follow-up**.
Until then a periodic reformat recovers the space.

### `SensNNN.csv` (one row per sample, 100 Hz)

CSV header (one line):
```
ms,ax_mg,ay_mg,az_mg,gx_mdps,gy_mdps,gz_mdps,mx_mg,my_mg,mz_mg,p_hPa,t_C
```

Per row (decimal integers / floats):
```
123456,12,-34,1003,-150,200,-50,4500,-1200,300,1013.25,21.3
```

Column widths intentionally not fixed (CSV) — post-processor (stbox-viz)
handles variable widths. ~70-80 bytes per row average.

### `GpsNNN.csv` (one row per fix, 10 Hz when fixed)

```
ms,utc,lat,lon,alt_m,speed_kmh,course_deg,fix_q,nsat,hdop,cn0_max,sats_in_view
```

Empty (header only) until first valid fix. Rows match the structure used
by stbox-viz so the existing GPS-overlay code keeps working. Since
v0.0.41 the row is filled from **UBX NAV-PVT** (one frame per nav epoch)
instead of the NMEA RMC+GGA pair; column meanings on the wire are kept
NMEA-compatible:

- `fix_q` — `1` on a valid fix (PVT `gnssFixOK` + fixType 2D/3D), `0`
  otherwise. Deliberately mapped to the NMEA-GGA "1 = GPS fix" scale all
  consumers were built for; the raw fixType is not exposed.
- `hdop` — carries **pDOP × 0.01** (NAV-PVT has no hDOP; same scale,
  same "smaller = better" gating the tools do). True hDOP only in the
  NMEA fallback mode.
- `nsat` — satellites used in the solution (PVT `numSV`).
- `utc` — `hhmmss.ss` from the PVT UTC fields, written only when the
  module flags date+time valid.
- `cn0_max` / `sats_in_view` — from **UBX NAV-SAT** (every 10th nav
  epoch = 1 burst/s at 10 Hz; GSV feeds the same roll-up in the NMEA
  fallback). Between bursts the parser holds the last committed value
  and decays to 0 ("no data") after 3 s of silence.

### GPS module configuration (v0.0.42+ — baud first)

The u-blox MAX-M10S is configured **on every boot, RAM layer only**
(UBX-CFG-VALSET; the M10 never ACKs the legacy CFG-CFG save — verified
across 15 field boots — so nothing persists and the module always
cold-starts at factory 9600 baud NMEA). Recipe from Peter's u-center
verification (2026-07-13; 15 sats used / 3D fix at ~40 % sky view where
the 4-constellation default never fixed):

1. **CFG-UART1-BAUDRATE = 230400 — FIRST, and sent blind at 9600.**
   No baud scan (Peter: *"Es braucht keinen Baudratentest am Anfang.
   Nach dem Booten ist es immer 9600 Baud."*). The only reconfiguration
   done on the slow line. The ACK arrives at the **new** baud, so the
   local UART is re-opened at 230400 immediately after the command
   drains.

   **230400 is Peter's explicit decision** (2026-07-14: *"Peter will
   Baudrate von exakt 230400"*) and settles a real conflict in the
   record — do not "correct" it back. His u-center **screenshot**
   (2026-07-13, layer RAM) shows `CFG-UART1-BAUDRATE = 115200` in the
   config that measured 15 sats / 3D fix / C/N0 41 dB-Hz, while his
   **written instruction** the same day said 230400. v0.0.44 briefly
   shipped 115200 to match the screenshot; v0.0.45 reverts to 230400.
   Both rates work (~6 % vs ~12 % line utilisation) — never a
   correctness question, and unrelated to the missing fix (issue #10).

   **Confirm, don't scan.** The confirm step covers both boot worlds:
   - module @9600 (cold boot) → accepts the command; ACK lands at 230400.
   - module @230400 (**MCU-only reset**: FOTA `SWAP_BANK` reboot, IWDG,
     software reset) → drops our 9600 bytes as framing garbage (UBX
     checksums make a false positive impossible), but answers the
     **MON-VER poll** we then send at 230400 → *already there*.

   This case is not optional: **no MCU pin switches the GPS supply**
   (GPS-off is software only — UBX-RXM-PMREQ backup), so an MCU-only
   reset leaves the module powered and still on this session's RAM
   config. Assuming 9600 unconditionally would kill the GPS after every
   FOTA update until a magnet power-cycle.

   Only if *neither* confirms does a real probe run, over **115200 →
   38400 → 9600**, and the raise is retried from wherever the module
   actually is. Logged as `*** gps: baud fallback — module @<baud> ***`.
   **115200 must stay a candidate** until no v0.0.44 box is left in the
   field: v0.0.44 is the one release that used 115200 as the session
   baud, so a box updated by FOTA *from* v0.0.44 reboots MCU-only with
   its module still at 115200. Drop the candidate and the GPS dies on
   exactly the update that installs the current firmware.
   (38400 = left by pre-v0.0.41 firmware; 9600 = alive but ignoring us.)
2. **CFG-SIGNAL**: GPS + Galileo *on*, BeiDou + GLONASS *off* (one
   VALSET, 9 keys), then 500 ms settle (GNSS subsystem restarts).
3. **CFG-PM-OPERATEMODE = 0** (full power).
4. **UBX output on**: `UART1INPROT/OUTPROT-UBX = 1` (UBX *out* is off in
   the M10 factory default), `MSGOUT-UBX_NAV_PVT_UART1 = 1`,
   `MSGOUT-UBX_NAV_SAT_UART1 = 10`.
5. **NMEA silenced** (GGA/RMC/GSV/GSA/VTG/GLL/ZDA rates → 0) — only
   after step 4 ACK'd, so a partial failure never leaves the module mute.
6. **CFG-RATE**: measRate 100 ms / navRate 1 (10 Hz). Falls back to
   **1 Hz** if step 1 failed and the box is stuck on a slow line.

**Why baud first (changed in v0.0.42).** Peter, 2026-07-13: *"Die
Baudrate als ersten Schritt auf 230400 erhöhen. Mit 9600 kollabiert die
gesamte Kommunikation wenn mit den Folgebefehlen die Datenrate erhöht
wird."* v0.0.41 put the baud switch at step 6, so steps 2-5 ran at 9600
**while the module was still streaming its factory NMEA** — and step 4
adds NAV-PVT-every-epoch + NAV-SAT on top of that. 9600 baud is 960 B/s;
default NMEA alone eats most of it. The module's TX backs up, its ACKs
miss the 300 ms `ubx_send_retry` budget, and every later command reports
FAIL — the observed "collapse". Raising the baud first buys 23 kB/s of
headroom, so **no later command can starve its own ACK**. Same reasoning
now governs the step-6 fallback: on a slow line the nav rate stays at
1 Hz (5 Hz NAV-PVT alone is ~52 % of a 960 B/s line before NAV-SAT).

Every command is ACK-verified (3 retries); **every un-ACK'd command
writes a `***` errlog entry** (Peter's rule — it also latches the red
LED). If step 4 fails (dead box→module TX), NMEA keeps flowing at the
module's factory defaults and the NMEA parser acts as the RX-only
fallback data path. The `gps_diag` errlog line keeps its exact 8-key
format (the desktop errlog_check parses it strictly): `lines_good/bad`
count decoded/corrupt *units* — NMEA lines or UBX frames; `gga` counts
epochs (PVT or GGA), `rmc` counts valid fixes.

### `BatNNN.csv` (one row per second)

```
ms,v_mV,soc_x10,i_x100uA
```

### Error log `Error_Log_Pump_Tsueri_DD.MM.YYYY.log`

Free-form text, one line per event. Format:
```
[<ms> ms] <module>: <event>
```

Example:
```
--- Boot May 11 2026 14:23:01 ---
fw: rewrite-v0 build May 11 2026 14:23:01 | sensors=4 GPS=10Hz | flash ~50KB
reset: POR (CSR=0x0C004400)
[42 ms] sensors: lsm6dsv16x ok
[58 ms] sensors: lis2mdl ok
[63 ms] sensors: lps22df ok
[67 ms] fatfs: mount ok, free 7.2 GB
[71 ms] ble: hci_reset ok
[225 ms] ble: gatt ok, advertising as STBoxFs
[514 ms] gps: first fix UTC 14:23:01.500 47.401234N 8.512345E
```

Heartbeat once per minute:
```
[60042 ms] hb: free=7.2 GB | ble=disconnected | last_sample=60040 ms | gps=fix | soc=87%
```

### File naming + counters

On boot the firmware enumerates the SD root looking for `SensNNN.csv`.
Counter NNN starts at `000` and walks up until an unused number is
found. New session uses that number; the matching `GpsNNN.csv`,
`BatNNN.csv` use the same NNN.

---

## 8. Cooperative scheduler

`main()` runs a single non-preemptive loop. Each "task" is a function
called at a fixed cadence relative to the 1 ms SysTick.

```
                       cadence (ticks)  worst-case duration
                       ---------------  -------------------
sensors_acc_gyro       1                ~80 µs (one SPI burst)
sensors_mag            1                ~100 µs (I²C)
sensors_baro_temp      4                ~150 µs (I²C, only on this tick)
gps_parse_dma          1                ~50 µs (scan ring buffer)
battery_poll           1000             ~2 ms (I²C, once/sec — 10 B status+data read + 16 B gauge-RAM SOC backup write @ ~145 kHz, v0.0.32+)
logger_flush           100              ~10 ms (FatFs flush — worst case)
ble_poll               1                ~200 µs (chip-event drain)
ble_task               1                ~500 µs (filesync/stream emit)
watchdog_kick          1                ~5 µs
plausibility_check     1000             ~50 µs
heartbeat_log          60000            ~1 ms
buzzer_step            1                ~5 µs
```

Total worst-case loop iteration: ~12 ms when `logger_flush` lands in
the same tick as everything else, ~2 ms otherwise. Sensor sampling at
100 Hz (= 10 ms period) is robust as long as the flush worst-case
doesn't compound — we space flushes at 1 s intervals.

The 1 ms SysTick tick increments a `tick_count` counter. Tasks check
`(tick_count % N) == 0` to gate their cadence. Cheap, predictable.

---

## 9. Module boundaries

New project tree under `Projects/STEVAL-MKBOXPRO/Applications/Rev_C/PumpLogger/`:

```
PumpLogger/
├── Makefile
├── STM32U585AIIXQ_FLASH.ld
├── Application/
│   ├── Startup/   startup_stm32u585aiixq.s
│   └── User/      syscalls.c, sysmem.c
├── Inc/
│   ├── main.h
│   ├── config.h           ; constants (rates, sizes, UUIDs)
│   ├── sched.h            ; main-loop tick + cadence helpers
│   ├── logger.h           ; public API of logger module
│   ├── ble.h              ; public API of ble module
│   ├── filesync.h
│   ├── stream.h
│   ├── battery.h
│   ├── buzzer.h
│   ├── watchdog.h
│   ├── errlog.h
│   ├── gps.h
│   ├── sd_fatfs.h
│   ├── sensors_imu.h      ; LSM6DSV16X
│   ├── sensors_mag.h      ; LIS2MDL
│   ├── sensors_baro.h     ; LPS22DF
│   ├── sensors_fuel.h     ; STC3115
│   └── hci_chip.h         ; BlueNRG-LP HCI command sender + GATT helper
└── Src/
    ├── main.c             ; cold-boot init + main loop
    ├── sched.c            ; SysTick handler + cadence helpers
    ├── logger.c           ; sensor sample → CSV row → SD
    ├── ble.c              ; HCI bringup + event dispatch
    ├── filesync.c         ; LIST/READ/DELETE state machines
    ├── stream.c           ; SensorStream emitter (no mode-switch — always-log design)
    ├── battery.c          ; STC3115 polling + BatteryStatus notify
    ├── buzzer.c           ; beep pattern engine
    ├── watchdog.c         ; IWDG + plausibility check
    ├── errlog.c           ; error log writes
    ├── gps.c              ; UART4 DMA-circular parser
    ├── sd_fatfs.c         ; SD low-level + FatFs glue
    ├── sensors_imu.c
    ├── sensors_mag.c
    ├── sensors_baro.c
    ├── sensors_fuel.c
    ├── hci_chip.c         ; SPI framing + HCI command sender
    ├── stm32u5xx_it.c     ; SysTick handler only
    └── system_stm32u5xx.c
```

**Inter-module rules:**

- A module exposes a header with: an `init()` function, a `tick()` function
  (called by the scheduler), and any module-specific public API.
- No module includes another module's `.c` — only headers.
- The SD card is accessed exclusively via `sd_fatfs.c`. Logger writes
  rows; filesync reads files; nobody else touches FatFs directly.
- The SPI1 + BlueNRG-LP chip is accessed exclusively via `hci_chip.c`.
- The error log is the only shared resource modules use directly —
  every module can call `errlog_write("module: event")`.

---

## 10. Memory budget

| Region | Allocation | Target | Notes |
|---|---|---|---|
| .text | Code | ≤ 80 KB | Whole firmware in one flash bank. |
| .data | Initialized globals | ≤ 4 KB | |
| .bss | Zero-initialized globals | ≤ 28 KB | Includes the buffers below. |
| Main stack | | 4 KB | Only one stack in bare-metal — no per-thread stacks. |
| GPS RX ring | static array | 2 KB | Byte-IRQ ring; covers a full-line-rate 50 ms burst at 230400 baud. |
| SD scratch | FatFs work area | 4 KB | One cluster cache. |
| BLE TX/RX | HCI frame buffers | 2 × 256 B | Largest event packet ~250 B. |
| Sensor frame | latest sample of each | < 1 KB | Doubles as the SensorStream source. |
| File handles | open `FIL` objects | 4 × ~256 B = 1 KB | Sens/Gps/Bat/Errlog open simultaneously. |
| Filesync buffer | one notify-sized chunk | 20 B | Read buffer for READ-stream chunks. |
| Logger ring | sensor samples queued for SD | 4 × 256 B = 1 KB | Drained by `logger_flush` every 100 ms. |
| Stream ring | samples queued for notify | 4 × 20 B = 80 B | Tiny — notify cadence keeps pace with sensor rate. |
| Battery cache | latest STC3115 readings | 16 B | |
| **Total RAM** | | **≈ 14 KB** | Well under the 32 KB NF-SIZE-2 target. |

**No dynamic allocation** after `init()`. All buffers are static.
`malloc` / `free` are not linked in.

---

## 11. State-machine robustness — design standard

**Locked 2026-05-15** after the Phase 6 file-sync hang: a BLE READ
spun forever inside `ble_notify_blocking` while feeding `Watchdog_Kick`
from the retry loop. IWDG stayed happy, `Logger_Tick` starved, green
LED dark. The watchdog "worked" — but it was the wrong layer to catch
this case, because the hang was a foreseeable software condition
(client disconnected mid-stream / TX queue saturated), not an
unforeseeable CPU fault.

Principle: **the watchdog is the last resort, not a recovery
mechanism.** A watchdog reset costs the in-flight SD session, a
visible reboot, and a flash-bank-swap risk. Every state machine in
PumpLogger must catch its own foreseeable failure modes in software,
so the watchdog only ever fires for things we genuinely cannot
foresee (stack overflow, bus stall, memory corruption).

### The three required ingredients for every state machine

1. **Per-state deadline**, measured in SysTick ticks (not in retry
   counts — retries say nothing about wall-clock progress). Stored
   alongside the state itself as `state_entered_tick`. Caller
   checks `now - state_entered_tick > DEADLINE_TICKS` on each Tick
   entry.
2. **Defined emergency-exit transition** back to IDLE, with full
   resource cleanup: close any open file, reset buffers, clear
   state-machine-internal counters, release any held BLE handles.
   The exit path must never depend on the same resource that may
   have caused the hang (e.g. don't try to notify the client when
   we're aborting *because* notify is stuck).
3. **Errlog entry with context**, on every emergency exit:
   `state-machine: <name> aborted in state <s> after <ms>ms,
   reason=<code>`. Counter for repeated hits in the same session.

### Single source of `Watchdog_Kick()`

Kicked **only from the main loop, between Tick calls**. Never from
inside a state machine's working code, never from a retry loop,
never from a chunk-emission hot path. Rationale: a kick from a hot
loop maskiert exactly the hang the watchdog is meant to catch.

### Standard exit codes for the errlog

| Code | Meaning |
|---|---|
| `SM_OK` | normal completion (logged on debug, suppressed in prod) |
| `SM_TIMEOUT` | deadline exceeded |
| `SM_PEER_GONE` | resource owner disappeared (BLE disconnect mid-op) |
| `SM_IO_ERROR` | underlying driver returned non-recoverable error |
| `SM_BAD_STATE` | unreachable state hit (assertion-style, should never fire) |

### Audit table (current PumpLogger state-machines)

| State machine | File | Deadline? | Emergency exit? | Errlog? |
|---|---|---|---|---|
| BLE READ chunking | `ble.c` | ✅ 60 s total / 5 s stall | ✅ `fsm_emergency_exit` + close file | ✅ `fsm: <name> aborted code=N` |
| BLE LIST emission | `ble.c` | ⚠️ bounded by root-dir size | ✅ notify-fail aborts enumeration | ✅ `fsm: LIST aborted` |
| BLE notify retry | `ble.c` | ✅ 50 ms (FSM) / 500 ms (one-shot) | ✅ returns -1 to caller | ⚠️ via caller |
| FAT cluster-alloc scan | `sd_fatfs.c` | ⚠️ implicit O(N) | ❌ | ❌ |
| GPS UBX init wait-ack | `gps.c` | ✅ 200 ms | ✅ NAK/TO marker | ✅ |
| Logger_Tick cadence | `logger.c` | n/a (no loop) | n/a | n/a |
| SD `HAL_SD_WriteBlocks` | HAL | ✅ HAL timeout | ⚠️ HAL-internal reset | ⚠️ partial |

Status as of build #40 (2026-05-15): the READ / LIST / notify-retry
triplet that motivated this section is now compliant. FAT cluster-alloc
is the next candidate — its current O(N) bound is fine at our cluster
counts (few tens) but a deliberate deadline would future-proof it
against a fragmented multi-GB card.

### Convention for adding a new state machine

Every PR adding a new state machine in PumpLogger must include the
three ingredients from the start, and the author must add a row to
the audit table above. PR reviewers reject state machines that lack
any of the three. This keeps the design standard from rotting back.

---

## 12. Watchdog details

Three independent layers, by purpose:

| Layer | Period | Source | Action on trip |
|---|---|---|---|
| WWDG (primary loop-alive, with ISR) | 1.5 s, early-warning 50 ms before | hardware, kicked from main loop | ISR: errlog + beep, then reset |
| IWDG (belt-and-suspenders) | 2.0 s | hardware, kicked from main loop | direct reset, no ISR |
| Sensor-plausibility (data-alive) | 5 s | software, in watchdog_tick() | errlog + beep + `NVIC_SystemReset()` |

### WWDG (primary loop-alive watchdog)

- APB1 clock @ 80 MHz / 4096 prescaler / 64 (`WDGTB[2:0] = 7`) = 305 Hz
  count rate ≈ 3.3 ms per count.
- Window register set so a kick is valid at any time (no early-kick
  reset).
- Counter starts at `0x7F` (max). Re-armed back to `0x7F` on every
  main-loop iteration.
- Early-warning fires when the counter passes `0x40` going downward —
  about 50 ms before the reset point. ISR fires, runs:
  1. `errlog_write("watchdog: WWDG fired; main loop hung")` — atomic
     append to the open error-log file.
  2. `buzzer_blocking_pattern(WD_PATTERN)` — beeps a distinctive
     pattern even though we're about to reset (~30 ms beep, fits in
     the 50 ms slack before reset).
  3. `NVIC_SystemReset()` — explicit reset, doesn't wait for the
     hardware to fire.
- Counter back to `0x7F` on a normal main-loop iteration; we never
  reach the early-warning threshold in steady state.

### IWDG (belt-and-suspenders)

- LSI clock @ 32 kHz, prescaler `/16`, reload `0xFA0` (4000).
- Period = 16 × 4000 / 32000 = **2 seconds**.
- Kicked unconditionally at the top of the main loop.
- Purpose: if the WWDG ISR itself hangs (e.g. errlog write blocks on a
  stuck SD bus), IWDG eventually resets the chip anyway. No log
  preserved in that case — but reset *will* happen, the box won't sit
  bricked.

### Sensor-plausibility watchdog (software, in `watchdog.c`)

```
state per sensor: last_value_hash[N_SENSORS]  (uint32_t)
                  last_change_ms[N_SENSORS]   (uint32_t)

every plausibility tick (1 Hz):
  for each sensor:
    h = hash_of_last_sample(sensor)
    if h != last_value_hash[sensor]:
      last_value_hash[sensor] = h
      last_change_ms[sensor]  = now_ms()

  if ALL sensors have (now_ms - last_change_ms) > 5000:
    errlog_write("watchdog: all sensors frozen for 5 s, resetting")
    buzzer_blocking_pattern(WD_PATTERN)
    NVIC_SystemReset()
```

"Hash" = simple XOR of the sample bytes — cheap, sufficient to detect
"new data" vs "identical bytes". 1 LSB jitter on a single axis already
changes the hash.

Note: this watchdog runs *inside* the main loop, so it benefits from
all the housekeeping — IWDG / WWDG are kicked normally. If the main
loop is alive but the sensor pipeline is dead, this is the layer that
catches it.

---

## 13. Boot sequence

From cold-boot to "advertising + logging" — target ≤ 1 s.

```
 0 ms    Vector table → Reset_Handler → __libc_init_array → main()
         (SystemInit clocks set up by startup code: HSI → PLL → 160 MHz)
 1 ms    HAL_Init() (SysTick, NVIC priority groups)
 2 ms    Buzzer init + ONE BEEP
20 ms    LED init, green = ON solid
22 ms    Clock + reset-reason snapshot
24 ms    IWDG enable
30 ms    SDMMC1 init + FatFs mount
70 ms    Sensors init (acc/gyro, mag, baro, fuel gauge)
220 ms   GPS UART4 start, baud detect + per-boot CFG-VALSET config
         (see "GPS module configuration"; adds ~2-5 s on a cold boot —
         dominated by the 1.5 s listen windows and the 0.5 s
         constellation-switch settle; runs before the IWDG is armed)
260 ms   BLE chip reset pulse (10 ms low + 150 ms wait)
430 ms   HCI command sequence (Section 5) — ~50 commands at ~5 ms each
700 ms   Advertising enabled — STBoxFs visible
710 ms   Open first SensNNN.csv, GpsNNN.csv, BatNNN.csv, Error_Log_*
720 ms   Main loop starts
```

Slack: ~280 ms before the 1-second target. Plenty of room for HCI
retries if the chip is slow.

---

## 14. USB Mass Storage (Phase 9)

The box exposes its microSD card to a connected USB host as a standard
SCSI bulk-only MSC drive. Implementation: TinyUSB vendored under
`Middlewares/Third_Party/tinyusb/` (DWC2 driver + device core + MSC
class — CDC class present in the tree but disabled at config-time).

### Bring-up sequence (`Src/usb_msc.c`)

Reproduces the proven order from `fp-sns-stbox1/SDDataLogFileX/Core/
Src/usb_cdc.c`. Every step is load-bearing:

1. `__HAL_RCC_PWR_CLK_ENABLE()` + `HAL_PWREx_EnableVddUSB()` — without
   the PWR clock the SVMCR write silently no-ops and VDDUSB stays
   gated; symptom: tud_rhport_init returns OK but no bus events.
2. Route HSI48 → ICLK selector for the USB peripheral (HSI48 is
   already running for SDMMC).
3. CRS auto-trim of HSI48 from USB SOF — HSI48 alone is ±1-2 %, too
   loose for USB FS spec; CRS locks it within ±0.25 % once a host is
   on the bus.
4. `__HAL_RCC_USB_OTG_FS_CLK_ENABLE()`.
5. GPIO PA11 (D-) / PA12 (D+) on AF10.
6. NVIC priority 13 for `OTG_FS_IRQn`.
7. **Manually set `USB_OTG_FS->GUSBCFG |= USB_OTG_GUSBCFG_PHYSEL`** —
   ST's HAL_PCD_Init does this inside `USB_CoreInit` but TinyUSB's
   `dwc2_phy_init` does not; without it the data lines stay floating.

Then `tud_rhport_init(0, {DEVICE, FULL_SPEED})`. The cooperative
`tud_task()` runs from the main loop via `UsbMsc_Tick()`; the
OTG_FS_IRQ forwards to `tud_int_handler(0)`.

### Descriptors (`Src/usb_descriptors.c`)

- VID/PID: `0xCAFE / 0x4002` (TinyUSB test VID, MSC variant). Will be
  replaced with a real ywesee-assigned VID/PID before any product
  release.
- Device class fields = 0 (class-per-interface, as MSC requires).
- One interface, two bulk endpoints (`0x01` OUT, `0x81` IN, 64 B FS
  packet size), `TUD_MSC_DESCRIPTOR()` macro.
- Strings: "ywesee GmbH" / "PumpLogger SD" / placeholder serial
  (`0123456789ABCDEF`) / "PumpLogger MSC".

### SCSI callbacks → HAL_SD bridge

| TinyUSB callback | What it does | Bridge target |
|---|---|---|
| `tud_msc_inquiry_cb` | Returns vendor/product/rev strings | static `ywesee/PumpLogger SD/0001` |
| `tud_msc_test_unit_ready_cb` | "Are you ready?" once/sec | `SDFat_IsMounted()` |
| `tud_msc_capacity_cb` | Block count + block size | `HAL_SD_GetCardInfo(SDFat_RawHandle())` |
| `tud_msc_read10_cb` | SCSI READ(10) | `HAL_SD_ReadBlocks(...)` |
| `tud_msc_write10_cb` | SCSI WRITE(10) | `HAL_SD_WriteBlocks(...)` |
| `tud_msc_start_stop_cb` | Host eject hint | logged + ack |
| `tud_msc_is_writable_cb` | WP query | `true` |

`SDFat_RawHandle()` is the only sanctioned bypass of the
single-owner-of-HAL_SD rule (DESIGN.md §9). The serialization argument
holds because both `tud_msc_*` callbacks and `Logger_Tick`'s
`SDFat_Append/Flush` calls run from the main loop, never from an ISR —
the OTG_FS_IRQ just sets TinyUSB internal flags.

### Logger suspension

`tud_mount_cb` fires when the host has set the configuration and is
about to issue MSC traffic. The callback calls `Logger_Stop()`, which
flushes + closes the open CSVs and sets `g_active = 0`. The existing
`g_active` gates in `Logger_Tick` then suppress every `SDFat_*` write
for the duration of the mount.

`tud_umount_cb` (host eject) and physical USB unplug both end the
mount. On the next tick, `Logger_Init()` is called and AUTO mode opens
a fresh session; MANUAL stays idle until the next `START_LOG`.

### State-machine table addendum

Added to §11's audit table:

| Module | States | `g_phase` | Backoff | Resets | Errlog code | Notes |
|---|---|---|---|---|---|---|
| `usb_msc` | unmounted ↔ mounted | none (TinyUSB-internal) | n/a | `Logger_Stop` on mount; `Logger_Init` on unmount | `usb: …` | Driven by TinyUSB mount/umount callbacks |

### Memory budget addendum (§10)

| Module | Static RAM | Notes |
|---|---|---|
| TinyUSB (usbd + dwc2 + msc) | ~3 KB | endpoint FIFOs, control state, MSC SCSI buffer (1×512 B) |

Total firmware text grew ~15 KB.

---

## 15. Decisions left for implementation phase

Some details we'll decide as we hit them, rather than over-specifying
here:

- Exact byte format of `manufacturer data` in the advertising packet
  (firmware version + maybe device ID). Trivial.
- Exact 1 ms tick budget per sub-task — measured empirically in
  Phase 3 and 4.
- Whether to expose the firmware version as a separate read-only
  GATT characteristic, or keep it bundled in advertising. Probably
  advertising for simplicity.
- Live-stream throttling strategy when BLE link is slow.

---

**Status:** Phases 1-8 hardware-verified; Phase 9 (USB MSC) added in
this revision — see §14 above and Issue #5. Every phase ends with a
flashable binary on the actual box.
