# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Status

The repo is empty at the time of writing (just `README.md` + `LICENSE`). The
first real commit lands as **Peter's PR #18** in
`zdavatz/fp-sns-stbox1` — a ground-up bare-metal rewrite of the
SensorTile.box PRO firmware, currently named **PumpLogger**. Source for
that PR lives under
`Projects/STEVAL-MKBOXPRO/Applications/Rev_C/PumpLogger/` in the source
repo; once it's imported here the path may collapse to the repo root —
adjust the build instructions below if so.

## Read first

When working on this firmware, two docs are authoritative; read them
before changing anything non-trivial. Both ship in the PR:

- **`REQUIREMENTS.md`** (v0.3, locked 2026-05-11) — *what* the firmware
  must do. Sensor rates, modes, BLE protocol obligations, watchdog
  behavior, the F-PWR/F-LOG/F-SYNC/F-LIVE/F-BAT/F-WDG/F-ARCH ID series.
- **`DESIGN.md`** (v0.2, locked 2026-05-11) — *how*. GATT UUIDs, FileSync
  wire protocol, SensorStream byte layout, HCI command sequence, SPI
  framing, on-SD CSV schemas, scheduler cadences, module boundaries,
  memory budget, watchdog implementation, boot timeline.

Don't re-derive anything that's already pinned in those docs — change
the doc first if the requirement actually shifted (REQUIREMENTS.md
Section 10 meta-rule 3).

**Logging modes (REQUIREMENTS.md Section 2 + DESIGN.md "Host → box"
reconciled):** there is no LOG/STREAM mode — SD logging and the
SensorStream notify run concurrently (streaming is a side-effect of a
client subscribing to the CCCD). What *is* configurable is **when SD
logging starts**: `AUTO` (default — session on every cold boot, the
data-safe always-on behaviour; also the fallback when `LOGMODE.CFG` is
absent/unreadable) vs `MANUAL` (boot idle, record only on host
`START_LOG [dur]` for a fixed duration). The mode is set by the host app
via the `SET_MODE` opcode (`0x06`), reported by `GET_MODE` (`0x07`),
and persisted in `LOGMODE.CFG` on the SD root. `MANUAL` is opt-in and
carries a deliberate silent-data-loss tradeoff (forgotten `START_LOG`
= lost run). Opcodes: `0x05 START_LOG`, `0x06 SET_MODE`, `0x07
GET_MODE`, `0x08 SET_TIME` (see DESIGN.md table).

**Host time-sync (`SET_TIME 0x08`, v0.0.10+).** The box has no RTC — the
CSV `ms` column is a free-running `HAL_GetTick()` counter that starts at 0
on cold boot. So on every BLE connect the host (iPhone / Android / desktop)
pushes its current wall-clock millis via `SET_TIME 0x08 <epoch_ms:u64-LE>`,
and `Logger_WriteSyncMarker` (logger.c) appends a `# SYNC epoch_ms=<u64>
tick_ms=<HAL_GetTick()>` comment line into the open `SensNNN.csv` AND
`GpsNNN.csv`, then flushes both. Pairing the host epoch with the box tick
lets the replay tools (`stbox-viz animate`, the iOS/Android apps) map every
logged row to absolute wall-clock with zero drift and **without needing a
GPS fix**. Because the firmware is a single-threaded cooperative superloop,
`Logger_WriteSyncMarker` is safe to call straight from the BLE command
handler — `Logger_Tick` and `BLE_Tick` never overlap, so no queue/lock is
needed. The marker is a `#`-comment line every CSV parser skips as a data
row; it's a best-effort no-op when no session is open.

## Hardware target

- Board: **STEVAL-MKBOXPRO Rev_C only.** Rev_A/B and STWIN.box stay on
  the old firmware tree.
- MCU: STM32U585AIIxQ, Cortex-M33 @ 160 MHz (HSI → PLL; HSE is unusable
  on the 3.3 V board mod).
- BLE chip: BlueNRG-LP / STM32WB07_06 on SPI1, polled. **EXTI11 is
  intentionally not armed** — the IRQ pin is read as plain GPIO. This is
  the architectural fix that took 43 firmware revisions to find in the
  predecessor stack; do not "re-enable" it.
- Sensors: LSM6DSV16X (SPI2), LIS2MDL + LPS22DF + STTS22H on I²C2,
  STC3115 fuel gauge on I²C4. GPS u-blox MAX-M10S on UART4 @ 230400 baud
  (raised per boot from factory 9600 — see the v0.0.41 section), captured
  by per-byte RX IRQ into a 2 KB ring.
- **GPS wiring (authoritative — verified against ST schematic Rev 3
  Fig. 3 + UM3133 Rev 7).** The box has **no STMod+ connector**; UART4
  (PA0/PA1) is exposed only on **JP2, the STLINK programming connector**
  (Samtec FTSH-107-01-L-D, 2×7, 1.27 mm). JP2 **pin 13 = UART4_RX =
  PA1** (MCU in), **pin 14 = UART4_TX = PA0** (MCU out), **pin 7 & 11 =
  GND**, **pin 1 = the 1V8 rail** (JTAG pull-up ref — *not* 3.3 V, not a
  supply). GPS 3.3 V must come from the **JP4** extension-board rail
  (domain switch on 3 V), meter-verified; never from any JP2 pin. Pins
  4/6/8/10/12 are SWD/NRST — don't bridge. Don't re-derive this from the
  schematic; the full hand-soldering procedure (incl. the mandatory
  pre-solder continuity checks and the harmless TX/RX-swap fallback) is
  in **`GPS_SOLDERING.md`** / `pdf/GPS_SOLDERING.pdf`.
- Power: Li-Po + wireless charging. **Hall sensor + magnet are wired as
  a hardware supply-rail interrupter**, not as a signal to the MCU. The
  firmware has no sleep modes — only "running" or "no power" — and no
  graceful-shutdown path (F-PWR-5). The on-SD format and flush cadence
  must tolerate a sudden cut.

## Build and flash

The PR's Makefile lives at
`Projects/STEVAL-MKBOXPRO/Applications/Rev_C/PumpLogger/STM32CubeIDE/Makefile`
and walks up six levels (`ROOT = ../../../../../..`) to pick up
`Drivers/STM32U5xx_HAL_Driver/` and `Drivers/CMSIS/` from the
fp-sns-stbox1 tree. If/when the code is moved into this repo, that
relative root must be adjusted (or the HAL/CMSIS drivers vendored
alongside). Don't assume the build works until the path layout has
been verified.

Toolchain (override per `Makefile`): `arm-gnu-toolchain` at
`$HOME/.software/arm-gnu-toolchain/bin/`; override with
`make TOOLCHAIN=/path/to/bin`.

```sh
cd <PumpLogger>/STM32CubeIDE
make clean && make
# outputs: build/PumpLogger.elf, build/firmware.bin,
#          build/firmware-v<N>.bin (N from .build_counter, bumped per build)
```

Flash via DFU (hold the user button while plugging USB-C to enter DFU):

```sh
dfu-util -d 0483:df11 -a 0 \
  -s 0x08000000:mass-erase:force \
  -D build/firmware.bin
```

`:mass-erase:force` is mandatory — there is a known bank-swap bug in the
predecessor firmware that leaves stale code at the alternate bank.

The `force-errlog` target in the Makefile recompiles `errlog.c` every
build so `__DATE__` / `__TIME__` in the boot banner always reflects the
actual binary. Don't remove it.

## Firmware update over BLE (FOTA) — `fwupdate.c`

The box can be reflashed **wirelessly** from the iOS / Android / desktop apps —
USB-C DFU is now only the recovery path + the one-time bootstrap of the first
FOTA-capable image. Implemented in `Src/fwupdate.c` (+ `Src/sha256.c`), wired
into `ble.c`'s FileSync dispatch as opcodes `0x09 FW_BEGIN` / `0x0A FW_DATA` /
`0x0B FW_COMMIT` / `0x0C FW_ABORT`. Authoritative spec: **DESIGN.md §"Firmware
update over BLE (FOTA)"** + **REQUIREMENTS.md F-FWU** (the former OOS-4, moved
in-scope 2026-06-19, pending Peter's review per the change-doc-first meta-rule).

**Dual-bank A/B, brick-safe.** STM32U585 = 2 × 1 MB banks + `SWAP_BANK` option
byte. The app is now linked to **one bank** (`STM32U585AIIXQ_FLASH.ld`
`LENGTH=1024K`, was 2048K) and runs/logs from the active bank while the new image
streams into the inactive bank (read-while-write). `SWAP_BANK` is toggled + the
box reset **only after** the whole image is programmed and its **SHA-256** matches
the host-declared digest — so a corrupt/interrupted upload can never brick the
box; the old image stays as automatic rollback. **No bootloader** (boot-ROM does
the swap; app stays linked at `0x08000000`).

**Three brick-safety invariants — do not weaken** (they're load-bearing, see the
ST gotchas the design is built around):

1. Only ever erase/program the **inactive** bank, computed live from
   `FLASH->OPTR & FLASH_OPTR_SWAP_BANK` (`swapped ? FLASH_BANK_1 : FLASH_BANK_2`).
   HAL erase bank numbers follow the swap state — a hardcoded `FLASH_BANK_2`
   erases the *running* bank once swapped. The inactive bank is always at
   `0x08100000`.
2. `SWAP_BANK` toggled only after SHA-256 verify (`FwUpdate_Commit` →
   `FwUpdate_Activate`). The active bank is never touched on any error path.
3. Runtime-hang rollback via the SD marker `FWPEND.STA`: `FwUpdate_BootCheck()`
   (called in `main.c` right after `SDFat_Mount`) bumps a boot counter and
   reverts the swap after `FW_MAX_BOOT_ATTEMPTS` (5) unconfirmed boots;
   `FwUpdate_ConfirmBoot()` (called from the superloop) clears it after
   `FW_CONFIRM_UPTIME_MS` (15 s) of healthy uptime.

SHA-256 is software (`sha256.c`, public-domain, verified against FIPS vectors) —
the U585 has no HASH peripheral and `HAL_CRC` isn't linked. The FileCmd
`char_value_length` + `g_cmd_buf` were raised 64 → 244/256 so a phone that
negotiates a large MTU can push big `FW_DATA` chunks; the offset-based protocol
works at any chunk size. The IWDG is really **~8 s** (LSI/64, reload 4000) — the
`config.h` `PL_IWDG_PERIOD_MS 2000` and old `ble.c` "2 s" comments are stale;
`fw_erase_inactive` feeds `Watchdog_Kick()` per 8 KB page anyway.

## Architecture in 30 seconds

`main()` runs a single non-preemptive loop driven by the 1 ms SysTick.
Each module (logger, ble, sensors_*, gps, battery, buzzer, watchdog,
errlog) exposes `init()` + `tick()` and is called at a fixed cadence
relative to `tick_count`. Cadences live in `Inc/config.h`
(`PL_CADENCE_*`).

**Hard rules** (these aren't style — violating them resurrects bugs the
predecessor stack hit):

- **Polling-only.** No application-level ISRs. Permitted exceptions are
  listed in F-ARCH-7: SysTick (1 ms tick), IWDG (no ISR, just resets),
  the WWDG early-warning ISR (last-gasp errlog + beep before reset),
  the UART4 RX byte IRQ for GPS NMEA, and (Phase 9, Issue #5) OTG_FS
  for USB MSC enumeration timing. Anything else — SPI/I²C/SDMMC/EXTI/
  DMA-complete — runs polled.
- **No dynamic allocation after `init()`.** `malloc`/`free` are not
  linked in. All buffers are static; sizes are budgeted in DESIGN.md §10.
- **Single owners.** Only `sd_fatfs.c` touches the SD/FatFs. Only
  `hci_chip.c` (in `ble.c` in the PR's flat layout) drives the SPI to
  the BlueNRG-LP. No module includes another module's `.c` — headers
  only.
- **Logger never depends on BLE for forward progress** (F-ARCH-3). The
  box logs perfectly with no host ever connecting. FileSync waits and
  retries when SD is busy; it never blocks logging (F-ARCH-4).
- **SD logging is always on.** There is no mode the box can be left in
  where it silently stops recording. SensorStream is concurrent with
  logging, gated only by CCCD subscription. See DESIGN.md §2.
- **No bond storage.** Every new connect goes through the PIN flow
  fresh (`123456`). This deliberately eliminates the bond-state bug
  class the predecessor firmware hit.
- **BLE advertise name is `STBoxFs`** (7 chars; max 8 for the BLE name
  field). The current MovementLogger GUI scans for this exact name.
  PR also references `PumpTsueri` as the field-test name — keep
  `PL_FW_BLE_NAME` (Inc/config.h) as the single source of truth.
- **GAP Device Name characteristic value must be written explicitly.**
  `aci_gap_init` only sizes the GAP Device Name characteristic; it
  leaves the BlueNRG-LP SDK's factory default in place, which surfaces
  on macOS Core Bluetooth as `BlueNRG [<configured>]` and breaks any
  client doing exact-name matching. After `aci_gap_init`, capture the
  returned `dev_name_char_handle` and issue
  `aci_gatt_srv_write_handle_value_nwk` (OCF=0x106) with the clean
  `BLE_ADV_NAME`. Both the advert AD packet *and* the GATT
  characteristic must carry the same string. Fixed in v0.0.3; see
  `Src/ble.c` around the GAP init + write block and
  zdavatz/movement_logger_firmware#1 for the macOS symptom.

## BLE peer-gone watchdog (v0.0.13+) — `ble.c`

**The "connected-in-limbo" deadlock + its fix (issue #4).** The box is BLE
peripheral-only with a **polled** BlueNRG-LP (EXTI11 deliberately off). When a
central drops the link **uncleanly** — iOS lock-screen suspend, BT power-nap,
app force-quit, carried out of range — the chip **often never raises
`Disconnection_Complete`**. The only paths that cleared `g_conn_handle` and
re-advertised were (a) that event and (b) the FileSync FSM's stall/deadline/
disconnect guards — but `fsm_advance()` returns immediately when
`g_fsm.state == FSM_IDLE`, so **the idle-connected case (app subscribed, no
active READ) had no recovery.** Result: `g_conn_handle` stayed set, the
superloop kept logging (so the IWDG kept being fed — **no reset**), and the box
was invisibly "connected", unreconnectable without a power-cycle. The
must-power-cycle symptom is the tell that it's a BLE-layer limbo, **not** a
brown-out reset (a BOR self-recovers in ~8 s). Field-reported repeatedly;
the original v0.0.13 close only covered the active-transfer path.

**Fix (shipped v0.0.13):** an application-level peer-gone watchdog in
`BLE_Tick`, independent of the FSM. `g_last_peer_seen_ms` is refreshed on
connect, on any inbound write (`ACI_GATT_SRV_ATTRIBUTE_MODIFIED`, ecode
0x0C01), and on every **successful** `ble_notify` — a notify only drains once
the peer ACKs at the link layer, so it also covers a passive live-stream viewer
that never writes back. If connected with no liveness evidence for
`PEER_GONE_DEADLINE_MS` (**90 s** — deliberately above the 60 s battery-only
notify period so a battery-only subscriber isn't false-dropped) it calls
`ble_recover_lost_peer()`. That helper is **Build #57's proven teardown**
(`ble_hci_disconnect` + clear `g_conn_handle`/subscriptions + `ble_adv_enable`)
**extracted and now shared** by both `fsm_emergency_exit` and the watchdog — one
recovery sequence, two callers. The watchdog runs every tick *before* the
`irq_high()` early-return so it fires on a dead-quiet link; `fsm_advance()` may
have already cleared `g_conn_handle`, so it won't double-fire. No new IRQ, no
conn-param/supervision-timeout request (macOS ignores those — the robust lever
is the local watchdog). Host clients (iOS/Android/desktop) auto-reconnect on
re-advertise. **Don't lower the 90 s** without re-checking it against every
notify cadence (stream 2 s, battery 60 s). See `Src/ble.c` and issue #4.

## Large-file FileSync stability (v0.0.15+) — `ble.c`

**The "connection often gets lost when syncing very large files (700 KB+)" fix.**
A big READ is thousands of 240 B notifies and can sit on-air for minutes; two
things made long files drop far more than short ones:

1. **On-air time.** The box never asked the central for a faster connection
   interval, so throughput was whatever the central defaulted to — minutes per
   file, and every extra second is another chance for a transient RF/host drop.
   Fix: ~1 s after connect, `BLE_Tick` fires one **L2CAP connection-parameter
   update request** (`ble_request_fast_conn_params`, ACI opcode `0xFD81`) asking
   for **15–30 ms** interval, latency 0, 4 s supervision timeout. It's deferred
   via `g_conn_param_due_ms` (set on connect, cleared on disconnect/recover) so
   it never races pairing or re-enters the connection-complete handler. Best-
   effort: Android/Linux honour it, macOS often imposes its own — harmless no-op
   then. Logged as `ble: conn-param-req fast 15-30ms rc=…`.

2. **The 15 s stall watchdog force-disconnected on legitimate host pauses.**
   During a long READ the host pauses draining for many seconds (writing the
   growing mirror to disk/SQLite, app briefly backgrounded, macOS BT power-nap);
   the shared 15 s stall in `fsm_advance` then tore the link down. READ now uses
   `FSM_READ_STALL_DEADLINE_MS` (**45 s**); FW_RECV keeps the tighter 15 s (a
   firmware upload is host-driven and should push steadily). 45 s stays below
   the 90 s peer-gone watchdog (which, gated on real liveness evidence, still
   catches a genuinely vanished peer) and the 600 s read deadline, and a truly-
   dead link is caught far sooner by the chip's LL supervision timeout raising
   `Disconnection_Complete`.

Both levers are throughput/exposure reductions, not protocol changes — the
host apps already resume a dropped transfer from `offset = bytes-on-disk`
(desktop `run_sync_diff`), so these just make drops rare instead of frequent.
**Don't raise the READ stall above the 90 s peer-gone deadline** — that would
let a genuinely idle-connected limbo survive. See `Src/ble.c` and issue #4.

## Post-drop re-advertise recovery (v0.0.20+) — `ble.c`

**The "box invisible until power-cycle after a *large-file* mid-READ drop" fix.**
This is the recovery counterpart to the stability levers above: those make drops
*rare*; this makes the box always come *back* after one. Root cause was a
one-shot dead-end that only bit long transfers:

1. **`ble_aci_cmd` ate the disconnect event.** During a big READ the box calls
   `ble_notify` → `ble_aci_cmd` on essentially every tick, and `ble_aci_cmd`'s
   response-poll loop **discarded every HCI event that wasn't its
   CommandComplete** — including `Disconnection_Complete` (`0x04 0x05`). So a
   drop mid-transfer was silently eaten, the clean `0x05` re-advertise handler
   never ran, and the box thought it was still connected. Small files return to
   IDLE in ~1–2 s so the drop landed *outside* the notify loop and recovered
   cleanly; large files stayed in the loop for minutes → dead-end. Fix: the poll
   loop now **latches** `Disconnection_Complete` (`g_disconnect_latched`) instead
   of discarding it; `BLE_Tick` drains the latch right after `fsm_advance()` and
   runs the same teardown (clear handles, `fsm_emergency_exit` if not IDLE,
   re-advertise) — immediate recovery instead of waiting the 90 s peer-gone
   watchdog.
2. **`ble_recover_lost_peer` was a one-shot.** It zeroed `g_conn_handle` *before*
   advertising was confirmed and did a **single unretried** `ble_adv_enable()`;
   that also disarmed the peer-gone watchdog (gated on `g_conn_handle != 0`), and
   nothing periodically re-armed advertising. One failed re-advertise — exactly
   what a saturated ACL TX queue right after a big transfer produces — left the
   box emitting **no connectable ADV**, unreachable by *any* central (desktop or
   mobile) until reboot. Fix: it now retries the re-advertise ×3 and re-enters
   when disconnected-but-not-advertising.
3. **New durable backstop: periodic advertising re-arm.** `BLE_Tick` now
   re-enables advertising every `ADV_REARM_INTERVAL_MS` (**3 s**) whenever the box
   is *not connected and not confirmed on air*, so a failed re-advertise can
   never wedge the box until a power-cycle. Tracked by `g_adv_active` (true iff
   the last `ble_adv_enable()` succeeded — distinct from `g_advertising`, the
   never-reset boot flag; cleared on connect). The re-arm self-stops once adv is
   confirmed up. **Don't gate the re-arm on `g_advertising`** (the boot flag) —
   it must run whenever off-air.

The desktop side complements this with retrieve-by-identifier + a held pending
connect (`movement_logger_desktop` v0.0.36/0.0.37), but **no host change can
reach a box emitting no connectable ADV** — the re-advertise re-arm here is the
actual fix for that tail. Look for `ble: periodic re-adv rc=0` /
`ble: latched-disconnect re-adv=0` in `ERRLOG.LOG` as proof the dead-end is gone.

## Host-requested disconnect (v0.0.22+) — `0x0F`, `ble.c`

**The "Mac-disconnect makes the box invisible to the iPhone" fix.** On macOS the
desktop's `cancelPeripheralConnection` tears down only the *app's* view of the
link; `bluetoothd` keeps the ACL alive with LL keepalives, so the box never sees
`LL_TERMINATE`, its supervision timer never fires, and it stays connected-in-limbo
advertising non-connectably — the next central (the iPhone) then can't connect
until a box power-cycle. iOS tears the link down for real, so it never triggers
this (Peter: "Mac-disconnect blockiert das iPhone; iPhone-disconnect lässt den Mac
problemlos neu verbinden"). There is no CoreBluetooth API for the host to force
the physical teardown. Fix: the desktop sends a new fire-and-forget opcode
**`0x0F FSYNC_OP_DISCONNECT`** right before it cancels; the FileData dispatch
routes it to **`ble_recover_lost_peer("host disconnect")`** — the same proven
`HCI_Disconnect`-from-the-box-side (real `LL_TERMINATE`) + re-advertise path the
watchdogs use. Purely additive: only the Mac sends `0x0F` (iOS/Android never do,
so their behaviour is unchanged), and it only affects the current connection.
Look for `ble: cmd DISCONNECT (host requested)` in `ERRLOG.LOG`.

## GPS power on/off (battery-save, v0.0.35+) — `0x11`/`0x12`, `gps.c`

The box can power its u-blox MAX-M10S **off** to save battery when GPS is faulty or
unused. Two FileSync opcodes mirror the `SET_MODE`/`GET_MODE` design:
**`0x11 FSYNC_OP_GPS_POWER <u8 on>`** (1 = on, 0 = off) applies + persists and
replies one status byte (`0x00` = OK), exactly like `SET_MODE`;
**`0x12 FSYNC_OP_GPS_GET_POWER`** (no payload) replies one byte (1 = on / 0 = off),
like `GET_MODE`. Public API: `int GPS_GetPower(void)` / `int GPS_SetPower(int on)`
(`Inc/gps.h`).

**Off = UBX-RXM-PMREQ backup mode** (~tens of µA vs ~25 mA acquiring). Uses the
16-byte v0 PMREQ (flags bit1 = backup, wakeupSources bit3 = uartrx); PMREQ is
**fire-and-forget** (not ACK'd). **On** wakes the module with a short `0xFF` UART
burst (`gps_wake_pulse`) — software backup keeps the module's RAM powered, so it
resumes this session's config (the original "held in BBR via CFG-CFG-SAVE" story
was wrong — the M10 never ACKs that save; see the v0.0.38/v0.0.41 sections).

**Persisted to `GPSPWR.CFG`** on the SD root (first byte `f`/`F` = off, anything
else = on — same pattern as `LOGMODE.CFG`; cached in `g_power`, `-1` = unread).
**Re-applied on boot:** `GPS_Init` runs its normal baud-lock/config, saves config
to BBR, then immediately enters backup if the persisted state is off. A
**wake-pulse was added at the very start of `GPS_Init`** (before baud detection)
so a module left asleep across an MCU-only reset still gets detected. `GPS_Tick`
early-exits while off, discarding any trickled bytes so the RX ring can't wrap.

Logging is unaffected — IMU + baro keep recording; GPS rows naturally stop (the
logger only writes a GPS row on a fresh fix), and the phone-clock `# SYNC` anchor
(`SET_TIME`) keeps replay time-aligned with no GPS fix. Purely additive: legacy
hosts that never send `0x11`/`0x12` leave GPS on. Bumped `PL_FW_VERSION` →
**0.0.35** (`Inc/config.h`).

## Deferred DELETE-reply notify (v0.0.36+) — `ble.c`

**The "rapid multi-file DELETE times out at 20 s" fix (host complaint: "I still
manage to hang it").** `SDFat_Delete` on the box completes fast; the symptom was
that its 1-byte status notify sometimes couldn't drain inside
`ble_notify_try`'s 500 ms window when the ACL was briefly congested (competing
SensorStream / BatteryStatus notifies, macOS BT power-nap, GATT credits starved
after a burst of DELETEs). The firmware then logged the failure internally and
moved on, but the host saw nothing and spun its own 20 s watchdog on a completed
op → forcing a disconnect + full reconnect.

DESIGN.md §11 forbids sitting in a longer `HAL_Delay(1)` loop
(**no `Watchdog_Kick` inside a retry loop** — the IWDG must be free to catch a
genuinely stuck bus). The fix respects that: a **one-slot deferred-reply
mechanism**. If `ble_notify_try` fails in the initial 500 ms window,
`ble_notify_or_defer` parks the payload in `g_pending_reply`; every subsequent
`BLE_Tick` calls `ble_flush_pending_reply` at its top, which re-tries the notify
until it drains **or 10 s expires** (safely under the host's 20 s watchdog).

Single slot is enough: status-byte replies (`DELETE`, `SET_MODE`, `SET_TIME`,
`GPS_POWER`, `CAL_SET`) are for one-in-flight ops and the client's state
machine won't queue a new one until the current resolves. `LIST` / `READ`
streams still abort on notify failure by design (that IS the peer-gone
signal). Currently only DELETE calls `ble_notify_or_defer`; the same treatment
can be added to any other status-byte reply that also occasionally gets eaten.

Look for these lines in `ERRLOG.LOG` after a rapid delete run:
```
ble: DELETE 'SENS040.CSV' s=0 st=0x00
ble: notify deferred (val=42 len=1)
ble: deferred reply flushed (val=42 len=1 after=137 ms)
```
The `*** ble: deferred reply expired (>10 s)` line would be the smoking gun
that the 10 s wasn't enough — hasn't fired yet in the field.

Bumped `PL_FW_VERSION` → **0.0.36** (`Inc/config.h`).

## Box-persisted board-orientation calibration (v0.0.37+) — `0x13`/`0x14`, `calibration.c`

Historically each host app (iPhone / Android / Desktop) kept its own copy of
`nosePlusY`, `magOffsetMg`, `angleZeroRef` (+`angleZeroAtEpoch`), and
`headingBiasDeg` in local `UserDefaults` / `SharedPreferences` / `config.toml`.
Three of the four are physical facts of the *specific box* (which end is the
nose, hard-iron of *this* magnetometer, the pose the user chose as level for
*this* board), so "kalibrierst du auf iPhone, sieht Desktop's nächste Connect
es sofort auch" didn't work — every host had to re-tap.

**Fix**: the box holds the calibration in a 32-byte blob (`CAL.CFG` on the SD
root, parallel to `LOGMODE.CFG` + `GPSPWR.CFG`) and exposes it over two new
FileSync opcodes:

- **`0x13 FSYNC_OP_CAL_GET`** — no payload. Reply is the whole 32-byte blob
  in one FileData notify (via the deferred-reply path — the blob is bigger
  than a status byte and the same ACL-congestion risk applies).
- **`0x14 FSYNC_OP_CAL_SET`** — 32-byte payload. **Per-field merge, not
  blob-replace**: only fields whose `valid_mask` bit is set in the incoming
  blob overwrite the stored ones; unset bits leave the corresponding field
  untouched. Reply is one status byte. Semantics let two hosts update
  disjoint fields without clobbering each other, and let a host push just
  `nosePlusY` (say) without needing to know the box's current `magOffsetMg`.

Blob layout, field encoding, semantics, and failure modes: **DESIGN.md**
→ *Box-persisted calibration (`CAL_GET` / `CAL_SET`)*. Do not re-derive.

Public API in `Src/calibration.c` / `Inc/calibration.h` is deliberately
opaque — no per-field getters/setters, just `Calibration_Init` (called from
`main()` after `SDFat_Mount`), `Calibration_GetBlob(out[32])`, and
`Calibration_SetFromBlob(in, len)`. The 32-byte layout is the ONLY spec so
the reserved bytes can grow into future mask bits without churning `ble.c`.

Purely additive: legacy hosts (< v0.0.37) that never send `0x13`/`0x14`
keep their local UserDefaults / config.toml as before. Bumped
`PL_FW_VERSION` → **0.0.37** (`Inc/config.h`).

## GSV un-starved: C/N0 telemetry actually flows (v0.0.38+) — `gps.c`

> **Superseded by v0.0.41** (next section): the port is UBX-native now, GSV is
> silenced along with the rest of NMEA, and `cn0_max`/`sats_in_view` come from
> UBX NAV-SAT. The GSV path survives only as the RX-only fallback parser. The
> cfg-cfg-save discovery below (nothing persists on the M10) still stands and
> is the reason v0.0.41 reconfigures on every boot.

**The "cn0_max/sats_in_view are always 0" fix.** v0.0.19 added `parse_gsv`
for per-satellite signal strength, but `GPS_Init`'s noisy-NMEA disable list
(GLL/GSA/**GSV**/VTG, ACK'd every boot) turned GSV off before a single
sentence could arrive — the two CSV columns and SensorStream byte 45 were
dead on every configured module. Diagnosed while refuting the field theory
"die Software hat die u-blox Chips nachhaltig umkonfiguriert": the ERRLOG
shows `gps: cfg-cfg-save FAIL` on **every** boot (the M10 never ACKs the
legacy `UBX-CFG-CFG` persist), so nothing our firmware writes survives a
module power cycle — 12 of 15 field boots found the module back at factory
9600. All module config is effectively RAM-layer/per-session; the modules
are fine, they just never see sky signal (survey: C/N0 ≤ 27 dB-Hz, 0 sats
used → antenna/RF problem, not config).

Three coupled changes (`Src/gps.c`):

1. **`GPS_Init`**: disable list is now GLL/GSA/VTG only; GSV is set to
   **rate 10** ("emit every 10th nav epoch" = 1 burst/s at 10 Hz) via the
   same legacy CFG-MSG write. Full-rate GSV at 10 Hz would oversubscribe
   38400 baud — don't set it to 1. At the 9600/5 Hz fallback GSV stays 0
   (no headroom; also covers factory-fresh modules that boot with GSV at
   every epoch). New errlog line: `gps: cfg-msg gsv rate=10 ACK`.
2. **`parse_gga` hold-and-decay**: GSV bursts arrive 1/s but GGA commits
   10/s, so the commit only overwrites when a fresh burst was parsed
   (`g_gsv_burst`), holds the last value between bursts, and decays to 0
   ("no data") after `GSV_STALE_MS` (3 s) of GSV silence. Without this the
   CSV flickers 0 on 9 of 10 rows.
3. **`gps_survey_nmea(1)` restores the post-init state** (GSV at
   `g_gsv_rate`, GSA/VTG/GLL/ZDA off) instead of the old uniform rate-1,
   which re-enabled everything at every epoch after a GPS-Debug survey and
   oversubscribed the UART until the next module power cycle.
   `gps_cfg_valset_bool` now writes its 1-byte value raw (still 0/1 for the
   protocol bools; rate values for the U1 MSGOUT keys).

Bumped `PL_FW_VERSION` → **0.0.38** (`Inc/config.h`).

## GPS config bypass via `GPSRAW.CFG` (v0.0.47+) — `gps.c`

**The issue-#10 A/B discriminator.** v0.0.46 (1-Hz-Akquisition) still showed
`rmc=0` in Peter's first test, which leaves two suspects that his experiments
can't separate — the FW's *configuration* vs. the *running system's* EMI
(bootloader mode has neither). The bypass separates them: if a file named
**`GPSRAW.CFG`** exists on the SD root (content ignored — create it via the
box's USB-MSC drive), `GPS_Init` sends **zero bytes** to the module (no wake
pulse, no baud raise, no CFG-* writes) and leaves it 100 % in the factory
state — 9600 baud, NMEA, 1 Hz, all constellations — exactly the state in
which the module fixes in Peter's Versuch 1+2, while SD logging, BLE and
sensors all run normally. The NMEA fallback parser (GGA/RMC/GSV @9600) keeps
logging fixes if they come. The nav-rate switcher is disarmed; the local
UART stays at 9600 (`g_locked_baud`).

Reading the result: **LED blinks with bypass → the config kills acquisition**
(next: bisect — drop CFG-SIGNAL, or CFG-RST after config). **LED stays dark →
EMI/supply of the running system** (hardware/layout track, not config).

Errlog signature (the `***` is deliberate — a bypass boot must never read as
a clean production boot; delete the file to restore normal config):
```
*** gps: GPSRAW.CFG — factory config untouched (A/B bypass) ***
gps: ready @9600 baud, factory NMEA 1Hz, RX-only bypass
```
Bumped `PL_FW_VERSION` → **0.0.47**.

**v0.0.50: `BLEOFF.CFG` BLE-off bypass (issue #10).** Same file-switch
mechanic for the other radio: `BLEOFF.CFG` on the SD root → `main()` skips
`BLE_Init`/`BLE_Tick` and holds the BlueNRG-LP in hardware reset (PD4 low,
RSTN active-low) — 2.4 GHz provably silent, box invisible over Bluetooth
(no FileSync/FOTA; undo = delete file via USB, or DFU). Logging/GPS/
sensors/USB unaffected. Combine with `GPSRAW.CFG` for "factory GPS + no
BLE". Errlog: `*** ble: BLEOFF.CFG — BLE disabled, chip held in reset
(A/B test) ***`. Context: Peter suspects the BLE TX as the RF jammer; TX
power was never raised (0 dBm since import, `ble.c` adv config).

**v0.0.49: bypass RX-IRQ fix — run the A/B test on ≥0.0.49.** v0.0.47/48
never armed the UART RX interrupt on the bypass path (normal boots get it as
a side-effect of `listen_traffic`/`ubx_wait_ack`, which the bypass skips), so
the ring stayed dead: `gps_diag bytes=0`, and even a fixing module would have
logged `rmc=0` — the ERRLOG/CSV half of the A/B test read false-negative
(the LED half was always valid). The bypass branch now aborts + re-arms
`HAL_UART_Receive_IT` right after `gps_uart_reopen(9600)`, mirroring
`listen_traffic`'s robust-arm pattern. Found by the v0.0.49 correctness
audit (adversarial gps.c review), not in the field.

## GPS adaptive nav rate: acquire @1 Hz, track @10 Hz (v0.0.46+) — `gps.c`

**The "no fix ever with the FW running" fix (issue #10).** Peter isolated it
cleanly: same module, same board — factory config (bootloader mode / PC
power) fixes and blinks the uputronics TP LED; FW v0.0.45 running → LED dark
forever. His ERRLOG proves the module is healthy and the config lands 100 %
(all ACKs, `lines_bad=1`, `ubx_drop=0`) yet **26'583 NAV-PVT epochs over
44.5 min with `rmc=0`** — the module navigates at 10 Hz but never *acquires*.
Root cause hypothesis: **cold acquisition at 10 Hz is far less sensitive than
at the 1 Hz the u-blox TTFF specs assume** (the factory default acquires at
1 Hz; Peter's u-center session proves *tracking* at 10 Hz is fine: 15 sats /
3D fix).

v0.0.46 therefore splits the nav rate:

- `GPS_Init` step 6 sets **1 Hz** (`GPS_ACQ_RATE_MS`, config.h) — the
  acquisition rate — and arms the switcher only when the configured UBX path
  is up (`230400 + pvt ACK`); the NMEA-fallback/slow line stays 1 Hz, switcher
  off.
- `gps_rate_manage()` (end of `GPS_Tick`) raises the module to `GPS_RATE_HZ`
  (10 Hz) once a valid fix is fresh (3 s window), and drops back to 1 Hz for
  re-acquisition when the fix has been gone `GPS_REACQ_DOWNSHIFT_MS` (60 s
  hysteresis).
- The switch VALSET is **fire-and-forget from the superloop** (F-ARCH: no
  blocking ACK waits outside `GPS_Init`). Its ACK-ACK is recognized by the
  UBX router (`g_ack_valset` — new `0x05 0x01` branch in `gps_rx_byte`),
  resent up to 3× on a 500 ms timeout, then backed off 60 s with a `***`
  errlog entry (Peter's rule). Known benign race: a survey-toggle VALSET ACK
  can be misattributed to a pending switch; the next fix/loss edge re-syncs.
- ERRLOG: boot banner now says `GPS 1-10Hz`; ready line
  `gps: ready @230400 baud, nav=1Hz acq (10Hz after first fix), UBX NAV-PVT`;
  switches log `gps: nav rate → 10Hz (fix acquired) ACK` / `→ 1Hz (re-acq) ACK`.
- Logger/CSV: unaffected — GPS rows exist only on valid fixes, and the first
  fix immediately triggers the 10 Hz upshift.

Don't revert step 6 to a flat 10 Hz — that resurrects the no-fix field bug.
If 1-Hz acquisition alone turns out not to be enough, the next lever is the
`GPSRAW.CFG` config-bypass A/B test (see issue #10) to separate config from
running-system EMI. Bumped `PL_FW_VERSION` → **0.0.46**.

## GPS: no baud scan — assume 9600, then confirm (v0.0.43+) — `gps.c`

**Peter, 2026-07-13:** *"Es braucht keinen Baudratentest am Anfang. Nach dem
Booten ist es immer 9600 Baud."* Correct for a **cold boot** — the hall switch
cuts the whole rail, the module returns to its factory 9600 NMEA default, and
nothing we write ever persists (the M10 never ACKs the legacy CFG-CFG save).

**But there is exactly one case where it is NOT 9600, and it matters:** no MCU
pin switches the GPS supply (grep — there is none; GPS-off is software-only,
`UBX-RXM-PMREQ` backup). On an **MCU-only reset** — FOTA's `SWAP_BANK` reboot,
an IWDG reset, a software reset — the module keeps its power **and this
session's RAM config**, so it is still at **230400**. FOTA is the primary
update path, so assuming 9600 unconditionally would leave the GPS **dead after
every wireless firmware update** until someone power-cycles with the magnet.

**Resolution — confirm, don't scan.** `GPS_Init` drops the listen-first baud
scan entirely (saves ~1.5 s on every cold boot). It opens the UART at 9600,
wake-pulses, sends `CFG-UART1-BAUDRATE=230400` **blind**, re-opens locally at
230400, and then *confirms*. That single confirm covers both worlds for free:

- module **@9600** → it accepted the command → the **ACK** arrives at 230400.
- module **@230400** → it saw our 9600 bytes as framing garbage and dropped them
  (UBX checksums make a false positive impossible), but the **MON-VER poll** we
  then send at 230400 gets a reply → `gps: already @230400 (MCU-only reset)`.

Only if **neither** confirms does a real probe run over **115200 → 38400 →
9600**, and the raise is retried from wherever the module actually is →
`*** gps: baud fallback — module @… ***`. **115200 must stay in that list**
until no v0.0.44 box is left in the field: v0.0.44 is the one release that used
115200 as the session baud, so a box updated by FOTA *from* v0.0.44 reboots
MCU-only with its module still at 115200 — without the candidate, the GPS would
be dead after exactly the update that installs the current firmware.

`gps_uart_reopen(baud)` is the shared helper for every baud transition
(DeInit → init → `OVRDIS` → clear flags → empty ring).

**Don't "simplify" this to a bare 9600 assumption** — that silently breaks FOTA.

## GPS baud is 230400 — Peter's decision, don't flip it (v0.0.45+) — `config.h`

`GPS_UART_BAUDRATE` = **230400**. Peter, 2026-07-14: *"Peter will Baudrate von
exakt 230400."* That is **authoritative** and settles a real conflict in the
record — do **not** "correct" it back to 115200:

- His **u-center screenshot** (2026-07-13) of the config that measured 15 sats /
  3D fix / C/N0 41 dB-Hz shows `CFG-UART1-BAUDRATE = **115200**`, layer RAM.
- His **written instruction** the same day said **230400**, and he has now
  confirmed 230400 is what he wants.

v0.0.44 briefly shipped 115200 to match the screenshot; **v0.0.45 reverts to
230400** per his decision. Both rates work — configured traffic is ~1.4 KB/s
(NAV-PVT@10 Hz + NAV-SAT@1 Hz), i.e. ~6 % of a 230400 line, ~12 % of a 115200
one. This was never a correctness question.

Everything *else* in his u-center config already matched the firmware exactly:
`CFG-RATE-MEAS 100`, `CFG-RATE-NAV 1`, `CFG-PM-OPERATEMODE 0`,
`MSGOUT-UBX_NAV_SAT_UART1 10`, `MSGOUT-UBX_NAV_PVT_UART1 1`, and every
`CFG-SIGNAL` key ID.

**The baud is not why the box wasn't fixing.** At 230400 the link already
measured `lines_good=1111 / lines_bad=1` with NAV-PVT steady at 10 Hz — and
`rmc` was still 0. See issue #10.

## GPS config order: baud FIRST (v0.0.42+) — `gps.c`

**Peter, 2026-07-13:** *"Die Baudrate als ersten Schritt auf 230400 erhöhen. Mit
9600 kollabiert die gesamte Kommunikation wenn mit den Folgebefehlen die
Datenrate erhöht wird."* v0.0.41 (below) put the `CFG-UART1-BAUDRATE` switch at
**step 6**, so constellations / power-mode / UBX-output all ran at **9600 while
the module was still streaming its factory NMEA**. 9600 = 960 B/s and default
NMEA eats most of it; step 4 then piles NAV-PVT-every-epoch + NAV-SAT on top.
The module's TX backs up, its ACKs miss the 300 ms `ubx_send_retry` budget, and
every later command reports FAIL — the "collapse" Peter saw.

**v0.0.42 order** (`GPS_Init`, `Src/gps.c`): baud-detect → **1. baud → 230400**
→ 2. CFG-SIGNAL → 3. CFG-PM-OPERATEMODE → 4. UBX out (INPROT/OUTPROT + MSGOUT
PVT=1/SAT=10) → 5. NMEA off → 6. CFG-RATE 10 Hz. The baud VALSET is now the
*only* command sent on the slow line — a single ~36 B frame, which even a
saturated 9600 link delivers. Everything after it has 23 kB/s of headroom (~6 % utilisation at PVT@10Hz + SAT@1Hz), so
**no command can starve its own ACK**. The ACK-arrives-at-the-new-baud handling
(re-init local UART right after the command drains, MON-VER poll as fallback
confirmation, revert to `locked_baud` if unconfirmed) is unchanged — it just
moved. Same reasoning now governs the fallback: if the baud switch fails, the
nav rate stays at **1 Hz** (was 5 Hz — ~52 % of a 960 B/s line for NAV-PVT alone
before NAV-SAT). New errlog line: `gps: cfg baud 230400 ACK (step 1)`.

**Don't move the baud switch back down the list** — that's the whole bug.

## GPS UBX-native at 230400, GPS+Galileo only (v0.0.41+) — `gps.c`

> **Command order superseded by v0.0.42** (above): the baud switch moved from
> last to first. The recipe's *content* (GPS+Galileo only, full power, UBX
> NAV-PVT/NAV-SAT, NMEA off, 230400, RAM-layer-only, ACK-verified) is unchanged.

**Peter's per-boot config recipe (2026-07-13), verified in u-center 2 over a
USB-UART adapter**: GPS + Galileo only (BeiDou/GLONASS off), full power, UBX
NAV-PVT every nav epoch + NAV-SAT every 10th, baud **230400** set as the
**last** command, everything **RAM layer only** (re-sent every
boot — the M10 never persists, see v0.0.38), every command ACK-verified, and
**every un-ACK'd command writes a `***` errlog entry** (which also latches the
red LED). Result at his window with ~40 % sky view: 15 sats used, 3D fix,
C/N0 up to 41 dB-Hz — where the 4-constellation NMEA default never fixed.

Authoritative sequence + rationale: **DESIGN.md → "GPS module configuration
(v0.0.41+)"**. Key implementation facts:

- **Boot flow (`GPS_Init`)**: listen-first baud detect (9600 cold-boot →
  230400 MCU-only reset → 38400 pre-v0.0.41 leftover; UBX syncs count as
  traffic, not just NMEA newlines) → **CFG-UART1-BAUDRATE=230400 first**
  (v0.0.42 — see the section above; local UART re-inited immediately because
  the ACK arrives at the NEW baud, MON-VER poll as fallback confirmation) →
  CFG-SIGNAL (9 keys, one VALSET, then 500 ms settle) → CFG-PM-OPERATEMODE=0
  → INPROT/OUTPROT-UBX=1 + MSGOUT PVT=1/SAT=10 (UBX *out* is OFF in the M10
  factory default — without this key PVT never flows) → NMEA MSGOUT all 0
  (only if the UBX step ACK'd, so a partial failure can't mute the module) →
  CFG-RATE 100 ms/1.
- **Parser** (`gps_rx_byte`): protocol router — UBX frames (Fletcher-checked,
  `parse_nav_pvt`/`parse_nav_sat`) + the old NMEA line assembler as fallback.
  `fix_q` is mapped to the NMEA-GGA scale (1 = valid fix) for host compat;
  `hdop` carries pDOP×0.01 in UBX mode; `utc` from PVT (validDate+Time gated).
- **Diag counters keep their exact 8-key `gps_diag:` format** (the desktop
  errlog_check parses it strictly): checksum-valid UBX frames count into
  `lines_good`, corrupt into `lines_bad`, PVT epochs into `gga`, valid fixes
  into `rmc`. The 30 s one-shot marker still contains the literal token
  `GPS NO NMEA` (errlog_check matches it).
- **Bridge (`0x0D`)** no longer flips protocols — the port is UBX-native; it
  only throttles our periodic PVT 10 Hz → 1 Hz while a survey runs.
- **RX ring 512 B → 2 KB** (`GPS_RX_RING_SIZE`, config.h); configured UBX
  traffic is ~1.4 KB/s, the 230400 line is mostly idle (headroom for Peter's
  future 18 Hz idea — 25 Hz "chip tuning" is out: irreversible per Peter).
- **NMEA-fallback mode** (box→module TX dead, nothing ACKs): module stays at
  factory 9600/1 Hz NMEA defaults, GGA/RMC/GSV parsing still logs — RX-only
  operation keeps working as before.

Bumped `PL_FW_VERSION` → **0.0.41** (`Inc/config.h`).

## Known WIP edges (per the PR description)

These are scheduled for cleanup in Phase 8; don't be surprised by them
and don't "fix" them as drive-bys — they're load-bearing shortcuts
during bring-up:

- READ/DELETE accept a **bare opcode → default test file** shortcut
  because nRF Connect Mobile caps its hex-write field at 16 chars. Real
  clients send the full `opcode + 8.3-name + offset`. The shortcut goes
  away in Phase 8.
- Notify chunks are 20 B (safe under default ATT MTU). MTU negotiation
  + larger chunks is Phase 8. DESIGN.md §3 describes the chunked-mode
  fallback that must keep working.
- LIST returns macOS junk filenames (`.Spotlight-V100` etc.) unfiltered.
  Cosmetic, Phase 8.

## Phase status (as of PR open)

Phases 1-6 + the re-advertise-on-disconnect fix are **done and
hardware-verified**. Phase 7 (BatteryStatus characteristic) is built but
not yet on the branch. Phase 8 is hardening + 1000× power-cycle / BLE
reconnect soak. The branch tip when the PR was opened
(`fc4d396474fb6896ca138fec77b4128d2282af1d`) is the last
hardware-verified commit — when integrating against PumpLogger, prefer
that SHA as the stable base.

## Backlog (separate task, not in this branch)

- GPS per-satellite signal strength (C/N0) + satellite count — parsing
  **done in v0.0.19** (`parse_gsv` in `gps.c`), but **starved until
  v0.0.38**: the v0.0.19 note claimed "the module already emits GSV at
  its NMEA default, so no config write is needed" — wrong, because
  `GPS_Init`'s own noisy-NMEA disable list included GSV (ACK'd every
  boot), so `cn0_max`/`sats_in_view` read a permanent 0 in the field.
  Fixed in v0.0.38 (see below). `parse_gga` commits the strongest C/N0
  (`cn0_max`) and tracked-sat count (`sats_in_view`) into `PL_GpsFix`,
  the Gps CSV (`cn0_max,sats_in_view` columns), and SensorStream
  byte 45. Parsing is RX-only, so it works even when the box→module
  command line is dead (unlike the UBX GPS Debug survey, which needs the
  box→module TX path). A full per-satellite C/N0 table is still future
  work.
