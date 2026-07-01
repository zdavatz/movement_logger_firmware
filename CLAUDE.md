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
  STC3115 fuel gauge on I²C4. GPS u-blox MAX-M10S on UART4 @ 38400 baud,
  captured by DMA-circular into a 512 B ring.
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

- GPS per-satellite signal strength (C/N0) + satellite count — **done in
  v0.0.19** (`parse_gsv` in `gps.c`). The module already emits GSV at its
  NMEA default, so no config write is needed (module→box RX direction);
  `parse_gga` commits the per-epoch strongest C/N0 (`cn0_max`) and
  tracked-sat count (`sats_in_view`) into `PL_GpsFix`, the Gps CSV
  (`cn0_max,sats_in_view` columns), and SensorStream byte 45. RX-only, so
  it works even when the box→module command line is dead (unlike the UBX
  GPS Debug survey, which needs the box→module TX path). A full
  per-satellite C/N0 table is still future work.
