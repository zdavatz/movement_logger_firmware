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

**One material requirements-vs-design divergence to be aware of:**
REQUIREMENTS.md still describes two **mutually exclusive** modes
(`LOG` / `STREAM`) with `STREAM_START` / `STREAM_STOP` opcodes.
DESIGN.md v0.2 supersedes this: SD logging is **always on**, and live
streaming is a side-effect of a client subscribing to the SensorStream
CCCD (no mode switch, no opcodes). The PR description marks this
"no-mode decision" explicit. Follow DESIGN.md.

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
  and the WWDG early-warning ISR (last-gasp errlog + beep before reset).
  Anything else — SPI/UART/I²C/SDMMC/EXTI/DMA-complete — runs polled.
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

- GPS per-satellite signal strength (C/N0) + satellite count. Needs GSV
  sentences re-enabled in the u-blox config and a parser added in
  `gps.c`.
