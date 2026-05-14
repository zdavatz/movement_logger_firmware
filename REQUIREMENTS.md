# Requirements — SensorTile.box PRO firmware rewrite

**Status:** **LOCKED v0.3** · 2026-05-11
**Working name:** STBox PumpLogger (final name TBD)
**Audience:** Peter (review), Claude (implementation), zeno (review)

## Contents

1. [Project goal](#1-project-goal)
2. [Operating modes](#2-operating-modes)
3. [Functional requirements](#3-functional-requirements)
4. [Non-functional requirements](#4-non-functional-requirements)
5. [Hardware constraints](#5-hardware-constraints)
6. [Architectural sketch](#6-architectural-sketch)
7. [Out of scope](#7-out-of-scope)
8. [Design decisions](#8-design-decisions)
9. [Glossary](#9-glossary)
10. [Implementation phases](#10-implementation-phases)

---

## 1. Project goal

A reliable, bare-metal firmware for the SensorTile.box PRO (Rev_C, in a
waterproof enclosure) that:

1. Logs motion + environmental + GPS data to SD card at fixed rates.
2. Background-syncs that data to a paired host (Mac/Android) over BLE
   whenever the host is in range — so by the time a session ends, the
   transfer is mostly done.
3. On host request, **switches** to a live-stream mode where the same
   sensor data is pushed over BLE in real time (calibration / debugging).
4. Survives connection drops, BLE flakiness, sensor glitches, and
   hard power-cycles without losing data or wedging.

This is a from-scratch replacement for the current ST BLE Manager +
STM32WB07_06 + ThreadX + FileX stack which has proven too opaque and
bug-prone to debug in our use case.

---

## 2. Operating modes

The firmware has two **mutually exclusive** operating modes. Default at
power-up is `LOG`. The host switches modes by writing an opcode on the
FileCmd characteristic.

| Mode | Sensors | SD writes | BLE FileSync | BLE Live stream | BLE Battery status |
|---|---|---|---|---|---|
| `LOG` | read at native rates | yes — to `Sens/Gps/Bat*.csv` | active (LIST/READ/DELETE) | off | active |
| `STREAM` | read at native rates | none (active CSV files closed) | paused | active — packed sample per notify | active |

Mode transitions:
- Power-on → `LOG`.
- Host writes `STREAM_START` opcode → switch to `STREAM`.
- Host writes `STREAM_STOP`, or disconnects, or stays disconnected for >5 s → revert to `LOG`.

---

## 3. Functional requirements

### 3.1 Power and startup

| ID | Requirement |
|---|---|
| F-PWR-1 | **Hardware power switch via Hall sensor.** The Hall sensor (or its latching circuit) physically interrupts the supply rail to the MCU. Magnet present = supply broken = MCU fully off. Magnet absent = supply applied = MCU cold-boots. No sleep modes in firmware — only "running" or "no power". |
| F-PWR-2 | Magnet is mounted inside the transport case. Behavior: box in case → MCU off, wireless charger tops up battery. Box out of case → MCU cold-boots, logging starts. |
| F-PWR-3 | Logging begins immediately on cold boot. No user interaction needed. Target boot-to-active ≤ 1 s. |
| F-PWR-4 | Acoustic feedback: one short beep right after `main()` enters its loop, so the user hears the box wake up. |
| F-PWR-5 | **Graceful shutdown is not possible** — power dies the moment the magnet returns. Therefore: (a) the on-SD format tolerates sudden truncation (CSV is fine — last line may be partial, post-processor discards it); (b) flush cadence bounds worst-case data loss to ≤ 1 s of sensor samples (see F-LOG-5). |
| F-PWR-6 | **Wireless charging** runs entirely independent of the MCU: the receiver IC / coil drive the charging path from the case's inductive transmitter directly into the Li-Po. No firmware participation. No USB-C cable involvement at the user level. |
| F-PWR-7 | Hall-switch debounce lives in **hardware** (RC filter / Schmitt trigger). Firmware sees only "powered" or "unpowered". |

### 3.2 Sensor logging to SD card (LOG mode)

| ID | Requirement |
|---|---|
| F-LOG-1 | While in LOG mode, continuously log to SD card. |
| F-LOG-2 | Sensors and rates: |
|         | • LSM6DSV16X accelerometer @ 100 Hz (XYZ, ±4 g, 0.122 mg/LSB) |
|         | • LSM6DSV16X gyroscope @ 100 Hz (XYZ, ±500 dps, 17.5 mdps/LSB) |
|         | • LIS2MDL magnetometer @ 100 Hz (XYZ, ±50 gauss) |
|         | • LPS22DF barometer @ 25 Hz (pressure hPa, temperature °C) |
|         | • GPS (u-blox MAX-M10S) @ 10 Hz (lat, lon, alt, speed, course, time) |
|         | • STC3115 fuel gauge @ 1 Hz (voltage, SOC, current) |
| F-LOG-3 | Timestamps use a free-running ms counter starting at 0 on cold boot. Wall-clock time is folded in when GPS produces a valid `$GNRMC` sentence — the FAT timestamp on each open file is updated then. |
| F-LOG-4 | File naming: `SensNNN.csv`, `GpsNNN.csv`, `BatNNN.csv`. Counter increments per session; first available number is used. One session = one of each file (no rollover). |
| F-LOG-5 | Periodic flush so an ungraceful power-off still leaves readable files. Target ≤ 1 s of data loss on hard cut. |
| F-LOG-6 | When an SD write returns an error, log to the error log and continue with the next sample. Never wedge the main loop on SD trouble. |

### 3.3 BLE — background sync of logged files (LOG mode)

The host downloads files in the background whenever it's connected.
While in STREAM mode this is paused; it resumes on STREAM_STOP / reconnect.

| ID | Requirement |
|---|---|
| F-SYNC-1 | Box advertises continuously while powered. Advertise name `STBoxFs` (v1; will rename in v2). Connectable, static PIN `123456`. |
| F-SYNC-2 | When the host connects in LOG mode, the box serves file operations **in parallel with active sensor logging to SD** — no need to stop logging to download. |
| F-SYNC-3 | Sync is **resumable across drops**: a dropped connection mid-transfer doesn't restart the next session from byte 0. |
| F-SYNC-4 | Sync is **incremental**: as the active session keeps appending to `SensNNN.csv`, the box pushes only the newly appended bytes, not the whole file again. |
| F-SYNC-5 | Sync spans multiple short host-encounters: e.g. user walks past phone, partial transfer happens, user leaves, transfer resumes next time. |
| F-SYNC-6 | Host can request files ad-hoc (LIST + READ) on top of the automatic background sync. |
| F-SYNC-7 | Host can delete files (DELETE opcode). GUI offers per-file "Delete from box" and bulk "Delete all synced files". |
| F-SYNC-8 | After a BLE drop the box simply re-enters advertising. Reconnect requires no user interaction beyond the host trying again. |
| F-SYNC-9 | **Resumable READ**: the READ opcode takes an offset. `READ <name> <offset>` streams bytes from `offset` to current EOF. `offset = 0` = entire file. |
| F-SYNC-10 | **Sync-state lives only on the host**, never on the box (per F-ARCH-1). Each connect: host LISTs, compares each file's size to its own per-file recorded high-water mark, and READs from offset where the box is ahead. |
| F-SYNC-11 | The error log file is part of the FileSync set and downloadable like any other. Excluded from automatic background sync; host pulls it on demand. |

### 3.4 BLE — live sensor stream (STREAM mode)

| ID | Requirement |
|---|---|
| F-LIVE-1 | Host writes opcode `0x10 STREAM_START` on FileCmd → box switches to STREAM mode. `0x11 STREAM_STOP` returns to LOG mode. |
| F-LIVE-2 | While in STREAM, sensor reads land in notifications on a dedicated `SensorStream` characteristic (UUID TBD). One packed binary sample per notify: acc XYZ + gyro XYZ + mag XYZ + pressure + temperature + (when valid) GPS lat/lon. Format finalized in design phase. |
| F-LIVE-3 | Stream rate **0.5 Hz** by default (one packet every 2 s). Sufficient for calibration / debugging use cases. One packet carries the full sensor snapshot (acc, gyro, mag, baro, GPS). Higher rates are a future feature if needed. |
| F-LIVE-4 | While in STREAM, the active `Sens/Gps/Bat*.csv` files are closed cleanly — no dangling half-written session. Re-entering LOG mode opens fresh `SensNNN+1.csv` etc. |
| F-LIVE-5 | Connection drop in STREAM mode → auto-revert to LOG mode after a 5-second grace window, so the box never sits in stream mode with no host listening, missing real ride data. |

### 3.5 BLE — battery status (both modes)

| ID | Requirement |
|---|---|
| F-BAT-1 | `BatteryStatus` GATT characteristic (UUID TBD, read + notify) exposes: voltage (mV), SOC (0.1 %), current (signed 100 µA). |
| F-BAT-2 | Box notifies the host once per minute while connected so the GUI's battery indicator stays current without polling. |
| F-BAT-3 | Host can read on demand (e.g. right after connect, to populate the GUI immediately). |
| F-BAT-4 | Available in both LOG and STREAM modes — battery info isn't data-mode-coupled. |

### 3.6 Audio feedback (buzzer)

| ID | Requirement |
|---|---|
| F-SND-1 | One short beep right after `main()` enters the loop. |
| F-SND-2 | Three short beeps the first time GPS acquires a valid fix. |
| F-SND-3 | Distinct beep pattern when a watchdog reset fires (see F-WDG). |
| F-SND-4 | Low-battery warning: long-pulse beep every 30 s while SOC < 10 %. The user notices "put me back in the case" without staring at the app. |

### 3.7 Watchdogs

| ID | Requirement |
|---|---|
| F-WDG-1 | The main loop kicks the hardware IWDG every iteration. If the loop hangs for > 2 s, IWDG resets the chip. |
| F-WDG-2 | **Sensor-plausibility watchdog**: at least one sensor must produce a different reading from the last poll within a 5 s window. Even 1-LSB jitter on a noise-floor reading counts as "alive". Watchdog fires only if every sensor on every axis returns identical bytes for 5 full seconds (signals "data pipeline frozen mid-stack"). |
| F-WDG-3 | On any watchdog reset: distinctive beep pattern + a log line with the cause in the error log. |
| F-WDG-4 | Reset reason from `RCC->CSR` is decoded and written at every boot: POR / BOR / IWDG / SOFTWARE / PIN / OBL / LPWR. Same decoder as the current firmware. |

### 3.8 Error log

| ID | Requirement |
|---|---|
| F-ERR-1 | Human-readable error log file, named after the compile-date as today: `Error_Log_Pump_Tsueri_DD.MM.YYYY.log`. |
| F-ERR-2 | Entries: boot markers, sensor init success/failure, watchdog events, SD write errors, BLE state transitions (connect/disconnect/mode-change), GPS-fix acquisition. |
| F-ERR-3 | Each entry: a ms-since-boot timestamp + (once GPS gives time) a wall-clock UTC timestamp. |
| F-ERR-4 | Downloadable via BLE FileSync (see F-SYNC-11). |

### 3.9 Architectural principles

| ID | Requirement |
|---|---|
| F-ARCH-1 | **Stateless** where possible. Each module owns its own minimal state, communicates via clear inputs/outputs. No big state machines other modules depend on. |
| F-ARCH-2 | On any module-internal error: module resets itself rather than corrupting state. (e.g. BLE wedges → restart BLE stack, leave logger alone.) |
| F-ARCH-3 | The logger never depends on BLE for forward progress. Box logs perfectly with no host ever connecting. |
| F-ARCH-4 | The BLE FileSync never blocks the logger. If the SD bus is busy, FileSync waits and retries next loop iteration. |
| F-ARCH-5 | No hidden globals across modules. Shared state (e.g. current session number) is accessed via explicit functions, not raw globals. |
| F-ARCH-6 | **Polling-only design.** No application-level ISRs. All peripherals (SPI, UART, I²C, SDMMC, EXTI, DMA-complete) are accessed via polled HAL calls from the main loop. Eliminates the nested-interrupt + race-condition bug class that wrecked the current firmware. |
| F-ARCH-7 | Permitted exceptions to F-ARCH-6, deliberately minimal: |
|          | • **SysTick @ 1 ms** as the system time base. Kept because HAL needs it and the ISR is one counter-increment. |
|          | • **IWDG** as the hardware loop-alive watchdog — produces no ISR, just resets the chip on miss. Belt-and-suspenders backup if WWDG ISR itself hangs. |
|          | • **WWDG early-warning ISR**: fires ~50 ms before WWDG would reset. ISR writes a last-gasp error-log line ("watchdog: about to reset, last_task=…"), plays the watchdog-beep pattern, then either returns (letting reset happen naturally) or calls `NVIC_SystemReset()` directly. Lets us preserve the diagnostic on the SD card across the reset. |
| F-ARCH-8 | No dynamic memory allocation after init. Heap is off-limits at runtime (see NF-SIZE-3). |

---

## 4. Non-functional requirements

### 4.1 Performance

| ID | Requirement |
|---|---|
| NF-PERF-1 | Sustained 100 Hz sensor logging with zero dropped samples across a 1-hour session. |
| NF-PERF-2 | 10 Hz GPS logging — every fix the module produces lands in `Gps*.csv`. |
| NF-PERF-3 | BLE FileSync throughput target: ≥ 5 KB/s sustained. |
| NF-PERF-4 | Mode switch LOG ↔ STREAM completes in ≤ 500 ms (GUI button must feel responsive). |
| NF-PERF-5 | Boot to "logging active + advertising" ≤ 1 s (matching F-PWR-3). |

### 4.2 Size / footprint

| ID | Requirement |
|---|---|
| NF-SIZE-1 | Binary fits comfortably in a single 1 MB flash bank, target ≤ 80 KB. Leaves room for OTA later. |
| NF-SIZE-2 | Static RAM ≤ 32 KB. |
| NF-SIZE-3 | No dynamic allocation after init. All buffers static / stack. |

### 4.3 Reliability

| ID | Requirement |
|---|---|
| NF-REL-1 | Survives 1000 consecutive power cycles without SD-card corruption. |
| NF-REL-2 | Survives 1000 BLE connect/disconnect cycles without leaking memory, descriptors, or wedging. |
| NF-REL-3 | The BlueNRG-LP is reset cleanly on each session start so leftover bond state can't produce "worked once, fails next time" symptoms. |
| NF-REL-4 | Worst-case data loss on ungraceful power-off: ≤ 1 s of sensor data. GPS / battery rows may lose up to one polling interval. |

### 4.4 Observability

| ID | Requirement |
|---|---|
| NF-OBS-1 | Every module logs at least its init success/failure + unexpected-state-transitions to the error log. |
| NF-OBS-2 | Periodic heartbeat line in the error log once per minute: free SD-card space, BLE connection state, last sensor sample tick, GPS fix state, SOC %. |
| NF-OBS-3 | Reset cause is logged with a human-readable name (POR / BOR / IWDG / SOFTWARE / PIN / OBL / LPWR) — same decoder as today. |

---

## 5. Hardware constraints

| ID | Constraint |
|---|---|
| HW-1 | Target board: **STEVAL-MKBOXPRO Rev_C** only. Other variants (Rev_A/B, STWIN.box) stay on the old firmware tree. |
| HW-2 | MCU: STM32U585AIIxQ, single-core Cortex-M33 @ 160 MHz, dual-bank 2 MB flash, 786 KB SRAM. |
| HW-3 | BLE chip: BlueNRG-LP / STM32WB07_06 over SPI1. EXTI11 IRQ pin present on the board but firmware uses pure polling (F-ARCH-6) — the EXTI line is left unarmed. |
| HW-4 | Sensors: LSM6DSV16X (SPI), LIS2MDL (I²C2), LPS22DF (I²C2), STTS22H (I²C2). |
| HW-5 | GPS: u-blox MAX-M10S on UART4 @ 38400 baud. Bytes captured by **DMA in circular mode** into a 512 B ring buffer; main loop reads at leisure. Auto-configured at boot (UBX-CFG-RATE, CFG-MSG, CFG-CFG). |
| HW-6 | Fuel gauge: STC3115 on I²C4. |
| HW-7 | SD card: SDMMC1 peripheral, FAT32. |
| HW-8 | Power: Li-Po battery, **wireless / inductive charging** via the transport case. On-board USB-C is sealed inside the case and used only for factory flashing / DFU repair. |
| HW-9 | Power switch: **Hall sensor + magnet** wired as a hardware supply-rail interrupter (not a signal to the MCU). Hall-sensor part + circuit are a hardware-team task, no firmware visibility required. |
| HW-10 | Wireless charging receiver IC + coil. Charging runs entirely outside firmware (MCU is off in that state) — no status pin to MCU is required. |
| HW-11 | Buzzer: existing piezo via a TIM channel. |
| HW-12 | Clock source: HSI (internal). HSE crystal is unreliable on battery due to the 3.3 V board mod. PLL targets 160 MHz sysclk. |
| HW-13 | Waterproof enclosure: no user-accessible buttons, no SD-card swap, no USB connector. Magnet (in transport case) is the only user interface. |

---

## 6. Architectural sketch

Informative — refined during the design phase.

```
┌──────────────────────────────────────────────────────────────┐
│ main() — bare-metal cooperative scheduler, 1 ms tick         │
│                                                              │
│  ┌──────────────────┐    ┌────────────────────────────────┐  │
│  │ logger task      │    │ ble task                       │  │
│  │  read sensors    │    │  - hci command sequence        │  │
│  │  parse GPS DMA   │    │  - filesync state machine      │  │
│  │  poll battery    │    │  - live-stream emit (STREAM)   │  │
│  │  write SD (LOG)  │    │  - polled SPI to BlueNRG-LP    │  │
│  └──────────────────┘    └────────────────────────────────┘  │
│                                                              │
│  ┌──────────────────┐    ┌────────────────────────────────┐  │
│  │ watchdog task    │    │ buzzer task                    │  │
│  │  kick IWDG       │    │  beep patterns                 │  │
│  │  plausibility    │    │  low-battery warning           │  │
│  │  heartbeat log   │    │                                │  │
│  └──────────────────┘    └────────────────────────────────┘  │
│                                                              │
└──────────────────────────────────────────────────────────────┘
        │                              │
        ▼                              ▼
     SD card                  BlueNRG-LP over SPI1
```

Key properties:

- **Cooperative scheduling, no RTOS.** Each task is a function called
  every main-loop tick. Long operations stay short enough not to starve
  others. No context switches, no priority inversion.
- **Pure polling everywhere** (F-ARCH-6). The only ISRs running are
  SysTick (system time base) and optionally WWDG early-warning.
- **One source of truth per resource** (SD card, SPI bus, sensor
  handles). No competing accessors from "the ISR" vs. "the thread".

---

## 7. Out of scope

| ID | Item | Note |
|---|---|---|
| OOS-1 | Microphone WAV recording | Was unreliable on the modified board, dropped in the current firmware already. |
| OOS-2 | NFC pairing | Static-PIN BLE is sufficient. |
| OOS-3 | SD-card firmware update | DFU via USB-C is the supported update path. (SD update has the known bank-swap bug.) |
| OOS-4 | OTA via BLE | Planned for v2, not v1. |
| OOS-5 | Multiple board revisions | Rev_C only. Rev_A/B + STWIN.box stay on the old firmware tree. |
| OOS-6 | PnP-Like sensor manifest, BlueST-SDK V2 | Not used by the GUI. |
| OOS-7 | Derived-key secure pairing | Static PIN `123456` is sufficient for our threat model (physical possession = trust). |
| OOS-8 | "Per-feature" GATT services à la BLE Manager | One custom service with three characteristics (FileCmd, FileData, SensorStream) plus BatteryStatus is enough. |
| OOS-9 | Microphone, audio in/out (other than the buzzer) | Outside our use case. |
| OOS-10 | Live-stream + FileSync running simultaneously | Mutually exclusive modes (see Section 2) — switching is allowed and even desirable for simplicity. |

---

## 8. Design decisions

These are the open questions resolved during the requirements pass on
2026-05-11. Listed here for traceability; future readers don't need to
revisit the trade-offs.

| ID | Decision | Why |
|---|---|---|
| D-1 | No MCU sleep modes. Hall switch is a hard power-rail interrupter. | Simpler firmware: no Stop 2 / Standby logic, no wake-from-sleep paths, no firmware-side magnet debounce. ~1 s cold-boot is acceptable. |
| D-2 | Keep BLE advertise name `STBoxFs` for v1; consider rename in v2. | Matches current MovementLogger GUI. Max 8 chars limit. |
| D-3 | Hall-sensor part choice + wiring is a hardware-team concern, firmware sees only "powered / unpowered". | Hardware-only matter — Peter tracks it on his side. |
| D-4 | Sync-state lives on the host, not on the box. Box exposes `READ <name> <offset>`; host tracks its own high-water marks per file. | Matches stateless principle (F-ARCH-1). No on-box sync-state file to corrupt or get out of sync. |
| D-5 | Live-stream samples are one packed binary blob per notify on a dedicated characteristic. | Faster than per-sensor characteristics; GUI knows the byte layout. |
| D-6 | Single host. Box rejects further connect attempts while one host is connected. | Avoids multi-host synchronization complexity. |
| D-7 | Sensor-plausibility watchdog accepts 1-LSB jitter as "alive". Fires only on identical raw bytes for 5 s. | Catches "pipeline frozen somewhere between sensor and main loop" without false positives from a stationary box. |
| D-8 | No file rollover. One session = one each of `SensNNN.csv` / `GpsNNN.csv` / `BatNNN.csv`. | SD wear isn't a concern at our ~6 KB/s rate. Incremental sync handles long files fine via `READ <name> <offset>`. |
| D-9 | Low-battery handling: beep warning at SOC < 10 %, no firmware-side cutoff. | Hardware power switch makes cutoff impossible anyway. Beep nudges user to put the box back in the case. |
| D-10 | Wireless-charging detection is irrelevant. The MCU is off while charging. | Battery state via STC3115 (F-BAT) is sufficient for GUI feedback. |
| D-11 | GPS UART4 captured via DMA-circular into a ring buffer, parsed in thread context. | Zero ISR for UART RX, no tight per-byte timing, no overrun risk. Matches F-ARCH-6. |
| D-12 | Firmware update is DFU via USB-C, requires unscrewing the case. | Acceptable for v1; BLE-OTA pushed to v2. |
| D-13 | Magnetometer readings while in the case (saturated by the switch magnet) are irrelevant because logging is off. | Confirmed: no scenario expects valid mag data inside the case. |
| D-14 | Live-stream and SD-logging are mutually exclusive modes, not parallel. | Massive design simplification — no contention for SD/SPI bandwidth, no priority decisions. |
| D-15 | Battery status is its own GATT characteristic, available in both modes. | Decoupled from data-mode so the GUI battery indicator works regardless. |

---

## 9. Glossary

- **Box**: the SensorTile.box PRO running this firmware.
- **Host**: the Mac / Android device running the MovementLogger GUI.
- **Session**: one cold-boot to power-off cycle. One session writes
  exactly one of each `SensNNN.csv` / `GpsNNN.csv` / `BatNNN.csv`.
- **LOG mode**: default mode. Sensors → SD card. BLE FileSync active.
- **STREAM mode**: requested by host. Sensors → BLE notifications. SD
  files closed.
- **FileSync**: the 4-opcode BLE protocol (LIST / READ-with-offset /
  DELETE / STREAM_START / STREAM_STOP) for managing on-box files +
  switching modes. Wire format finalized in the design phase.
- **Live stream**: dedicated GATT characteristic that emits one packed
  sensor sample per notification while in STREAM mode.

---

## 10. Implementation phases

Each phase ends with a flashable binary that demonstrates one capability.
Peter tests on the real box before we move to the next phase. Each phase
ends with a git commit + push.

| # | Phase | Deliverable | Effort |
|---|---|---|---|
| 1 | Design | `DESIGN.md` — concrete UUIDs, byte layouts, HCI command sequence, memory map, module boundaries. Peter reviews + locks. | 1-2 d |
| 2 | Skeleton + boot-beep | Flashable binary that cold-boots, beeps once, blinks LED. Bare-metal scheduler in place, empty module skeletons. | 2-3 d |
| 3 | Logger without BLE | Binary that logs sensors + GPS + battery to SD identically to today's firmware, but with no ThreadX / FileX / BSP middleware. From this point Peter has a usable logger for field data while phases 4-7 happen. | 2-3 d |
| 4 | BLE bringup | Box advertises as `STBoxFs`, Mac can connect with PIN. Custom HCI command sequence, polled SPI, four empty GATT characteristics. | 3-5 d |
| 5 | FileSync | LIST + READ-with-offset + DELETE. GUI can download files end-to-end, resume after a drop. | 1-2 d |
| 6 | Live stream + mode switch | STREAM_START / STREAM_STOP. GUI can flip the box to live-stream mode and back. Auto-revert on disconnect. | 1-2 d |
| 7 | Battery status char | Per-minute notifications + low-battery beep. GUI shows live battery. | 0.5 d |
| 8 | Hardening | 1000 × power-cycle test, 1000 × BLE-reconnect test, plausibility watchdog tuning, READMEs. | 2-3 d |

**Total ≈ 12-20 days of focused work.**

**Meta-rules:**

1. Every phase ends with commit + push (clean git history, easy revert).
2. Every phase is verified on the real box before the next starts.
3. If any phase exposes a requirement gap, work pauses to update this
   document before continuing.
4. Peter may say "stop, re-evaluate" at any point.
5. From Phase 3 onward, Peter has a usable logger for actual rides;
   the BLE work happens in parallel with real-world data collection.

---

**Next step:** Peter has reviewed. Requirements locked 2026-05-11.
Phase 1 (DESIGN.md draft) starts now.
