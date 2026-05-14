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
11. [Watchdog details](#11-watchdog-details)
12. [Boot sequence](#12-boot-sequence)

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
| `0x04` | `STOP_LOG` | none | Optional: flush + close the current session and rotate to the next `SensNNN+1.csv`. **Not required for READ** — it exists only as a file-rotation convenience. Logging resumes immediately with the new session. |
| any other | reserved | — | Box replies `0xE3 BAD_REQUEST` on FileData. |

**No mode switch.** There is deliberately no STREAM_START / STREAM_STOP
and no LOG-vs-STREAM mode. SD logging is *always on* — there is no state
the box can be left in where it silently stops recording. Live streaming
(Section 3) is a side effect of a client subscribing to the SensorStream
CCCD, runs concurrently with logging, and stops on its own when the
client unsubscribes or disconnects. The earlier mutually-exclusive-mode
design was inherited from the ThreadX firmware (where BLE and SDMMC could
not coexist); the polling architecture here has no such constraint, so
the mode — and its lost-session failure mode — is simply removed.

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

A 1-byte READ payload is **never** a status byte by coincidence — the
host distinguishes by checking whether the byte fits the file's
expected continuation (any value 0–255 is valid file content). The
"end of stream" marker is recognized by `0x00` arriving *after* the
expected total length has been received. (A 1-byte file containing
`0x00` and a 0-byte file with terminator are indistinguishable —
acceptable corner case.)

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
 42       1   gps_fix_q               0=none 1=GPS …    u-blox NMEA GGA
 43       1   gps_nsat                # satellites
 44       1   flags                   bit field         see below
 45       1   reserved                0x00
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

- FAT32. FatFs library (not FileX). Single-volume mount.
- Long filenames disabled (8.3 only) — smaller code, no Unicode tables.
- Cluster cache 4 KB.

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
ms,utc,lat,lon,alt_m,speed_kmh,course_deg,fix_q,nsat,hdop
```

Empty (header only) until first valid fix. Rows match the structure used
by stbox-viz so the existing GPS-overlay code keeps working.

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
battery_poll           1000             ~200 µs (I²C, once/sec)
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
| GPS DMA ring | static array | 512 B | ~3 s of GPS bytes at 38400 baud. |
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

## 11. Watchdog details

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

## 12. Boot sequence

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
220 ms   GPS UART4 + DMA-circular start, send UBX-CFG-RATE config
260 ms   BLE chip reset pulse (10 ms low + 150 ms wait)
430 ms   HCI command sequence (Section 5) — ~50 commands at ~5 ms each
700 ms   Advertising enabled — STBoxFs visible
710 ms   Open first SensNNN.csv, GpsNNN.csv, BatNNN.csv, Error_Log_*
720 ms   Main loop starts
```

Slack: ~280 ms before the 1-second target. Plenty of room for HCI
retries if the chip is slow.

---

## 13. Decisions left for implementation phase

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

**Next step:** Peter reviews. When DESIGN.md v0.1 is locked, we move to
**Phase 2: skeleton + boot-beep** — and from there, every phase ends
with a flashable binary on the actual box.
