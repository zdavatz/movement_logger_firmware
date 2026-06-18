# movement_logger_firmware

[![Latest release](https://img.shields.io/github/v/release/zdavatz/movement_logger_firmware)](https://github.com/zdavatz/movement_logger_firmware/releases/latest)

Bare-metal firmware for the **STEVAL-MKBOXPRO Rev_C** (SensorTile.box PRO) running as the pumpfoil **MovementLogger** session recorder — logs IMU/baro/mag/fuel/GPS to SD continuously and exposes a BLE FileSync + SensorStream interface to the host GUI ([movement_logger_desktop](https://github.com/zdavatz/movement_logger_desktop)) and Android client ([movement_logger_android](https://github.com/zdavatz/movement_logger_android)).

## Read first

- **[REQUIREMENTS.md](REQUIREMENTS.md)** — *what* the firmware must do (sensor rates, modes, BLE protocol obligations, watchdog behavior).
- **[DESIGN.md](DESIGN.md)** — *how* (GATT UUIDs, FileSync wire protocol, SensorStream byte layout, HCI command sequence, on-SD CSV schemas).
- **[CLAUDE.md](CLAUDE.md)** — invariants, hard rules, and gotchas accumulated during bring-up. Worth reading before making non-trivial changes.
- **[GPS_SOLDERING.md](GPS_SOLDERING.md)** ([PDF](pdf/GPS_SOLDERING.pdf)) — step-by-step manual for hand-soldering the u-blox MAX-M10S GPS to the box's JP2 programming connector (UART4). Read before touching the iron.

## Supported hardware

**STMicroelectronics STEVAL-MKBOXPRO (SensorTile.box PRO), Rev_C only.**

- Buy: [st.com — STEVAL-MKBOXPRO](https://www.st.com/en/evaluation-tools/steval-mkboxpro.html) (also available from Mouser, DigiKey, Farnell and other ST distributors)
- Rev_A/B and STWIN.box are **not** supported — they stay on the old firmware tree.

MCU: STM32U585AIIxQ (Cortex-M33 @ 160 MHz, HSI→PLL). BLE: BlueNRG-LP on SPI1, polled (EXTI11 intentionally unused — see CLAUDE.md). Sensors: LSM6DSV16X / LIS2MDL / LPS22DF / STTS22H / STC3115 / [u-blox MAX-M10S](https://www.u-blox.com/en/product/max-m10s-module) GPS receiver. Power: Li-Po + wireless charging; Hall sensor + magnet wired as a hardware supply-rail interrupter (no MCU sleep modes).

The GPS receiver is **not on the STEVAL-MKBOXPRO** — it is a separate [u-blox MAX-M10S](https://www.u-blox.com/en/product/max-m10s-module) module wired to UART4 @ 38400 baud (e.g. a [MAX-M10S breakout from SparkFun](https://www.sparkfun.com/products/18037) or any MAX-M10S carrier). UART4 is brought out on the **JP2 programming connector** (the box has no STMod+): pin 13 = RX (MCU PA1), pin 14 = TX (MCU PA0), pin 7/11 = GND. **3.3 V logic only** — GPS power comes from the JP4 3 V rail, never from JP2. Full procedure in **[GPS_SOLDERING.md](GPS_SOLDERING.md)**; see also CLAUDE.md / DESIGN.md for the u-blox config sequence.

## Build

Requires `arm-gnu-toolchain` (e.g. via Homebrew `arm-none-eabi-gcc` or [the upstream tarball](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) at `$HOME/.software/arm-gnu-toolchain/`).

```sh
cd STM32CubeIDE
make clean && make            # outputs: build/PumpLogger.elf, build/firmware.bin
# Override toolchain prefix dir:
# make TOOLCHAIN=/opt/homebrew/bin
```

Each `make` bumps `STM32CubeIDE/.build_counter` and bakes the value into the ErrLog boot banner via `-DPL_BUILD_NUM=N`, so it's obvious which binary is on the box.

## Read the SD card over USB-C (Phase 9, v0.0.9+)

Power the box on (magnet attached) and plug it into a host over USB-C — the microSD enumerates as a standard USB mass-storage drive. No button, no DFU dance:

```sh
lsblk            # find the new device (e.g. /dev/sda1, label PUMPTSUERI)
sudo mount /dev/sda1 /mnt
```

While the host has the drive mounted, the box pauses its own SD writes (two filesystem drivers can't share the card). Eject / unplug and the logger automatically resumes the previous mode (AUTO → new session, MANUAL → wait for `START_LOG`). BLE FileSync remains the wireless path; they coexist but are mutually exclusive at runtime.

See REQUIREMENTS.md F-USB-1..6, DESIGN.md §14, and [issue #5](https://github.com/zdavatz/movement_logger_firmware/issues/5) for the full design.

## Host time-sync (`SET_TIME 0x08`, v0.0.10+)

The box has no RTC — the CSV `ms` column is a free-running `HAL_GetTick()` counter that starts at 0 on cold boot. So on every BLE connect the host (iPhone / Android / desktop) pushes its current wall-clock millis via `SET_TIME 0x08 <epoch_ms:u64-LE>`, and the firmware appends a marker line into the **open** `SensNNN.csv` and `GpsNNN.csv`:

```
# SYNC epoch_ms=1750000000000 tick_ms=42137
```

Pairing the host epoch with the box's free-running `ms` counter lets the replay tools map every logged row to absolute wall-clock with **zero drift and without needing a GPS fix** — the marker shares the host's clock domain with the replay video's `creation_time`, so the data panels line up exactly. The marker is a `#`-comment line every CSV parser skips as a data row; it's a best-effort no-op when no session is open. See DESIGN.md opcode table (0x08).

## Flash via DFU

Hold the user button while plugging USB-C to enter STM32 DFU mode:

```sh
dfu-util -d 0483:df11 -a 0 \
  -s 0x08000000:mass-erase:force \
  -D build/firmware.bin
```

`:mass-erase:force` is **mandatory** — there is a known bank-swap bug in the predecessor firmware that leaves stale code at the alternate bank.

## Releases

Push a `vX.Y.Z` tag to trigger `.github/workflows/release.yml`, which builds the firmware with `gcc-arm-none-eabi` (apt) on Ubuntu and attaches `firmware-vX.Y.Z.bin` (+ SHA-256) to an auto-created GitHub Release. End users flash that binary via the `dfu-util` command above.

Current release: see the badge at the top. Latest is the most reliable build.
