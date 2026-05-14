# movement_logger_firmware

[![Latest release](https://img.shields.io/github/v/release/zdavatz/movement_logger_firmware)](https://github.com/zdavatz/movement_logger_firmware/releases/latest)

Bare-metal firmware for the **STEVAL-MKBOXPRO Rev_C** (SensorTile.box PRO) running as the pumpfoil **MovementLogger** session recorder — logs IMU/baro/mag/fuel/GPS to SD continuously and exposes a BLE FileSync + SensorStream interface to the host GUI ([movement_logger_desktop](https://github.com/zdavatz/movement_logger_desktop)) and Android client ([movement_logger_android](https://github.com/zdavatz/movement_logger_android)).

## Read first

- **[REQUIREMENTS.md](REQUIREMENTS.md)** — *what* the firmware must do (sensor rates, modes, BLE protocol obligations, watchdog behavior).
- **[DESIGN.md](DESIGN.md)** — *how* (GATT UUIDs, FileSync wire protocol, SensorStream byte layout, HCI command sequence, on-SD CSV schemas).
- **[CLAUDE.md](CLAUDE.md)** — invariants, hard rules, and gotchas accumulated during bring-up. Worth reading before making non-trivial changes.

## Hardware target

STEVAL-MKBOXPRO Rev_C only. MCU: STM32U585AIIxQ (Cortex-M33 @ 160 MHz, HSI→PLL). BLE: BlueNRG-LP on SPI1, polled (EXTI11 intentionally unused — see CLAUDE.md). Sensors: LSM6DSV16X / LIS2MDL / LPS22DF / STTS22H / STC3115 / u-blox MAX-M10S GPS.

## Build

Requires `arm-gnu-toolchain` (e.g. via Homebrew `arm-none-eabi-gcc` or [the upstream tarball](https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads) at `$HOME/.software/arm-gnu-toolchain/`).

```sh
cd STM32CubeIDE
make clean && make            # outputs: build/PumpLogger.elf, build/firmware.bin
# Override toolchain prefix dir:
# make TOOLCHAIN=/opt/homebrew/bin
```

Each `make` bumps `STM32CubeIDE/.build_counter` and bakes the value into the ErrLog boot banner via `-DPL_BUILD_NUM=N`, so it's obvious which binary is on the box.

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
