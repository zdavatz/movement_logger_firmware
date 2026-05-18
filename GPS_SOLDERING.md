---
title: "Soldering the u-blox MAX-M10S GPS to the STEVAL-MKBOXPRO"
subtitle: "MovementLogger firmware — UART4 wiring on the JP2 programming connector"
date: "2026-05-18"
---

# What you are doing

The MovementLogger firmware reads GPS over **UART4** on MCU pins
**PA0 (TX)** and **PA1 (RX)**, at **38400 baud, 8 data bits, no parity,
1 stop bit (8N1)**, 3.3 V logic. The receiver is a separate
**u-blox MAX-M10S** module — it is *not* on the STEVAL-MKBOXPRO. This
sheet covers wiring that module to the box by hand.

> **The STEVAL-MKBOXPRO has no STMod+ connector.** On the
> SensorTile.box PRO core board, UART4 (PA0/PA1) is brought out only on
> **JP2, the programming connector** — the same 14-pin header the
> STLINK-V3 plugs into. That is where you solder.

Sources: ST *STEVAL-MKBOXPRO schematic* Rev 3 (Figure 3,
*Sensortilebox\_pro\_MCU*), ST *UM3133* Rev 7 §1.8.6 / §2.3, and the
firmware (`Src/gps.c`, `Inc/config.h`).

# Connector

| Item | Value |
|---|---|
| Reference | **JP2** ("Prog Connector" / programming connector) |
| Part | Samtec **FTSH-107-01-L-D** |
| Layout | **2 × 7 = 14 pins**, **1.27 mm (0.05") pitch**, shrouded |
| Also exposes | SWD/JTAG + NRST (leave those alone) |

This is fine-pitch surface work. You need: a fine conical tip, flux,
magnification, and thin wire — **30 AWG** enamelled / wire-wrap
(≤ 0.5 mm). Tweezers and a steady bench.

## JP2 pinout (from schematic Figure 3)

```
            JP2  (pin 1 = square pad / silk triangle)
          odd column              even column
        +--------------------------------------------+
   1 -> | 1V8 rail (ref)     |  2  -> (no connect)   |
   3 -> | (do not use)       |  4  -> SWDIO / JTMS   |
   5 -> | (do not use)       |  6  -> SWCLK / JTCK   |
   7 -> | GND                |  8  -> JTDO / SWO     |
   9 -> | (do not use)       | 10  -> JTDI           |
  11 -> | GND                | 12  -> NRST           |
  13 -> | UART4_RX  (PA1)    | 14  -> UART4_TX (PA0) |
        +--------------------------------------------+
```

- **Pin 13 = UART4\_RX = MCU PA1** — the MCU's *input*.
- **Pin 14 = UART4\_TX = MCU PA0** — the MCU's *output*.
- **Pin 7 and pin 11 = GND** (the only two GND pins confirmed by the
  schematic's junction dots — use one of these for the GPS ground).
- **Pin 1 ties to the on-board 1.8 V rail** (the JTAG pull-up
  reference; R52/R54/R55 are 10 k to 1V8). It is **1.8 V, not 3.3 V**,
  and it is **not** a power source — never connect GPS VCC here. Take
  GPS 3.3 V from JP4 (see next section).
- Pins **4, 6, 8, 10, 12** are the debug bus. A solder bridge from
  pin 14 to pin 12, or onto SWDIO/SWCLK, can brick debug access — keep
  these clean.
- Pins **3, 5, 9** carry no signal you need — leave them unconnected.

Verify pin 1's position against the square pad / silkscreen triangle on
*your* board before counting, and **meter-confirm** every pin
(continuity to MCU PA0/PA1 and to battery −) before soldering. Treat
this table as a guide, not gospel — connector orientation can differ
between board builds.

# Wiring (note the cross-over)

UART is always TX-to-RX. The GPS and the MCU each have their own TX/RX,
so the data lines cross:

| GPS module pin | → | JP2 pin | Net (MCU) |
|---|---|---|---|
| **TX** (GPS out) | → | **pin 13** | UART4\_RX (PA1) |
| **RX** (GPS in)  | → | **pin 14** | UART4\_TX (PA0) |
| **GND** | → | JP2 **pin 7 or pin 11** (meter-verified) | GND |
| **VCC 3.3 V** | → | **board 3 V rail — see below** | — |

## GPS power: 3.3 V from the board rail

Per UM3133 §1.8.6 the **JP4 extension-board connector** (Multicomp
MC-SVT1-S07-G, 7-pin) carries the board supply and **GND**, with a
**domain switch selectable 1.8 V / 3 V**. To power the GPS from the
board rail:

1. Set the extension-board supply domain to **3 V** (not 1.8 V).
2. With the board powered, **measure with a multimeter** that the pin
   reads **3.25–3.40 V** before you connect the GPS VCC to it.
3. Take **GND** from the same JP4 connector (or a JP2 GND pin) — the
   GPS ground and the box ground must be common.

The GPS will be powered and unpowered together with the box. There is
**no graceful shutdown** — the Hall-sensor magnet cuts the whole supply
rail at once (firmware design F-PWR-5). This is expected; the firmware
and on-SD format tolerate a sudden cut.

# Safety — read before the iron is hot

- **3.3 V logic only.** PA0/PA1 are **not 5 V tolerant**. A 5 V GPS
  breakout's TX into PA1 will destroy the MCU. If your module is a 5 V
  board, level-shift or use a 3.3 V-native MAX-M10S carrier.
- **Never bridge to the SWD pins** (4/6/8/10/12).
- Keep each pad heated **< 2 s**; FTSH-107 pads lift if cooked.
- **Strain-relieve** the wires (Kapton tape + a dab of hot glue) close
  to JP2. A tugged wire rips the pad off the PCB.

# Procedure

1. **Power down completely**: power switch S1 off, USB unplugged,
   LiPo disconnected.
2. **Locate JP2** — the 14-pin fine-pitch header by the MCU (the
   STLINK programming connector). Identify pin 1.
3. **Continuity-check before soldering** (meter, board off):
   - JP2 pin 13 ↔ MCU **PA1**
   - JP2 pin 14 ↔ MCU **PA0**
   - candidate GND pin ↔ battery **(−)**
   Confirm, don't assume — connector orientation differs between
   board builds.
4. **Pre-tin** the three JP2 pads and the wire ends with flux.
5. **Solder the data wires**: pin 13 ← GPS TX, pin 14 ← GPS RX.
   Then **GND**.
6. **GPS VCC last**: with the board powered, confirm **3.3 V** at the
   JP4 3 V pin, power down again, solder the VCC wire.
7. **Inspect under magnification** for bridges — especially pin 14 to
   pin 12 and onto the SWD pins.
8. **Strain-relieve** all four wires.

# Bring-up test

1. Insert the SD card, take the box **outdoors with clear sky**, power
   on.
2. The firmware sends the u-blox UBX config on boot, then expects
   10 Hz NMEA. First fix is typically reached within **~30–60 s**
   cold.
3. Confirm a fix by either:
   - `GpsNNN.csv` on the SD card gaining rows (one per fix), or
   - the BLE SensorStream `gps_valid` flag going set, or
   - (antenna-test build, `make GPS_ANTENNA_BEEP=1`) the buzzer beeping
     N times every 10 s with N = satellites.

## No fix after several minutes outdoors?

In order of likelihood:

1. **Swap the two data wires** (pin 13 ↔ pin 14). TX/RX reversed is the
   single most common wiring mistake and swapping is harmless to try.
2. Re-check **GND** continuity (cold/again) and that GPS VCC really is
   3.3 V *at the module* under load.
3. Confirm the module is a 3.3 V part and actually powered (most
   MAX-M10S carriers have a power LED).
4. Give it a longer cold-start with an unobstructed sky view; indoors
   it will never fix.
