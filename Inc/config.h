/**
  ******************************************************************************
  * @file    config.h
  * @brief   Single-file constants for the PumpLogger firmware. Sensor rates,
  *          buffer sizes, GATT UUIDs, watchdog periods. Everything anyone
  *          might want to tune lives here.
  ******************************************************************************
  */

#ifndef PUMPLOGGER_CONFIG_H
#define PUMPLOGGER_CONFIG_H

/* Firmware identification --------------------------------------------------*/
#define PL_FW_NAME             "MovementLogger"
#define PL_FW_VERSION          "0.0.53"
#define PL_FW_BLE_NAME         "STBoxFs"   /* 7 chars, fits BLE name budget */

/* GPIO pin map (taken from the SensorTileBoxPro BSP; we use HAL directly) --*/
#define PL_LED_GREEN_PORT      GPIOF
#define PL_LED_GREEN_PIN       GPIO_PIN_6
#define PL_LED_RED_PORT        GPIOH
#define PL_LED_RED_PIN         GPIO_PIN_11
#define PL_BUZZER_PORT         GPIOE
#define PL_BUZZER_PIN          GPIO_PIN_13
#define PL_BUZZER_TIM_AF       GPIO_AF1_TIM1

/* LSM6DSV16X chip-select on SPI2 */
#define PL_LSM6_CS_PORT        GPIOI
#define PL_LSM6_CS_PIN         GPIO_PIN_5

/* SD-card enable + voltage-select (Rev_C wiring, lifted from BSP) */
#define PL_SD_ENABLE_PORT      GPIOH
#define PL_SD_ENABLE_PIN       GPIO_PIN_10
#define PL_SD_SEL_V_PORT       GPIOH
#define PL_SD_SEL_V_PIN        GPIO_PIN_8

/* Sensor 7-bit I²C addresses (from datasheets / ST drivers) ----------------*/
#define PL_LIS2MDL_I2C_ADDR    (0x1E << 1)   /* 0x3C 8-bit */
#define PL_LPS22DF_I2C_ADDR    (0x5D << 1)   /* SA0=1 → 0xBA 8-bit (LPS22DF on this board) */
#define PL_STTS22H_I2C_ADDR    (0x38 << 1)   /* 0x70 8-bit */
#define PL_STC3115_I2C_ADDR    (0x70 << 1)   /* 0xE0 8-bit */

/* Battery model for the STC3115 gauge (sensors_fuel.c). Values are ST's own
   STC3115_Battery_Conf.h for STEVAL-MKBOXPRO: HiMax 752535 pack behind the
   board's R83 shunt. Changing the pack means re-deriving CC_CNF/VM_CNF —
   see the formulas in sensors_fuel.c. */
#define PL_BATT_CAPACITY_MAH   480    /* nominal capacity */
#define PL_BATT_RINT_MOHM      160    /* pack internal impedance */
#define PL_BATT_RSENSE_MOHM    50     /* R83 current-sense shunt */

/* GPS / UART4 --------------------------------------------------------------*/
#ifndef GPS_RATE_HZ
#define GPS_RATE_HZ            10            /* tracking rate (after first fix) */
#endif
/* Adaptive nav rate (v0.0.46, issue #10): cold acquisition happens at 1 Hz —
   the rate the u-blox TTFF specs assume and the rate the factory default
   acquires at. 10-Hz-from-cold never produced a single fix in the field
   (Peter's ERRLOG: 26'583 PVT epochs in 44.5 min, rmc=0, while the same
   module at factory 1 Hz fixes on the same board). gps.c switches the module
   to GPS_RATE_HZ only once a valid fix is in hand, and drops back to 1 Hz
   for re-acquisition when the fix has been gone GPS_REACQ_DOWNSHIFT_MS. */
#define GPS_ACQ_RATE_MS         1000U        /* acquisition nav period (1 Hz) */
#define GPS_REACQ_DOWNSHIFT_MS 60000U        /* fix lost this long → back to 1 Hz */
/* Session baud, set per boot via UBX-CFG-VALSET (RAM only — the module always
   cold-starts at factory 9600; nothing we write persists across a module power
   cut).

   230400 — Peter's explicit decision (2026-07-14): "Peter will Baudrate von
   exakt 230400". This is AUTHORITATIVE and settles a genuine conflict in the
   record, so don't "correct" it back:
     - Peter's u-center screenshot (2026-07-13) shows CFG-UART1-BAUDRATE =
       115200 in the config that measured 15 sats / 3D fix / C/N0 41 dB-Hz.
     - Peter's written instruction the same day said 230400, and he has now
       confirmed 230400 is what he wants.
   v0.0.44 briefly shipped 115200 to match the screenshot; v0.0.45 reverts to
   230400 per his decision. Both numbers work — this is not a correctness
   question, and it is NOT related to the missing fix (at 230400 the link
   measured lines_good=1111 / lines_bad=1 with NAV-PVT steady at 10 Hz, and
   rmc was still 0 — see issue #10).

   Headroom: configured traffic is ~1.4 KB/s (NAV-PVT@10Hz + NAV-SAT@1Hz)
   against 23 KB/s of line → ~6 % utilisation. */
#define GPS_UART_BAUDRATE      230400U
#define GPS_RX_RING_SIZE       2048U         /* byte-IRQ RX ring (gps.c) */
/* Periodic `gps_rf:` errlog line (v0.0.52): Peter's assembly metrics
   (fixType, numSV used, top-6 GPS+Galileo C/N0) + the MON-RF EMI set
   (noisePerMS, agcCnt, jamInd, jammingState, antStatus) — one line per
   interval while the UBX path is up. ~110 B/line → ~160 KB/day at 60 s,
   small next to the 5 s gps_diag cadence. */
#define GPS_RF_LOG_INTERVAL_MS 60000U

/* Cooperative scheduler ----------------------------------------------------*/
#define PL_TICK_HZ             1000U         /* SysTick @ 1 ms */

/* Task cadences (in ticks). Tasks run when `(tick_count % CADENCE) == 0`.   */
#define PL_CADENCE_LED_PHASE   125U          /* LED pattern phase tick — 8 phases × 125 ms = 1 s cycle. Pattern table in main.c encodes single (no-fix) / double (weak) / triple (good) GPS-quality flash. */
#define PL_CADENCE_SENSOR      10U           /* 100 Hz sensor sample */
#define PL_CADENCE_BARO        40U           /* 25 Hz baro/temp poll */
#define PL_CADENCE_GPS_POLL    50U           /* drain GPS DMA every 50 ms */
#define PL_CADENCE_BATTERY     1000U         /* 1 Hz STC3115 + battery row */
#define PL_CADENCE_FLUSH       1000U         /* SD flush every 1 s */
#define PL_CADENCE_HEARTBEAT   60000U        /* error-log heartbeat */
#define PL_CADENCE_WATCHDOG    1U            /* kick IWDG every tick */
#define PL_CADENCE_PLAUSIBLE   1000U         /* check sensor freshness 1 Hz */

/* Sample-buffer sizes -----------------------------------------------------*/
#define PL_SD_BLOCK_SIZE       512U
#define PL_FAT_CACHE_CLUSTERS  1U            /* tiny — append-only writer */

/* Watchdog -----------------------------------------------------------------*/
#define PL_IWDG_PERIOD_MS      2000U         /* hard reset after ≤2 s hang */
#define PL_PLAUSIBLE_WINDOW_MS 5000U

#endif /* PUMPLOGGER_CONFIG_H */
