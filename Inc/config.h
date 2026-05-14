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
#define PL_FW_NAME             "PumpLogger"
#define PL_FW_VERSION          "0.1.0"
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

/* GPS / UART4 --------------------------------------------------------------*/
#ifndef GPS_RATE_HZ
#define GPS_RATE_HZ            10
#endif
#define GPS_UART_BAUDRATE      38400U
#define GPS_DMA_RING_SIZE      512U          /* DMA-circular buffer */

/* Cooperative scheduler ----------------------------------------------------*/
#define PL_TICK_HZ             1000U         /* SysTick @ 1 ms */

/* Task cadences (in ticks). Tasks run when `(tick_count % CADENCE) == 0`.   */
#define PL_CADENCE_LED_BLINK   500U          /* heartbeat LED toggle = 0.5 Hz */
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
