/**
  ******************************************************************************
  * @file    main.c
  * @brief   PumpLogger entry point. Bare-metal cooperative scheduler.
  *
  *          Phase 3 deliverable: cold-boot → clocks/LED/buzzer → SD mount →
  *          error log → sensor + GPS init → IWDG → main loop calling
  *          watchdog/logger/gps tasks. Per F-ARCH-6 no application ISRs;
  *          SysTick + IWDG only.
  ******************************************************************************
  */

#include "main.h"
#include "sched.h"
#include "buzzer.h"
#include "watchdog.h"
#include "sd_fatfs.h"
#include "errlog.h"
#include "sensors_imu.h"
#include "sensors_mag.h"
#include "sensors_baro.h"
#include "sensors_fuel.h"
#include "gps.h"
#include "logger.h"
#include "ble.h"

/* Captured *before* HAL clears anything so the error log can decode the
   reset reason in ErrLog_Init(). */
uint32_t BootResetCsr;

static void gpio_init_leds(void);
static void beep_pattern(uint16_t freq, uint8_t reps, uint16_t ms_on, uint16_t ms_off);

int main(void)
{
  /* HAL_MspInit (defined below) lands SMPS + VDDIO2 + UCPD-dead-battery
     setup INSIDE HAL_Init() so the chip is ready for scale 1 / 160 MHz
     before any other HAL call. */
  HAL_Init();

  /* Snapshot reset cause first, then clear flags so the NEXT reset's bits
     come back clean. */
  BootResetCsr = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();

  SystemClock_Config();

  /* Red LED on while we set up peripherals; green LED idle off. */
  gpio_init_leds();
  HAL_GPIO_WritePin(PL_LED_RED_PORT,   PL_LED_RED_PIN,   GPIO_PIN_SET);
  HAL_GPIO_WritePin(PL_LED_GREEN_PORT, PL_LED_GREEN_PIN, GPIO_PIN_RESET);

  /* Buzzer last — depends on TIM1 + GPIOE clocks. The two-tone "boot done"
     beep is removed: logger has been past Phase-2 bring-up for weeks, and
     the antenna-test build needs the buzzer silent at boot so it can use
     it for the sat-count signal pattern instead. */
  Buzzer_Init();

  /* Hand-off: red off, green on. From here on the scheduler drives
     everything via SysTick. */
  HAL_GPIO_WritePin(PL_LED_RED_PORT,   PL_LED_RED_PIN,   GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PL_LED_GREEN_PORT, PL_LED_GREEN_PIN, GPIO_PIN_SET);

  /* Sensor-bus enable. PI0 is a board-level mux that gates the SPI/I²C
     traces to the LSM6DSV16X, LIS2MDL and LPS22DF — must be driven LOW
     before any sensor init or they NAK every transaction. PI5 is the
     LSM6DSV16X SPI chip-select, kept idle HIGH. Both lifted from the
     SDDataLogFileX::InitMemsSensors sequence. */
  __HAL_RCC_GPIOI_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOI, GPIO_PIN_0, GPIO_PIN_RESET);
  {
    GPIO_InitTypeDef g = {0};
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Pull  = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    g.Pin   = GPIO_PIN_5;
    HAL_GPIO_Init(GPIOI, &g);
    g.Pin   = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOI, &g);
  }

  pl_fx_status_t sd_rc = SDFat_Mount();
  if (sd_rc != PL_FX_OK) {
    /* No SD card = no logging = useless box. Treat as fatal: red LED
       on solid + green off + long beep, then halt. The operator must
       NOT be able to start a pumpfoil session and discover hours later
       that the SD slot was empty — same failure mode as forgetting
       the film in a camera. Hard halt, no second chance. */
    Error_Handler(__FILE__, __LINE__);
  } else {
    Buzzer_Beep(3000U, 60U);
    ErrLog_Init();
    if (FUEL_Init() != 0) {
      ErrLog_Write("fuel: init FAIL");
      beep_pattern(2000, 4, 60, 80);
    } else {
      ErrLog_Write("fuel: ok (STC3115)");
    }
    if (IMU_Init() != 0) {
      ErrLog_Write("imu: init FAIL");
      beep_pattern(2000, 1, 60, 80);
    } else {
      ErrLog_Write("imu: ok (LSM6DSV16X)");
    }
    if (MAG_Init() != 0) {
      ErrLog_Write("mag: init FAIL");
      beep_pattern(2000, 2, 60, 80);
    } else {
      ErrLog_Write("mag: ok (LIS2MDL)");
    }
    if (BARO_Init() != 0) {
      ErrLog_Write("baro: init FAIL");
      beep_pattern(2000, 3, 60, 80);
    } else {
      ErrLog_Write("baro: ok (LPS22DF)");
    }
    if (GPS_Init() != 0) {
      ErrLog_Write("gps: init FAIL");
      beep_pattern(2000, 5, 60, 80);
    } else {
      ErrLog_Write("gps: ok (MAX-M10S UART4)");
    }
    if (Logger_Init() != 0) {
      ErrLog_Write("logger: init FAIL");
      beep_pattern(800, 6, 100, 100);
    } else {
      ErrLog_Write("logger: session opened");
    }
    if (BLE_Init() != 0) {
      ErrLog_Write("ble: init FAIL");
      beep_pattern(2000, 7, 60, 80);
    }
    ErrLog_Flush();
  }

  Watchdog_Init();

  for (;;)
  {
    Watchdog_Tick();
    Logger_Tick();
    GPS_Tick();
    BLE_Tick();

    /* Green-LED pattern based on GPS fix quality. Phase advances every
       125 ms; eight 125 ms slots = one second per cycle. Pattern table:
         NONE: 1 1 1 1 0 0 0 0  → slow blink (500 ms on / 500 ms off)
         WEAK: 1 0 1 0 0 0 0 0  → double flash (4-6 sats)
         GOOD: 1 0 1 0 1 0 0 0  → triple flash (≥7 sats)
       The slow-blink case is identical in feel to the pre-Build-#49
       LED — so when GPS isn't fixed yet (boot, indoors, etc.) the
       operator sees the familiar "logger alive" heartbeat. Adding
       a 2nd / 3rd flash means GPS is actually producing valid fixes,
       letting the operator drop the dedicated GPS module LEDs to save
       power without losing GPS-status feedback. */
    if (sched_due(PL_SCHED_LED, PL_CADENCE_LED_PHASE)) {
      static const uint8_t LED_PATTERN[3][8] = {
        { 1, 1, 1, 1, 0, 0, 0, 0 },   /* PL_GPS_QUALITY_NONE */
        { 1, 0, 1, 0, 0, 0, 0, 0 },   /* PL_GPS_QUALITY_WEAK */
        { 1, 0, 1, 0, 1, 0, 0, 0 },   /* PL_GPS_QUALITY_GOOD */
      };
      static uint8_t led_phase = 0;
      pl_gps_quality_t q = GPS_LastFixQuality();
      if (q > PL_GPS_QUALITY_GOOD) q = PL_GPS_QUALITY_GOOD;
      HAL_GPIO_WritePin(PL_LED_GREEN_PORT, PL_LED_GREEN_PIN,
                        LED_PATTERN[q][led_phase] ? GPIO_PIN_SET : GPIO_PIN_RESET);
      led_phase = (uint8_t)((led_phase + 1u) & 0x07u);
    }

    /* Red LED solid-on as soon as ErrLog latches a '***' warning. The
       operator notices something needs attention without needing to
       pull the SD card mid-session. Once latched stays latched until
       reset — single transition per boot, no LED flicker. Green keeps
       blinking so the operator can tell "warning + still running"
       apart from the fatal-Error_Handler case ("warning + green dark"). */
    if (ErrLog_WarningLatched()) {
      HAL_GPIO_WritePin(PL_LED_RED_PORT, PL_LED_RED_PIN, GPIO_PIN_SET);
    }

    sched_wait_next_tick();
  }
}

/* ----------------------------------------------------------------------- */

static void beep_pattern(uint16_t freq, uint8_t reps, uint16_t ms_on, uint16_t ms_off)
{
  for (uint8_t i = 0; i < reps; i++) {
    Buzzer_Beep(freq, ms_on);
    HAL_Delay(ms_off);
  }
}

static void gpio_init_leds(void)
{
  __HAL_RCC_GPIOF_CLK_ENABLE();   /* green LED on PF6 */
  __HAL_RCC_GPIOH_CLK_ENABLE();   /* red LED on PH11 */

  GPIO_InitTypeDef gpio = {0};
  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  gpio.Pin = PL_LED_GREEN_PIN;
  HAL_GPIO_Init(PL_LED_GREEN_PORT, &gpio);

  gpio.Pin = PL_LED_RED_PIN;
  HAL_GPIO_Init(PL_LED_RED_PORT, &gpio);
}

/* HSI → 160 MHz sysclk. Lifted from SDDataLogFileX (verified working).
   HSE is unreliable on this board's 3.3 V mod — see CLAUDE.md. */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef     osc  = {0};
  RCC_ClkInitTypeDef     clk  = {0};
  RCC_PeriphCLKInitTypeDef pc = {0};

  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
    Error_Handler(__FILE__, __LINE__);
  }

  osc.OscillatorType      = RCC_OSCILLATORTYPE_HSI48 | RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
  osc.HSIState            = RCC_HSI_ON;
  osc.HSI48State          = RCC_HSI48_ON;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.LSIState            = RCC_LSI_ON;
  osc.LSIDiv              = RCC_LSI_DIV1;
  osc.PLL.PLLState        = RCC_PLL_ON;
  osc.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  osc.PLL.PLLMBOOST       = RCC_PLLMBOOST_DIV1;
  osc.PLL.PLLM            = 1;
  osc.PLL.PLLN            = 10;
  osc.PLL.PLLP            = 1;
  osc.PLL.PLLQ            = 2;
  osc.PLL.PLLR            = 1;
  osc.PLL.PLLRGE          = RCC_PLLVCIRANGE_1;
  osc.PLL.PLLFRACN        = 0;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) {
    Error_Handler(__FILE__, __LINE__);
  }

  clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                       RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_PCLK3;
  clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV1;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  clk.APB3CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_4) != HAL_OK) {
    Error_Handler(__FILE__, __LINE__);
  }

  pc.PeriphClockSelection = RCC_PERIPHCLK_MDF1 | RCC_PERIPHCLK_ADF1 | RCC_PERIPHCLK_ADCDAC;
  pc.Mdf1ClockSelection   = RCC_MDF1CLKSOURCE_PLL3;
  pc.Adf1ClockSelection   = RCC_ADF1CLKSOURCE_PLL3;
  pc.AdcDacClockSelection = RCC_ADCDACCLKSOURCE_PLL2;
  pc.PLL3.PLL3Source      = RCC_PLLSOURCE_HSI;
  pc.PLL3.PLL3M           = 2;
  pc.PLL3.PLL3N           = 48;
  pc.PLL3.PLL3P           = 2;
  pc.PLL3.PLL3Q           = 25;
  pc.PLL3.PLL3R           = 2;
  pc.PLL3.PLL3RGE         = RCC_PLLVCIRANGE_1;
  pc.PLL3.PLL3FRACN       = 0;
  pc.PLL3.PLL3ClockOut    = RCC_PLL3_DIVQ;
  pc.PLL2.PLL2Source      = RCC_PLLSOURCE_HSI;
  pc.PLL2.PLL2M           = 2;
  pc.PLL2.PLL2N           = 48;
  pc.PLL2.PLL2P           = 2;
  pc.PLL2.PLL2Q           = 7;
  pc.PLL2.PLL2R           = 25;
  pc.PLL2.PLL2RGE         = RCC_PLLVCIRANGE_1;
  pc.PLL2.PLL2FRACN       = 0;
  pc.PLL2.PLL2ClockOut    = RCC_PLL2_DIVR;
  if (HAL_RCCEx_PeriphCLKConfig(&pc) != HAL_OK) {
    Error_Handler(__FILE__, __LINE__);
  }

  /* Phase 3 peripheral kernel-clock sources. Required so I²C2 / SPI2 /
     UART4 / I²C4 run at the rate their Timing/baud config assumes
     (160 MHz PCLK1 for I²C2/SPI2/UART4; HSI16 for I²C4 to match
     SDDataLogFileX). Without these, the peripherals default to HSI16
     and the timing values produce wrong frequencies → bus dead. */
  RCC_PeriphCLKInitTypeDef bus = {0};
  bus.PeriphClockSelection = RCC_PERIPHCLK_I2C1 | RCC_PERIPHCLK_I2C4
                           | RCC_PERIPHCLK_SPI2 | RCC_PERIPHCLK_UART4;
  bus.I2c1ClockSelection   = RCC_I2C1CLKSOURCE_PCLK1;
  bus.I2c4ClockSelection   = RCC_I2C4CLKSOURCE_PCLK1;
  bus.Spi2ClockSelection   = RCC_SPI2CLKSOURCE_PCLK1;
  bus.Uart4ClockSelection  = RCC_UART4CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&bus) != HAL_OK) {
    Error_Handler(__FILE__, __LINE__);
  }
}

void Error_Handler(const char *file, int line)
{
  (void)file; (void)line;
  /* Fatal-error LED+buzzer convention (DESIGN.md §11-ish):
       red LED on solid + green LED dark → "the box is dead, check errlog"
       one long low-frequency beep → audible even if the box is in a
       rucksack or you're not looking at it.
     Order matters: beep + LED updates BEFORE __disable_irq, because
     Buzzer_Beep uses HAL_Delay which spins on SysTick and would hang
     forever once interrupts are off. Once the alarm has sounded we
     disable IRQs and park the CPU. */
  HAL_GPIO_WritePin(PL_LED_RED_PORT,   PL_LED_RED_PIN,   GPIO_PIN_SET);
  HAL_GPIO_WritePin(PL_LED_GREEN_PORT, PL_LED_GREEN_PIN, GPIO_PIN_RESET);
  Buzzer_Beep(800U, 800U);          /* 800 Hz, 0.8 s = unmistakable alarm */
  __disable_irq();
  for (;;) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file; (void)line;
  Error_Handler((const char *)file, (int)line);
}
#endif

void HAL_MspInit(void)
{
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWREx_DisableUCPDDeadBattery();
  HAL_PWREx_EnableVddIO2();
  HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY);
}
