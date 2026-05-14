/**
  ******************************************************************************
  * @file    sensors_fuel.c
  * @brief   STC3115 fuel-gauge on I²C4. Polled.
  *
  *          I²C4 pins (BSP):
  *            SCL = PD12  AF4
  *            SDA = PD13  AF4
  *          7-bit addr 0x70 (8-bit 0xE0).
  ******************************************************************************
  */
#include "main.h"
#include "sensors_fuel.h"
#include <string.h>

static I2C_HandleTypeDef g_hi2c4;
static int g_ok = 0;

#define REG_MODE            0x00
#define REG_CTRL            0x01
#define REG_SOC_L           0x02
#define REG_COUNTER_L       0x04
#define REG_CURRENT_L       0x06
#define REG_VOLTAGE_L       0x08

static int rd(uint8_t reg, uint8_t *buf, uint16_t len)
{
  return (HAL_I2C_Mem_Read(&g_hi2c4, PL_STC3115_I2C_ADDR, reg,
                           I2C_MEMADD_SIZE_8BIT, buf, len, 100) == HAL_OK) ? 0 : -1;
}

static int wr(uint8_t reg, uint8_t v)
{
  return (HAL_I2C_Mem_Write(&g_hi2c4, PL_STC3115_I2C_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, &v, 1, 100) == HAL_OK) ? 0 : -1;
}

int FUEL_Init(void)
{
  __HAL_RCC_I2C4_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  GPIO_InitTypeDef g = {0};
  g.Mode      = GPIO_MODE_AF_OD;
  g.Pull      = GPIO_PULLUP;
  g.Speed     = GPIO_SPEED_FREQ_LOW;
  g.Alternate = GPIO_AF4_I2C4;
  g.Pin       = GPIO_PIN_12 | GPIO_PIN_13;
  HAL_GPIO_Init(GPIOD, &g);

  memset(&g_hi2c4, 0, sizeof(g_hi2c4));
  g_hi2c4.Instance       = I2C4;
  g_hi2c4.Init.Timing    = 0xA040184A;   /* ~145 kHz @ PCLK1=160 MHz — the STC3115-safe value */
  g_hi2c4.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
  g_hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  g_hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  g_hi2c4.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&g_hi2c4) != HAL_OK) return -1;
  HAL_I2CEx_ConfigAnalogFilter(&g_hi2c4, I2C_ANALOGFILTER_ENABLE);
  HAL_I2CEx_ConfigDigitalFilter(&g_hi2c4, 0);

  if (HAL_I2C_IsDeviceReady(&g_hi2c4, PL_STC3115_I2C_ADDR, 3, 200) != HAL_OK)
    return -2;

  /* Default-mode init: GG_RUN=1, FORCE_CC=0, ADC res = 14 bit. */
  if (wr(REG_MODE, 0x10) != 0) return -3;   /* CC mode, GG run */
  HAL_Delay(10);
  g_ok = 1;
  return 0;
}

int FUEL_Read(PL_FuelSample *out)
{
  if (!g_ok || !out) return -1;
  uint8_t b[10];
  if (rd(REG_SOC_L, b, 8) != 0) return -1;

  uint16_t soc_raw = (uint16_t)((b[1] << 8) | b[0]);   /* 0..51200 = 0..100% (1/512 % per LSB) */
  /* counter at b[2..3] ignored */
  int16_t  cur_raw = (int16_t) ((b[5] << 8) | b[4]);
  uint16_t v_raw   = (uint16_t)((b[7] << 8) | b[6]);

  /* Voltage: VLSB ≈ 2.20 mV */
  out->voltage_mV     = (uint16_t)(((uint32_t)v_raw * 220 + 50) / 100);
  /* Current: 5.88 µV per LSB at 10 mΩ → I = raw * 5.88 / Rsense (mΩ) * 1 µA;
     with default 10 mΩ Rsense the chip reports raw * 0.588 µA.
     We report tenths of mA = 100 µA, so x100uA = raw * 0.588 / 100 ≈ raw / 170. */
  out->current_x100uA = (int16_t)(cur_raw / 170);
  /* SOC: chip reports 1/512 % per LSB; convert to 0.1 % = soc_raw * 10 / 512 */
  out->soc_x10        = (uint16_t)(((uint32_t)soc_raw * 10) / 512);
  out->tick_ms        = HAL_GetTick();
  out->valid          = 1;
  return 0;
}
