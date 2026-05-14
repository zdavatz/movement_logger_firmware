/**
  ******************************************************************************
  * @file    sensors_mag.c
  * @brief   LIS2MDL @ 100 Hz on I²C1 (PB6/PB7 AF4). Owns the I²C1 init —
  *          LPS22DF reads share the same handle via I2C1_ReadReg/WriteReg.
  ******************************************************************************
  */
#include "main.h"
#include "sensors_mag.h"
#include <string.h>
#include <stdio.h>

static I2C_HandleTypeDef g_hi2c1;
static int g_i2c1_inited = 0;
static int g_mag_ok = 0;

#define LIS2MDL_WHO_AM_I        0x4F
#define LIS2MDL_OFFSET_X_REG_L  0x45
#define LIS2MDL_WHO_AM_I_VAL    0x40
#define LIS2MDL_CFG_REG_A       0x60
#define LIS2MDL_CFG_REG_B       0x61
#define LIS2MDL_CFG_REG_C       0x62
#define LIS2MDL_OUTX_L_REG      0x68

int I2C1_Init(void)
{
  extern void ErrLog_Write(const char *msg);
  if (g_i2c1_inited) return 0;

  __HAL_RCC_GPIOB_CLK_ENABLE();

  /* ------------ Bus recovery before any I²C HAL setup ------------
     If a previous firmware was mid-transaction at reset, a slave may
     still be holding SDA low waiting for clock pulses to finish the
     byte. Standard rescue: drive SCL as a GPIO output, pulse it ≥ 16
     times, then synthesize a STOP. Slave releases SDA. */
  GPIO_InitTypeDef gpio = {0};
  gpio.Mode  = GPIO_MODE_OUTPUT_OD;
  gpio.Pull  = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Pin   = GPIO_PIN_6;            /* SCL */
  HAL_GPIO_Init(GPIOB, &gpio);
  gpio.Pin   = GPIO_PIN_7;            /* SDA */
  HAL_GPIO_Init(GPIOB, &gpio);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
  HAL_Delay(2);

  int sda_low_before = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET) ? 1 : 0;

  /* 16 SCL pulses at ~10 kHz. Enough for any stuck slave to bail. */
  for (int i = 0; i < 16; i++) {
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    for (volatile int j = 0; j < 1600; j++) { __NOP(); }  /* ~50 µs */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    for (volatile int j = 0; j < 1600; j++) { __NOP(); }
  }
  /* Synthesize STOP: SDA low → SDA high while SCL high. */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
  for (volatile int j = 0; j < 1600; j++) { __NOP(); }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
  for (volatile int j = 0; j < 1600; j++) { __NOP(); }
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);
  for (volatile int j = 0; j < 1600; j++) { __NOP(); }

  int sda_low_after = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET) ? 1 : 0;
  {
    char m[64];
    snprintf(m, sizeof m, "i2c1_recovery: sda_before=%d sda_after=%d",
             sda_low_before, sda_low_after);
    ErrLog_Write(m);
  }

  /* Now re-map PB6/PB7 to I²C1 AF4. */
  __HAL_RCC_I2C1_CLK_ENABLE();
  gpio.Mode      = GPIO_MODE_AF_OD;
  gpio.Pull      = GPIO_PULLUP;
  gpio.Speed     = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF4_I2C1;
  gpio.Pin       = GPIO_PIN_6 | GPIO_PIN_7;
  HAL_GPIO_Init(GPIOB, &gpio);

  /* Give the sensors a moment after recovery before reading WHO_AM_I. */
  HAL_Delay(50);

  memset(&g_hi2c1, 0, sizeof(g_hi2c1));
  g_hi2c1.Instance              = I2C1;
  g_hi2c1.Init.Timing           = 0xA040184A;   /* ~145 kHz @ PCLK1=160 MHz — slow + forgiving, matches the I²C4 timing fuel uses */
  g_hi2c1.Init.AddressingMode   = I2C_ADDRESSINGMODE_7BIT;
  g_hi2c1.Init.DualAddressMode  = I2C_DUALADDRESS_DISABLE;
  g_hi2c1.Init.GeneralCallMode  = I2C_GENERALCALL_DISABLE;
  g_hi2c1.Init.NoStretchMode    = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&g_hi2c1) != HAL_OK) return -1;
  HAL_I2CEx_ConfigAnalogFilter(&g_hi2c1, I2C_ANALOGFILTER_ENABLE);
  HAL_I2CEx_ConfigDigitalFilter(&g_hi2c1, 0);
  /* No NVIC enable — polled HAL calls. */

  g_i2c1_inited = 1;
  return 0;
}

int I2C1_ReadReg(uint16_t dev8, uint8_t reg, uint8_t *buf, uint16_t len)
{
  return (HAL_I2C_Mem_Read(&g_hi2c1, dev8, reg, I2C_MEMADD_SIZE_8BIT,
                           buf, len, 100) == HAL_OK) ? 0 : -1;
}

int I2C1_WriteReg(uint16_t dev8, uint8_t reg, const uint8_t *buf, uint16_t len)
{
  return (HAL_I2C_Mem_Write(&g_hi2c1, dev8, reg, I2C_MEMADD_SIZE_8BIT,
                            (uint8_t *)buf, len, 100) == HAL_OK) ? 0 : -1;
}

int MAG_Init(void)
{
  extern void ErrLog_Write(const char *msg);
  if (I2C1_Init() != 0) {
    ErrLog_Write("mag: I2C1_Init failed");
    return -1;
  }
  /* Probe both possible addresses + raise a flag for each step. */
  HAL_StatusTypeDef ready_a = HAL_I2C_IsDeviceReady(&g_hi2c1, PL_LIS2MDL_I2C_ADDR, 3, 200);
  {
    char m[64];
    snprintf(m, sizeof m, "mag: ready@0x%02x=%d err=0x%lx",
             (unsigned)PL_LIS2MDL_I2C_ADDR, (int)ready_a,
             (unsigned long)g_hi2c1.ErrorCode);
    ErrLog_Write(m);
  }
  uint8_t who = 0xFF;
  HAL_StatusTypeDef rc = HAL_I2C_Mem_Read(&g_hi2c1, PL_LIS2MDL_I2C_ADDR, LIS2MDL_WHO_AM_I,
                                          I2C_MEMADD_SIZE_8BIT, &who, 1, 200);
  {
    char m[64];
    snprintf(m, sizeof m, "mag: who_rc=%d val=0x%02x err=0x%lx",
             (int)rc, who, (unsigned long)g_hi2c1.ErrorCode);
    ErrLog_Write(m);
  }
  if (rc != HAL_OK) return -1;
  if (who != LIS2MDL_WHO_AM_I_VAL) return -2;

  /* CFG_REG_A:
       MD=00 (continuous), ODR=11 (100 Hz), LP=0, SOFT_RST=0,
       REBOOT=0, COMP_TEMP_EN=1, OFFSET_CANCELLATION=1
     bits: COMP_TEMP_EN bit7, REBOOT bit6, SOFT_RST bit5, LP bit4,
           ODR[3:2]=11, MD[1:0]=00 → 0x8C.
     But we also want offset-cancellation-every-ODR — that's bit4 of CFG_REG_B
     (LPF, set_pulse_freq, OFFSET_CANCELLATION).
   */
  uint8_t v;
  v = 0x8C;   /* COMP_TEMP_EN=1, ODR=100Hz, continuous */
  I2C1_WriteReg(PL_LIS2MDL_I2C_ADDR, LIS2MDL_CFG_REG_A, &v, 1);
  v = 0x02;   /* OFFSET_CANCELLATION enabled */
  I2C1_WriteReg(PL_LIS2MDL_I2C_ADDR, LIS2MDL_CFG_REG_B, &v, 1);
  v = 0x10;   /* BDU=1 (block data update) */
  I2C1_WriteReg(PL_LIS2MDL_I2C_ADDR, LIS2MDL_CFG_REG_C, &v, 1);

  HAL_Delay(10);
  g_mag_ok = 1;
  return 0;
}

int MAG_Read(PL_MagSample *out)
{
  if (!g_mag_ok || !out) return -1;
  uint8_t b[6];
  if (I2C1_ReadReg(PL_LIS2MDL_I2C_ADDR, LIS2MDL_OUTX_L_REG, b, 6) != 0) return -1;
  out->mx = (int16_t)((b[1] << 8) | b[0]);
  out->my = (int16_t)((b[3] << 8) | b[2]);
  out->mz = (int16_t)((b[5] << 8) | b[4]);
  out->tick_ms = HAL_GetTick();
  out->valid   = 1;
  return 0;
}
