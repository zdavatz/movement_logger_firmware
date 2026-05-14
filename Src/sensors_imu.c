/**
  ******************************************************************************
  * @file    sensors_imu.c
  * @brief   LSM6DSV16X via SPI2, polled, no DMA.
  *
  *          Wire layout (BSP, lifted into PumpLogger):
  *            SPI2_SCK  = PI1   AF5
  *            SPI2_MISO = PI2   AF5
  *            SPI2_MOSI = PI3   AF5
  *            NSS       = PI5   GPIO output, software-driven
  *          Baud = 160 MHz / 32 = 5 MHz (sensor spec ≤ 10 MHz).
  ******************************************************************************
  */
#include "main.h"
#include "sensors_imu.h"
#include <string.h>
#include <stdio.h>

extern void ErrLog_Write(const char *msg);

#define LSM_REG_WHO_AM_I        0x0FU
#define LSM_REG_CTRL1_XL        0x10U
#define LSM_REG_CTRL2_G         0x11U
#define LSM_REG_CTRL3_C         0x12U
#define LSM_REG_CTRL6_G         0x15U
#define LSM_REG_CTRL8_XL        0x17U
#define LSM_REG_OUTX_L_G        0x22U
#define LSM_WHO_AM_I_VAL        0x70U

static SPI_HandleTypeDef g_hspi2;
static int g_ok = 0;

static inline void cs_lo(void) {
  HAL_GPIO_WritePin(PL_LSM6_CS_PORT, PL_LSM6_CS_PIN, GPIO_PIN_RESET);
}
static inline void cs_hi(void) {
  HAL_GPIO_WritePin(PL_LSM6_CS_PORT, PL_LSM6_CS_PIN, GPIO_PIN_SET);
}

static int spi_xfer(uint8_t *tx, uint8_t *rx, uint16_t len)
{
  return (HAL_SPI_TransmitReceive(&g_hspi2, tx, rx, len, 100) == HAL_OK) ? 0 : -1;
}

static int imu_write_reg(uint8_t reg, uint8_t val)
{
  uint8_t tx[2] = { reg & 0x7Fu, val };
  uint8_t rx[2];
  cs_lo();
  int r = spi_xfer(tx, rx, 2);
  cs_hi();
  return r;
}

static int imu_read_buf(uint8_t reg, uint8_t *buf, uint16_t len)
{
  /* Single transfer: 1 cmd byte + len data bytes. LSM6DSV16X expects an
     unbroken CS window over the whole address+data sequence; the
     previous code split it into two TransmitReceive calls, which
     introduced a tiny SCK gap that confused the chip's SPI mode
     detection (datasheet recommends one contiguous transaction).
     We use a local 17-byte scratch buffer (max len = 12 for OUTX_L_G
     burst + 1 cmd byte = 13). */
  uint8_t tx[17] = {0};
  uint8_t rx[17] = {0};
  if (len > 16) len = 16;
  tx[0] = reg | 0x80u;
  for (uint16_t i = 1; i <= len; i++) tx[i] = 0xFF;
  cs_lo();
  HAL_StatusTypeDef s = HAL_SPI_TransmitReceive(&g_hspi2, tx, rx, len + 1, 100);
  cs_hi();
  if (s != HAL_OK) return -1;
  for (uint16_t i = 0; i < len; i++) buf[i] = rx[i + 1];
  return 0;
}

int IMU_Init(void)
{
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_SPI2_CLK_ENABLE();

  /* CS pin */
  GPIO_InitTypeDef g = {0};
  g.Pin   = PL_LSM6_CS_PIN;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(PL_LSM6_CS_PORT, &g);
  cs_hi();

  /* SCK/MOSI/MISO on PI1/PI3/PI2 AF5 */
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_PULLUP;
  g.Speed     = GPIO_SPEED_FREQ_HIGH;
  g.Alternate = GPIO_AF5_SPI2;
  g.Pin       = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOI, &g);
  g.Pull = GPIO_NOPULL;
  g.Pin  = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOI, &g);
  g.Pin  = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOI, &g);

  memset(&g_hspi2, 0, sizeof(g_hspi2));
  g_hspi2.Instance               = SPI2;
  g_hspi2.Init.Mode              = SPI_MODE_MASTER;
  g_hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
  g_hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
  g_hspi2.Init.CLKPolarity       = SPI_POLARITY_HIGH;
  g_hspi2.Init.CLKPhase          = SPI_PHASE_2EDGE;
  g_hspi2.Init.NSS               = SPI_NSS_SOFT;
  g_hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;  /* 1.25 MHz — slow + tolerant for bring-up */
  g_hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
  g_hspi2.Init.TIMode            = SPI_TIMODE_DISABLE;
  g_hspi2.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
  g_hspi2.Init.CRCPolynomial     = 7;
  g_hspi2.Init.NSSPMode          = SPI_NSS_PULSE_DISABLE;
  g_hspi2.Init.NSSPolarity       = SPI_NSS_POLARITY_LOW;
  g_hspi2.Init.FifoThreshold     = SPI_FIFO_THRESHOLD_01DATA;
  g_hspi2.Init.MasterSSIdleness  = SPI_MASTER_SS_IDLENESS_00CYCLE;
  g_hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  g_hspi2.Init.MasterReceiverAutoSusp  = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  g_hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_ENABLE;
  g_hspi2.Init.IOSwap            = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&g_hspi2) != HAL_OK) {
    ErrLog_Write("imu: HAL_SPI_Init FAIL");
    return -1;
  }

  /* Match BSP_SPI2_Init exactly: explicit autonomous-mode DISABLE. The
     STM32U5 SPI peripheral has an autonomous-mode register that may
     keep a stale trigger config from a previous boot. Without this
     call, the very first TransmitReceive looks correct but the slave
     responses come back garbled. (Same lesson as the BLE SPI in
     SDDataLogFileX v32.) */
  SPI_AutonomousModeConfTypeDef ac = {0};
  ac.TriggerState     = SPI_AUTO_MODE_DISABLE;
  ac.TriggerSelection = SPI_GRP1_GPDMA_CH0_TCF_TRG;
  ac.TriggerPolarity  = SPI_TRIG_POLARITY_RISING;
  if (HAL_SPIEx_SetConfigAutonomousMode(&g_hspi2, &ac) != HAL_OK) {
    ErrLog_Write("imu: SetConfigAutonomousMode FAIL");
    return -1;
  }

  HAL_Delay(20);

  /* Toggle CS a few times BEFORE any real read. On battery-powered
     boards the LSM6DSV16X never loses VDD across USB cycles, so it
     can still be in I²C mode from the previous firmware boot. A
     CS-LOW edge while SCK is idle re-selects SPI 4-wire mode. */
  for (int i = 0; i < 3; i++) {
    cs_lo();
    for (volatile int j = 0; j < 1000; j++) { __NOP(); }
    cs_hi();
    for (volatile int j = 0; j < 1000; j++) { __NOP(); }
  }

  /* Now blind-fire a SW_RESET on CTRL3_C. Even if the chip wakes up
     in some odd state, this brings it back to factory defaults. */
  (void)imu_write_reg(LSM_REG_CTRL3_C, 0x01);
  HAL_Delay(30);  /* chip needs ~25 ms after SW_RESET */

  /* WHO_AM_I check. */
  uint8_t who = 0xFF;
  int rc = imu_read_buf(LSM_REG_WHO_AM_I, &who, 1);
  {
    char m[48];
    snprintf(m, sizeof m, "imu: who_rc=%d val=0x%02x", rc, who);
    ErrLog_Write(m);
  }
  if (rc != 0) return -1;
  if (who != LSM_WHO_AM_I_VAL) return -2;

  /* Software reset + IF_INC + BDU. */
  imu_write_reg(LSM_REG_CTRL3_C, 0x05);   /* BOOT + SW_RESET-like */
  HAL_Delay(20);
  imu_write_reg(LSM_REG_CTRL3_C, 0x44);   /* BDU=1, IF_INC=1 */

  /* LSM6DSV16X register layout (per datasheet + lsm6dsv16x_reg.h):
       CTRL1 (0x10): odr_xl[3:0] | op_mode_xl[6:4]
       CTRL2 (0x11): odr_g[3:0]  | op_mode_g[6:4]
       CTRL6 (0x15): fs_g[3:0]   | lpf1_g_bw[6:4]
       CTRL8 (0x17): fs_xl[1:0]  | xl_dualc_en[3] | hp_lpf2_xl_bw[7:5]
     ODR codes for HP mode: 0x6=60Hz, 0x7=120Hz, 0x8=240Hz.
     FS_XL: 0b00=±2g, 0b01=±4g, 0b10=±8g, 0b11=±16g
     FS_G : 0b0000=±125dps, 0b0001=±250, 0b0010=±500, 0b0011=±1000, 0b0100=±2000
     The earlier 0x40 writes put 0b0000 in the ODR nibble — chip was sitting
     in POWER DOWN, which is why every sample read back as zero even though
     imu_ok=1049/1049 (SPI ACK'd, but the data registers held a stuck zero). */
  imu_write_reg(LSM_REG_CTRL1_XL, 0x07);  /* op_mode_xl=HP, odr_xl=120 Hz */
  imu_write_reg(LSM_REG_CTRL8_XL, 0x01);  /* fs_xl = ±4 g (0.122 mg/LSB — matches emit_sensor_row scaling) */
  imu_write_reg(LSM_REG_CTRL2_G,  0x07);  /* op_mode_g=HP, odr_g=120 Hz */
  imu_write_reg(LSM_REG_CTRL6_G,  0x02);  /* fs_g = ±500 dps (17.5 mdps/LSB — matches emit_sensor_row) */

  HAL_Delay(50);
  g_ok = 1;
  return 0;
}

int IMU_Read(PL_ImuSample *out)
{
  if (!g_ok || !out) return -1;
  uint8_t b[12];
  if (imu_read_buf(LSM_REG_OUTX_L_G, b, 12) != 0) return -1;
  out->gx = (int16_t)((b[1] << 8) | b[0]);
  out->gy = (int16_t)((b[3] << 8) | b[2]);
  out->gz = (int16_t)((b[5] << 8) | b[4]);
  out->ax = (int16_t)((b[7] << 8) | b[6]);
  out->ay = (int16_t)((b[9] << 8) | b[8]);
  out->az = (int16_t)((b[11] << 8) | b[10]);
  out->tick_ms = HAL_GetTick();
  out->valid   = 1;
  return 0;
}
