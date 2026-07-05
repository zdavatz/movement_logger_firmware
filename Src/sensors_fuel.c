/**
  ******************************************************************************
  * @file    sensors_fuel.c
  * @brief   STC3115 fuel-gauge on I²C4. Polled.
  *
  *          I²C4 pins (BSP):
  *            SCL = PD12  AF4
  *            SDA = PD13  AF4
  *          7-bit addr 0x70 (8-bit 0xE0).
  *
  *          v0.0.32: program the real battery model. Before this the init
  *          wrote only REG_MODE=0x10, so the gauge ran on POR defaults
  *          (1957 mAh @ 10 mΩ, Ri 190 mΩ — a completely different pack)
  *          and, with no state handling, re-preset SOC from its generic
  *          OCV curve at every power cycle: a Li-Po resting anywhere on
  *          the 3.8 V plateau read "~55 %" forever. Now:
  *
  *          - CC_CNF / VM_CNF / OCVTAB / CURRENT_THRES are programmed for
  *            the real pack (HiMax 752535, 480 mAh, Ri 160 mΩ) behind the
  *            board's R83 = 50 mΩ shunt — values verbatim from ST's own
  *            STC3115_Battery_Conf.h for STEVAL-MKBOXPRO; formulas and
  *            sequence from the STC3115 datasheet + ST's reference driver.
  *          - Warm/restore/cold start discrimination via the gauge's
  *            16-byte on-chip RAM (test word + CRC8, ST layout): if the
  *            gauge kept running through the Hall power cut (it sits on
  *            VBAT upstream of the switch on the stock board) its state
  *            is left untouched; if it reset but the RAM survived, SOC is
  *            restored; only a true first boot re-presets from OCV — now
  *            through the board-tuned curve.
  *          - FUEL_Read backs the running SOC up into the gauge RAM once
  *            per second — the box has no shutdown hook (F-PWR-5), so all
  *            persistence must be periodic.
  *          - Current conversion fixed: the register LSB is 5.88 µV
  *            ACROSS THE SHUNT (0.588 mA/LSB at 10 mΩ — the old comment
  *            had a µA/mA slip and the old /170 reported ~200× too low),
  *            and the real shunt is 50 mΩ → 117.6 µA/LSB.
  ******************************************************************************
  */
#include "main.h"
#include "sensors_fuel.h"
#include "errlog.h"
#include <string.h>

static I2C_HandleTypeDef g_hi2c4;
static int g_ok = 0;

/* Register map (STC3115 datasheet DocID023755) */
#define REG_MODE            0x00
#define REG_CTRL            0x01
#define REG_SOC_L           0x02
#define REG_COUNTER_L       0x04
#define REG_CURRENT_L       0x06
#define REG_VOLTAGE_L       0x08
#define REG_OCV_L           0x0D
#define REG_CC_CNF_L        0x0F
#define REG_VM_CNF_L        0x11
#define REG_CURRENT_THRES   0x15
#define REG_ID              0x18
#define REG_RAM             0x20
#define REG_OCVTAB          0x30

#define STC3115_CHIP_ID     0x14

/* REG_MODE bits */
#define MODE_VMODE          0x01   /* 1 = voltage-only gauge; 0 = mixed (CC active) */
#define MODE_GG_RUN         0x10   /* 1 = run, 0 = standby (params writable) */
/* REG_CTRL bits */
#define CTRL_GG_VM          0x04   /* read-only: which estimator feeds REG_SOC */
#define CTRL_BATFAIL        0x08
#define CTRL_PORDET         0x10
#define CTRL_CLEAR          0x03   /* clears PORDET+BATFAIL, frees ALM, resets conv counter */

/* Battery model — ST's own STC3115_Battery_Conf.h for this board, with the
   datasheet formulas in ST's integer form:
     CC_CNF = Rsense(mΩ)·Cnom(mAh)/49.556  → (480·50·250 + 6194)/12389  = 484
     VM_CNF = Rint(mΩ)·Cnom(mAh)/977.78    → (480·160·50 + 24444)/48889 =  79
     relax  = Cnom/20 = 24 mA; THRES LSB = 8×5.88 µV → (24<<9)/(24084/50) = 25 */
#define BATT_CC_CNF   ((PL_BATT_CAPACITY_MAH * PL_BATT_RSENSE_MOHM * 250 + 6194) / 12389)
#define BATT_VM_CNF   ((PL_BATT_CAPACITY_MAH * PL_BATT_RINT_MOHM * 50 + 24444) / 48889)
#define BATT_RELAX_MA (PL_BATT_CAPACITY_MAH / 20)
#define BATT_CURR_THRES ((BATT_RELAX_MA << 9) / (24084 / PL_BATT_RSENSE_MOHM))

/* Board-tuned offsets added to the chip's internal OCV curve (0.55 mV LSB,
   signed). Verbatim from ST's STC3115_Battery_Conf.h for STEVAL-MKBOXPRO. */
static const int8_t k_ocv_offset[16] = {
  0, -40, -100, -58, -31, -30, -9, -19, -36, -40, -4, -36, -40, -23, -41, -20
};

/* Gauge on-chip RAM (16 B @0x20) — survives an MCU power cut as long as the
   gauge itself stays powered. ST driver layout:
     [0..1] test word 0x53A9   [2..3] HRSOC (1/512 %)
     [4..5] CC_CNF             [6..7] VM_CNF
     [8]    SOC (%)            [9]    state 'I' (init) / 'R' (running)
     [15]   CRC8 (poly 0x07) over bytes 0..14 */
#define RAM_TSTWORD  0x53A9
#define RAM_SIZE     16
#define RAM_ST_INIT     'I'
#define RAM_ST_RUNNING  'R'
static uint8_t g_ram[RAM_SIZE];

static int rd(uint8_t reg, uint8_t *buf, uint16_t len)
{
  return (HAL_I2C_Mem_Read(&g_hi2c4, PL_STC3115_I2C_ADDR, reg,
                           I2C_MEMADD_SIZE_8BIT, buf, len, 100) == HAL_OK) ? 0 : -1;
}

static int wr_buf(uint8_t reg, const uint8_t *buf, uint16_t len)
{
  return (HAL_I2C_Mem_Write(&g_hi2c4, PL_STC3115_I2C_ADDR, reg,
                            I2C_MEMADD_SIZE_8BIT, (uint8_t *)buf, len, 100) == HAL_OK)
             ? 0 : -1;
}

static int wr(uint8_t reg, uint8_t v)  { return wr_buf(reg, &v, 1); }

static int wr16(uint8_t reg, uint16_t v)
{
  uint8_t b[2] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
  return wr_buf(reg, b, 2);
}

/* ST's RAM CRC8 (poly 0x07): crc over all 16 bytes == 0 iff byte 15 was
   written as the crc of bytes 0..14. Unsigned accumulator (ST's own uses
   int, which is signed-shift UB once stale high bits march up — result
   is bit-identical either way, we just avoid the UB). */
static uint8_t crc8(const uint8_t *d, int n)
{
  unsigned int crc = 0;
  for (int i = 0; i < n; i++) {
    crc ^= d[i];
    for (int j = 0; j < 8; j++) {
      crc <<= 1;
      if (crc & 0x100) crc ^= 7;
    }
  }
  return (uint8_t)(crc & 0xFF);
}

static int ram_valid(void)
{
  uint16_t tst = (uint16_t)(g_ram[0] | (g_ram[1] << 8));
  return tst == RAM_TSTWORD && crc8(g_ram, RAM_SIZE) == 0;
}

static void ram_seal_and_write(void)
{
  g_ram[15] = crc8(g_ram, RAM_SIZE - 1);
  (void)wr_buf(REG_RAM, g_ram, RAM_SIZE);   /* best-effort — retried next second */
}

/* ST SetParam: GG_RUN=0 (standby) → OCV curve + relax threshold + battery
   model → clear PORDET/BATFAIL → run in mixed mode, alarms off. The
   final GG_RUN write happens even when a parameter write failed — a
   gauge stranded in standby integrates nothing for the whole session,
   which is strictly worse than running on a partly-stale model. */
static int set_param(void)
{
  if (wr(REG_MODE, MODE_VMODE) != 0) return -1;              /* standby */
  int rc = 0;
  rc |= wr_buf(REG_OCVTAB, (const uint8_t *)k_ocv_offset, 16);
  rc |= wr(REG_CURRENT_THRES, (uint8_t)BATT_CURR_THRES);
  rc |= wr16(REG_CC_CNF_L, (uint16_t)BATT_CC_CNF);
  rc |= wr16(REG_VM_CNF_L, (uint16_t)BATT_VM_CNF);
  rc |= wr(REG_CTRL, CTRL_CLEAR);
  rc |= wr(REG_MODE, MODE_GG_RUN);                           /* mixed mode, run */
  return rc ? -1 : 0;
}

/* Cold-start core, ST's STC3115_Startup: preserve the chip's own OCV
   reading across the parameter load so SOC re-derives through the
   corrected curve + model (an OCV taken later, under the box's load,
   would read several % low through Ri). */
static int startup_from_ocv(void)
{
  uint8_t ocv[2] = {0};
  if (rd(REG_OCV_L, ocv, 2) != 0) return -1;
  if (set_param() != 0) return -1;
  if (wr_buf(REG_OCV_L, ocv, 2) != 0) return -1;
  return 0;
}

/* Fresh RAM image for a cold start: model recorded, no SOC backup yet
   (state 'I' — the first FUEL_Read flips it to 'R' with a live SOC). */
static void init_ram_cold(void)
{
  memset(g_ram, 0, sizeof(g_ram));
  g_ram[0] = (uint8_t)(RAM_TSTWORD & 0xFF);
  g_ram[1] = (uint8_t)(RAM_TSTWORD >> 8);
  g_ram[4] = (uint8_t)(BATT_CC_CNF & 0xFF);
  g_ram[5] = (uint8_t)(BATT_CC_CNF >> 8);
  g_ram[6] = (uint8_t)(BATT_VM_CNF & 0xFF);
  g_ram[7] = (uint8_t)(BATT_VM_CNF >> 8);
  g_ram[9] = RAM_ST_INIT;
  ram_seal_and_write();
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

  /* The STC3115 can be slow to ACK right after a cold power-up (battery just
     plugged in, wireless-charger rail still settling). Probe a few rounds with
     a short settle between them instead of a single shot, so a slow-but-alive
     gauge isn't written off as dead — which would leave BatNNN.csv header-only
     for the whole boot (logger.c emit_bat_row only runs on FUEL_Read()==0).
     The outer loop IS the retry mechanism, so the HAL call uses Trials=1 (no
     stacked retry), and there is no settle after the final probe. Boot-time
     only: runs before Watchdog_Init and never from the superloop, so the
     worst-case ~0.7 s on a dead gauge is harmless. */
  int ready = 0;
  for (int attempt = 0; attempt < 5; attempt++) {
    if (HAL_I2C_IsDeviceReady(&g_hi2c4, PL_STC3115_I2C_ADDR, 1, 100) == HAL_OK) {
      ready = 1;
      break;
    }
    if (attempt < 4) HAL_Delay(50);   /* settle before next probe; none after the last */
  }
  if (!ready) return -2;

  uint8_t id = 0;
  if (rd(REG_ID, &id, 1) != 0 || id != STC3115_CHIP_ID) {
    ErrLog_Writef("fuel: bad chip id 0x%02x", id);
    return -3;
  }

  /* Startup discrimination, ST-driver semantics adapted to this box: the
     battery is soldered in (never swapped) and power can vanish any
     millisecond with no shutdown hook (F-PWR-5). ST's 1.2 s BATFAIL
     debounce is deliberately dropped — boot-to-logging must stay ≤ 1 s
     (F-PWR-3), and the worst case of trusting a spurious flag here is a
     fresh OCV preset through the corrected curve. */
  uint8_t ctrl = 0, mode = 0;
  if (rd(REG_CTRL, &ctrl, 1) != 0) return -4;
  if (rd(REG_MODE, &mode, 1) != 0) return -4;
  if (rd(REG_RAM, g_ram, RAM_SIZE) != 0) return -4;

  int flags_clear = (ctrl & (CTRL_PORDET | CTRL_BATFAIL)) == 0;
  int running     = (mode & MODE_GG_RUN) != 0;
  uint16_t hrsoc  = (uint16_t)(g_ram[2] | (g_ram[3] << 8));

  if (ram_valid() && flags_clear && running) {
    /* Warm: the gauge (on VBAT, upstream of the Hall switch on the stock
       board) kept running through the MCU power cut with our model still
       loaded — touch nothing, its SOC is continuous. Log the LIVE SOC,
       not the RAM backup (which is 0 while state is still 'I'). */
    uint8_t s[2] = {0};
    uint16_t live = (rd(REG_SOC_L, s, 2) == 0) ? (uint16_t)(s[0] | (s[1] << 8)) : 0;
    ErrLog_Writef("fuel: warm (gauge running, soc=%u.%u%%)",
                  (unsigned)(live / 512), (unsigned)((live * 10 / 512) % 10));
  } else if (!ram_valid() && flags_clear && running) {
    /* Torn-seal fallback: PORDET can only be clear because OUR set_param
       cleared it, and the gauge is still running — so the model is loaded
       and only the 1 Hz RAM backup was cut mid-write (the seal is not
       atomic; F-PWR-5 can kill power inside the 16-byte transfer).
       Verify the model readback to be sure, then treat as warm and
       rebuild the backup from the live SOC instead of discarding the
       gauge's continuity with a cold OCV re-preset. */
    uint8_t cc[2] = {0}, s[2] = {0};
    if (rd(REG_CC_CNF_L, cc, 2) == 0
        && (uint16_t)(cc[0] | (cc[1] << 8)) == (uint16_t)BATT_CC_CNF
        && rd(REG_SOC_L, s, 2) == 0) {
      uint16_t live = (uint16_t)(s[0] | (s[1] << 8));
      memset(g_ram, 0, sizeof(g_ram));
      g_ram[0] = (uint8_t)(RAM_TSTWORD & 0xFF);
      g_ram[1] = (uint8_t)(RAM_TSTWORD >> 8);
      g_ram[2] = s[0];
      g_ram[3] = s[1];
      g_ram[4] = (uint8_t)(BATT_CC_CNF & 0xFF);
      g_ram[5] = (uint8_t)(BATT_CC_CNF >> 8);
      g_ram[6] = (uint8_t)(BATT_VM_CNF & 0xFF);
      g_ram[7] = (uint8_t)(BATT_VM_CNF >> 8);
      g_ram[9] = RAM_ST_RUNNING;
      ram_seal_and_write();
      ErrLog_Writef("fuel: warm (torn backup resealed, soc=%u.%u%%)",
                    (unsigned)(live / 512), (unsigned)((live * 10 / 512) % 10));
    } else {
      /* Model readback mismatch — not our gauge state after all. */
      if (startup_from_ocv() != 0) return -5;
      init_ram_cold();
      ErrLog_Writef("fuel: cold-start (model mismatch, ctrl=0x%02x)", ctrl);
    }
  } else if (ram_valid() && g_ram[9] == RAM_ST_RUNNING && hrsoc != 0) {
    /* Restore: the gauge reset (or latched BATFAIL) but its RAM survived —
       battery unchanged (soldered), so reload the model and continue from
       the backed-up SOC instead of an OCV guess. */
    if (set_param() != 0) return -5;
    (void)wr16(REG_SOC_L, hrsoc);     /* writing REG_SOC restarts the algo there */
    ErrLog_Writef("fuel: restore soc=%u.%u%% (ctrl=0x%02x)",
                  (unsigned)(hrsoc / 512), (unsigned)((hrsoc * 10 / 512) % 10), ctrl);
  } else {
    /* Cold: first boot with this firmware, the gauge truly lost power
       (its RAM died with it), or RAM state 'I' with nothing to restore —
       ST startup, SOC re-derived from OCV through the corrected curve. */
    if (startup_from_ocv() != 0) return -5;
    init_ram_cold();
    ErrLog_Writef("fuel: cold-start (ctrl=0x%02x, cc=%d vm=%d thr=%d)",
                  ctrl, (int)BATT_CC_CNF, (int)BATT_VM_CNF, (int)BATT_CURR_THRES);
  }

  g_ok = 1;
  return 0;
}

int FUEL_Read(PL_FuelSample *out)
{
  if (!g_ok || !out) return -1;
  /* One transaction for MODE..VOLTAGE (0x00..0x09): the two status bytes
     ride along nearly for free and let us catch a mid-run gauge reset —
     ST's reference re-checks status every task cycle for the same reason. */
  uint8_t b[10];
  if (rd(REG_MODE, b, 10) != 0) return -1;
  uint8_t mode = b[0], ctrl = b[1];

  static uint8_t reset_logged = 0;
  if ((mode & MODE_GG_RUN) == 0 || (ctrl & (CTRL_PORDET | CTRL_BATFAIL)) != 0) {
    /* The gauge reset under us (VBAT droop the MCU rode through, or a
       latched BATFAIL glitch): its registers read 0 — do NOT publish the
       zeroed sample and do NOT let the 1 Hz backup clobber the last good
       SOC in g_ram. Reload the model and continue from that backup.
       Logged once per episode so a reset-looping gauge can't flood the
       errlog at the 1 Hz call cadence. */
    if (!reset_logged) {
      ErrLog_Writef("fuel: gauge reset mid-run (mode=0x%02x ctrl=0x%02x) — restoring",
                    mode, ctrl);
      reset_logged = 1;
    }
    if (set_param() == 0) {
      uint16_t hrsoc = (uint16_t)(g_ram[2] | (g_ram[3] << 8));
      if (g_ram[9] == RAM_ST_RUNNING && hrsoc != 0) {
        (void)wr16(REG_SOC_L, hrsoc);
      }
      ram_seal_and_write();
    }
    return -1;   /* sample invalid this second; next read publishes again */
  }
  reset_logged = 0;

  uint16_t soc_raw = (uint16_t)((b[3] << 8) | b[2]);   /* 0..51200 = 0..100% (1/512 % per LSB) */
  /* counter at b[4..5] ignored */
  int32_t  cur_raw = (int32_t)((b[7] << 8) | b[6]);
  uint16_t v_raw   = (uint16_t)(((b[9] << 8) | b[8]) & 0x0FFF); /* 12-bit field */

  /* Voltage: VLSB ≈ 2.20 mV */
  out->voltage_mV     = (uint16_t)(((uint32_t)v_raw * 220 + 50) / 100);
  /* Current: 14-bit two's complement (upper bits are NOT sign-extended by
     the chip — mask and extend by hand, per ST's driver). LSB is 5.88 µV
     across the shunt → 5880/Rsense(mΩ) µA per LSB = 117.6 µA at 50 mΩ.
     In 100 µA units: raw × 588 / (Rsense × 10). Discharge reads negative. */
  cur_raw &= 0x3FFF;
  if (cur_raw >= 0x2000) cur_raw -= 0x4000;
  out->current_x100uA = (int16_t)((cur_raw * 588) / (PL_BATT_RSENSE_MOHM * 10));
  /* SOC: chip reports 1/512 % per LSB; convert to 0.1 % (ST rounding).
     Clamp to 100.0 % — in mixed mode the coulomb counter integrates past
     51200 while charging (ST's driver clamps for the same reason), and
     the wire contract is 0..1000. */
  out->soc_x10        = (uint16_t)(((uint32_t)soc_raw * 10 + 256) / 512);
  if (out->soc_x10 > 1000) out->soc_x10 = 1000;
  out->tick_ms        = HAL_GetTick();
  out->valid          = 1;

  /* Periodic SOC backup into the gauge RAM (1 Hz caller cadence). The box
     has no shutdown hook, so this is the only persistence there is: if
     the gauge survives the next power cut we take the warm path; if the
     gauge resets but was re-powered before the next boot, the restore
     path picks this value up. Raw HRSOC (unclamped) like ST's driver;
     the display byte gets the clamped percent. ~1 ms of polled I²C/s. */
  g_ram[2] = (uint8_t)(soc_raw & 0xFF);
  g_ram[3] = (uint8_t)(soc_raw >> 8);
  g_ram[8] = (uint8_t)((out->soc_x10 + 5) / 10);
  g_ram[9] = RAM_ST_RUNNING;
  ram_seal_and_write();

  return 0;
}
