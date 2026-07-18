/**
  ******************************************************************************
  * @file    clktrim.c
  * @brief   SysTick trim against the 32.768 kHz LSE crystal (v0.0.54).
  *
  *          Why: sysclk is HSI16 → PLL (the board's HSE is unusable on the
  *          3.3 V mod), and HSI16 is a ±1 % RC. Field measurement across 15
  *          host SET_TIME anchors in one session: the tick runs ~+4400 ppm
  *          fast (~0.26 s gained per minute, ~10 s over a 38-min session).
  *          Sessions with a single time anchor therefore drift visibly
  *          against video/wall clock.
  *
  *          Reference: the STM32's own LSE crystal — same 32.768 kHz class
  *          of reference the BLE chip uses for its sleep clock, but wired
  *          to this MCU. Whether it is fitted/working on the Rev_C box is
  *          unproven (ST's own MKBOXPRO examples never enable it), so this
  *          is a probe: LSEON is set non-blockingly at boot and everything
  *          degrades to today's behaviour if it never comes ready.
  *
  *          Mechanism: LPTIM1 free-runs from LSE (register-level, HAL LPTIM
  *          module is not compiled in). The superloop task samples
  *          (LPTIM1->CNT, HAL_GetTick()) pairs and once per
  *          CLK_TRIM_WINDOW_MS compares elapsed tick-ms vs elapsed LSE-ms;
  *          the error goes into the SysTick reload register, so after the
  *          first window 1 tick ≈ 1 wall millisecond (residual re-measured
  *          every window, crystal-grade ±30 ppm). The correction moves the
  *          tick everything shares — logger timestamps, sensor cadence
  *          (real 100.0 Hz), LED phases; peripheral bit clocks (UART/SPI/
  *          I²C baud) are untouched, they don't derive from SysTick.
  *
  *          Errlog: plain lines only, NEVER `***` (the desktop grader FAILs
  *          a boot on `***`, and an absent crystal is not a mission
  *          failure). One line when armed, one with the first applied trim,
  *          later lines only if the residual re-drifts (temperature).
  ******************************************************************************
  */

#include "stm32u5xx_hal.h"
#include "clktrim.h"
#include "config.h"
#include "errlog.h"

typedef enum {
  CT_WAIT_LSE = 0,   /* LSEON set, polling LSERDY (bounded)            */
  CT_MEASURE,        /* LPTIM running, accumulating the current window */
  CT_OFF             /* no LSE — permanently inactive this boot        */
} ct_state_t;

static ct_state_t g_state = CT_OFF;
static uint32_t   g_lse_wait_t0;     /* tick when LSEON was set           */
static uint16_t   g_last_raw;        /* last LPTIM1 CNT sample            */
static uint32_t   g_last_sample_ms;  /* tick at that sample               */
static uint64_t   g_lse_acc;         /* unwrapped LSE counts this window  */
static uint32_t   g_win_start_ms;    /* tick at window start              */
static int32_t    g_applied_ppm;     /* cumulative SysTick correction     */
static uint8_t    g_first_trim_done;

int32_t ClkTrim_AppliedPpm(void) { return g_applied_ppm; }

void ClkTrim_Init(void)
{
  /* RCC->BDCR lives in the backup domain — writable only with DBP set.
     Left set afterwards (harmless; nothing else here touches the domain). */
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();

  /* Medium-high drive for robust start on an uncharacterised crystal.
     Legal only while LSEON=0 (true at boot). */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_MEDIUMHIGH);
  SET_BIT(RCC->BDCR, RCC_BDCR_LSEON);
  /* LSEON alone feeds only the RTC domain. Peripherals (LPTIM1 kernel
     clock here) receive LSE only with LSESYSEN set — without it the
     first flash of v0.0.54 saw LSERDY but LPTIM1's ARR never synced
     (kernel clock dead) and the trim bailed with "LPTIM1 setup failed". */
  SET_BIT(RCC->BDCR, RCC_BDCR_LSESYSEN);

  g_lse_wait_t0 = HAL_GetTick();
  g_state       = CT_WAIT_LSE;
}

/* Double-read-until-stable: LPTIM1 counts in the asynchronous LSE domain,
   so a single CNT read can tear. Returns 0 on success. */
static int lptim_read(uint16_t *out)
{
  for (int i = 0; i < 8; i++) {
    uint16_t a = (uint16_t)LPTIM1->CNT;
    uint16_t b = (uint16_t)LPTIM1->CNT;
    if (a == b) { *out = a; return 0; }
  }
  return -1;
}

/* Bring LPTIM1 up free-running from LSE. Returns 0 on success, -1 on a
   kernel-clock mux failure, -2 when the ARR write never syncs into the
   LSE domain (kernel clock not actually ticking). */
static int lptim_start(void)
{
  RCC_PeriphCLKInitTypeDef pc = {0};
  pc.PeriphClockSelection   = RCC_PERIPHCLK_LPTIM1;
  pc.Lptim1ClockSelection   = RCC_LPTIM1CLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&pc) != HAL_OK) return -1;
  __HAL_RCC_LPTIM1_CLK_ENABLE();

  LPTIM1->CFGR = 0;                       /* kernel clock direct, /1 */
  LPTIM1->CR   = LPTIM_CR_ENABLE;
  LPTIM1->ICR  = LPTIM_ICR_ARROKCF;
  LPTIM1->ARR  = 0xFFFFu;                 /* full 16-bit, wraps every 2 s */
  /* ARR write syncs into the LSE domain (a couple of 30.5 µs cycles);
     bounded wait, torn setup degrades to CT_OFF rather than blocking. */
  uint32_t t0 = HAL_GetTick();
  while (!(LPTIM1->ISR & LPTIM_ISR_ARROK)) {
    if (HAL_GetTick() - t0 > 10u) return -2;
  }
  LPTIM1->CR = LPTIM_CR_ENABLE | LPTIM_CR_CNTSTRT;
  return 0;
}

/* Slew SysTick by err_ppm (positive = tick currently fast → lengthen the
   reload). Bounded to ±CLK_TRIM_MAX_PPM cumulative so a torn measurement
   can never run the tick away. */
static void apply_ppm(int32_t err_ppm)
{
  int32_t total = g_applied_ppm + err_ppm;
  if (total >  (int32_t)CLK_TRIM_MAX_PPM) total =  (int32_t)CLK_TRIM_MAX_PPM;
  if (total < -(int32_t)CLK_TRIM_MAX_PPM) total = -(int32_t)CLK_TRIM_MAX_PPM;
  err_ppm = total - g_applied_ppm;
  if (err_ppm == 0) return;

  uint32_t load    = SysTick->LOAD + 1u;
  uint32_t newload = (uint32_t)(((int64_t)load * (1000000 + err_ppm)) / 1000000);
  if (newload < 2u || newload > SysTick_LOAD_RELOAD_Msk) return;
  SysTick->LOAD  = newload - 1u;
  g_applied_ppm  = total;
}

static void window_reset(uint16_t raw, uint32_t now)
{
  g_last_raw       = raw;
  g_last_sample_ms = now;
  g_lse_acc        = 0;
  g_win_start_ms   = now;
}

void ClkTrim_Tick(void)
{
  uint32_t now = HAL_GetTick();

  switch (g_state) {
  case CT_OFF:
    return;

  case CT_WAIT_LSE:
    if ((RCC->BDCR & RCC_BDCR_LSERDY) && (RCC->BDCR & RCC_BDCR_LSESYSRDY)) {
      int rc = lptim_start();
      if (rc != 0) {
        ErrLog_Writef("clk: LSE ready but LPTIM1 setup failed (%s) — tick untrimmed",
                      (rc == -1) ? "kernel mux" : "ARR sync");
        g_state = CT_OFF;
        return;
      }
      uint16_t raw;
      if (lptim_read(&raw) != 0) raw = 0;
      window_reset(raw, now);
      ErrLog_Writef("clk: LSE ok — SysTick trim armed (HSI tick, %lu ms to ready)",
                    (unsigned long)(now - g_lse_wait_t0));
      g_state = CT_MEASURE;
    } else if (now - g_lse_wait_t0 > CLK_LSE_TIMEOUT_MS) {
      ErrLog_Write("clk: no LSE within 10 s — tick untrimmed (HSI RC, ~1% off)");
      g_state = CT_OFF;
    }
    return;

  case CT_MEASURE: {
    uint16_t raw;
    if (lptim_read(&raw) != 0) return;          /* torn read — next cadence */

    /* A gap > 3/4 of the LPTIM wrap period (2 s at 32768 Hz / 16 bit)
       makes the unwrap ambiguous (superloop stall, USB burst): restart
       the window instead of folding a wrong wrap count into it. */
    if (now - g_last_sample_ms > 1500u) {
      window_reset(raw, now);
      return;
    }
    g_lse_acc       += (uint16_t)(raw - g_last_raw);   /* mod-2^16 unwrap */
    g_last_raw       = raw;
    g_last_sample_ms = now;

    if (now - g_win_start_ms < CLK_TRIM_WINDOW_MS) return;

    uint64_t lse_ms = (g_lse_acc * 1000u) / 32768u;
    if (lse_ms == 0) { window_reset(raw, now); return; }
    int64_t err_ppm64 =
        (((int64_t)(now - g_win_start_ms) - (int64_t)lse_ms) * 1000000) /
        (int64_t)lse_ms;
    /* A window claiming worse than the RC's physical tolerance is a torn
       measurement, not a clock: discard. */
    if (err_ppm64 > 30000 || err_ppm64 < -30000) { window_reset(raw, now); return; }

    apply_ppm((int32_t)err_ppm64);
    if (!g_first_trim_done) {
      g_first_trim_done = 1;
      ErrLog_Writef("clk: trim %+ld ppm vs LSE — tick wall-locked",
                    (long)g_applied_ppm);
    } else if (err_ppm64 > 300 || err_ppm64 < -300) {
      /* Post-lock residual should be tens of ppm; a re-drift this big is
         temperature swing or a measurement artifact — worth a trace. */
      ErrLog_Writef("clk: re-trim %+ld ppm (total %+ld)",
                    (long)err_ppm64, (long)g_applied_ppm);
    }
    window_reset(raw, now);
    return;
  }
  }
}
