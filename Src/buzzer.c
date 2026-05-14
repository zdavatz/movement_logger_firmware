/**
  ******************************************************************************
  * @file    buzzer.c
  * @brief   Piezo PE13 over TIM1_CH3 PWM. Sysclk 160 MHz, APB2 prescaler 1
  *          → TIM1 input clock 160 MHz. Prescaler set so timer ticks @ 1 MHz,
  *          then period = 1_000_000 / freq for accurate audio.
  ******************************************************************************
  */

#include "main.h"
#include "buzzer.h"

static TIM_HandleTypeDef htim1_buzzer;
static uint8_t initialised = 0;

void Buzzer_Init(void)
{
  if (initialised) return;

  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {0};
  gpio.Pin       = PL_BUZZER_PIN;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_NOPULL;
  gpio.Speed     = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = PL_BUZZER_TIM_AF;
  HAL_GPIO_Init(PL_BUZZER_PORT, &gpio);

  htim1_buzzer.Instance               = TIM1;
  htim1_buzzer.Init.Prescaler         = (SystemCoreClock / 1000000U) - 1U;
  htim1_buzzer.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim1_buzzer.Init.Period            = 1000U - 1U;
  htim1_buzzer.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
  htim1_buzzer.Init.RepetitionCounter = 0;
  htim1_buzzer.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1_buzzer) != HAL_OK) return;

  TIM_OC_InitTypeDef oc = {0};
  oc.OCMode      = TIM_OCMODE_PWM1;
  oc.Pulse       = 0;
  oc.OCPolarity  = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  HAL_TIM_PWM_ConfigChannel(&htim1_buzzer, &oc, TIM_CHANNEL_3);

  TIM_BreakDeadTimeConfigTypeDef bdt = {0};
  bdt.OffStateRunMode  = TIM_OSSR_DISABLE;
  bdt.OffStateIDLEMode = TIM_OSSI_DISABLE;
  bdt.LockLevel        = TIM_LOCKLEVEL_OFF;
  bdt.BreakState       = TIM_BREAK_DISABLE;
  bdt.BreakPolarity    = TIM_BREAKPOLARITY_HIGH;
  bdt.Break2State      = TIM_BREAK2_DISABLE;
  bdt.Break2Polarity   = TIM_BREAK2POLARITY_HIGH;
  bdt.AutomaticOutput  = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim1_buzzer, &bdt);

  initialised = 1;
}

void Buzzer_Beep(uint32_t freq_hz, uint16_t duration_ms)
{
  if (!initialised || freq_hz == 0) return;

  uint32_t period = (1000000U / freq_hz);
  if (period < 2) period = 2;
  __HAL_TIM_SET_AUTORELOAD(&htim1_buzzer, period - 1U);
  __HAL_TIM_SET_COMPARE(&htim1_buzzer, TIM_CHANNEL_3, period / 2U);
  __HAL_TIM_SET_COUNTER(&htim1_buzzer, 0);

  HAL_TIM_PWM_Start(&htim1_buzzer, TIM_CHANNEL_3);
  if (duration_ms) {
    HAL_Delay(duration_ms);
    HAL_TIM_PWM_Stop(&htim1_buzzer, TIM_CHANNEL_3);
  }
}

void Buzzer_BootDone(void)
{
  Buzzer_Beep(1500U, 90U);
  HAL_Delay(60);
  Buzzer_Beep(3000U, 90U);
}
