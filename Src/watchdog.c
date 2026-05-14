/**
  ******************************************************************************
  * @file    watchdog.c
  * @brief   IWDG hardware watchdog. No software logic — see watchdog.h.
  ******************************************************************************
  */

#include "main.h"
#include "watchdog.h"

static IWDG_HandleTypeDef hiwdg1;

void Watchdog_Init(void)
{
  /* LSI ≈ 32 kHz; prescaler /64, reload 4000 → period ≈ 8.0 s. SD writes
     (especially the first cluster-extension on a new session, or a BLE file
     transfer) can stall well over a second; 8 s leaves comfortable headroom
     while still catching a genuinely wedged main loop. */
  hiwdg1.Instance       = IWDG;
  hiwdg1.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg1.Init.Window    = IWDG_WINDOW_DISABLE;
  hiwdg1.Init.Reload    = 4000U;
  hiwdg1.Init.EWI       = 0;          /* no early-warning ISR — pure HW reset */
  (void)HAL_IWDG_Init(&hiwdg1);
}

void Watchdog_Tick(void)
{
  HAL_IWDG_Refresh(&hiwdg1);
}

void Watchdog_Kick(void)
{
  HAL_IWDG_Refresh(&hiwdg1);
}
