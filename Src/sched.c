/**
  ******************************************************************************
  * @file    sched.c
  * @brief   SysTick-driven cooperative scheduler with edge-triggered cadences.
  ******************************************************************************
  */

#include "main.h"
#include "sched.h"

static volatile uint32_t g_tick_count;
static uint32_t g_last_fire[PL_SCHED_SLOT_COUNT];

/* Called by HAL_IncTick() (which the SysTick_Handler calls). Keeps our
   own counter independent of HAL_GetTick(). */
void HAL_SYSTICK_Callback(void)
{
  g_tick_count++;
}

uint32_t sched_tick(void)
{
  return g_tick_count;
}

void sched_wait_next_tick(void)
{
  uint32_t before = g_tick_count;
  while (g_tick_count == before) {
    __WFI();
  }
}

bool sched_due(pl_sched_slot_t slot, uint32_t cadence_ticks)
{
  if (slot >= PL_SCHED_SLOT_COUNT) return false;
  uint32_t now = g_tick_count;
  if ((uint32_t)(now - g_last_fire[slot]) >= cadence_ticks) {
    g_last_fire[slot] = now;
    return true;
  }
  return false;
}
