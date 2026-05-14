/**
  ******************************************************************************
  * @file    buzzer.h
  * @brief   Piezo on PE13 driven by TIM1_CH3 PWM. Ported from SDDataLogFileX.
  ******************************************************************************
  */

#ifndef PUMPLOGGER_BUZZER_H
#define PUMPLOGGER_BUZZER_H

#include <stdint.h>

void Buzzer_Init(void);
void Buzzer_Beep(uint32_t freq_hz, uint16_t duration_ms);
void Buzzer_BootDone(void);

#endif /* PUMPLOGGER_BUZZER_H */
