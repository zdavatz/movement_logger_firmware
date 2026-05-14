/**
  ******************************************************************************
  * @file    ble.h
  * @brief   BlueNRG-LP minimal HCI driver — polling-only, no EXTI11 ISR.
  *          Phase 4 deliverable: advertise as "PumpTsueri", visible on phone.
  *          GATT services + FileSync come in Phase 5.
  ******************************************************************************
  */
#ifndef PL_BLE_H
#define PL_BLE_H

#include <stdint.h>

int     BLE_Init(void);
void    BLE_Tick(void);
uint8_t BLE_IsAdvertising(void);

#endif /* PL_BLE_H */
