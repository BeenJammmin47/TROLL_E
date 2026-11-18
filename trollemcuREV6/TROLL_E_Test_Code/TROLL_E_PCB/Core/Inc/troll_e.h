/*
 * troll_e.h -- TROLL-E following-cart application
 *
 * Call TrollE_Init() once in main.c (USER CODE BEGIN 2)
 * and TrollE_Loop() inside the while(1) loop (USER CODE BEGIN 3).
 */
#ifndef TROLL_E_H
#define TROLL_E_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void TrollE_Init(void);
void TrollE_Loop(void);

/* PCB only: call from CDC_Receive_FS() in usbd_cdc_if.c (USER CODE BEGIN 6) */
void TrollE_UsbRx(const uint8_t *buf, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* TROLL_E_H */
