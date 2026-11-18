/*
 * troll_e.h -- TROLL-E following-cart application
 *
 * Call TrollE_Init() once in main.c (USER CODE BEGIN 2)
 * and TrollE_Loop() inside the while(1) loop (USER CODE BEGIN 3).
 */
#ifndef TROLL_E_H
#define TROLL_E_H

#ifdef __cplusplus
extern "C" {
#endif

void TrollE_Init(void);
void TrollE_Loop(void);

#ifdef __cplusplus
}
#endif

#endif /* TROLL_E_H */
