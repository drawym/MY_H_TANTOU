
#ifndef EFM8LB1_DAC_H
#define EFM8LB1_DAC_H

#include <stdint.h>

#define DAC0_MIN_CODE 0u
#define DAC0_MAX_CODE 4095u
#define DAC1_MIN_CODE 0u
#define DAC1_MAX_CODE 4095u

#define DAC01_MID_CODE 2046u

void DAC0_Init(void);
void DAC0_SetCode(uint16_t mycode);
uint16_t DAC0_GetCode(void);
void DAC1_Init(void);
void DAC1_SetCode(uint16_t mycode);
uint16_t DAC1_GetCode(void);
void DAC0_SinWave(void);
#endif

