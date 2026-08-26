#ifndef EFM8LB1_DAC_H
#define EFM8LB1_DAC_H

#include <stdint.h>

#define DAC1_MIN_CODE 0u
#define DAC1_MAX_CODE 4095u

void DAC1_Init(void);
void DAC1_SetCode(uint16_t mycode);
uint16_t DAC1_GetCode(void);

#endif
