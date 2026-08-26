#include <SI_EFM8LB1_Register_Enums.h>
#include "EFM8LB1_DAC.h"

static uint16_t dac1_code;

void DAC1_Init(void)
{
    SFRPAGE = 0x00;
    P3MDIN &= (uint8_t)~P3MDIN_B1__DIGITAL;
    P3MDOUT &= (uint8_t)~P3MDOUT_B1__PUSH_PULL;

    REF0CN = REF0CN_VREFSL__VREF_2P4;
    DACGCF0 = DACGCF0_D01REFSL__VREF
            | DACGCF0_D23REFSL__VREF
            | DACGCF0_D1SRC__DAC1;
    DAC1CF0 = DAC1CF0_EN__ENABLE
            | DAC1CF0_UPDATE__SYSCLK
            | DAC1CF0_LJST__RIGHT_JUSTIFY
            | DAC1CF0_RSTMD__NORMAL;

    DAC1_SetCode(0);
}

void DAC1_SetCode(uint16_t mycode)
{
    if (mycode > DAC1_MAX_CODE)
    {
        mycode = DAC1_MAX_CODE;
    }

    dac1_code = mycode;
    DAC1H = (uint8_t)(mycode >> 8);
    DAC1L = (uint8_t)(mycode & 0xFF);
}

uint16_t DAC1_GetCode(void)
{
    return dac1_code;
}
