#include <SI_EFM8LB1_Register_Enums.h>
#include "EFM8LB1_DAC.h"

static uint16_t dac0_code;
static uint16_t dac1_code;

void DAC0_Init(void)
{
    uint8_t sfrpage_save = SFRPAGE;

    /* DAC0 fixed-voltage test: output full-scale voltage on P3.0. */
    SFRPAGE = 0x20;
    P3MDIN = (P3MDIN & (uint8_t)~P3MDIN_B0__BMASK) | P3MDIN_B0__ANALOG;
    P3MDOUT = (P3MDOUT & (uint8_t)~P3MDOUT_B0__BMASK) | P3MDOUT_B0__OPEN_DRAIN;

    /* Use VDD as DAC0/1 reference so full-scale is close to VDD. */
    SFRPAGE = 0x30;
    DACGCF0 = DACGCF0_D01REFSL__VDD
            | DACGCF0_D23REFSL__VDD
            | DACGCF0_D1SRC__DAC1
            | DACGCF0_D3SRC__DAC3
            | DACGCF0_D1AMEN__NORMAL
            | DACGCF0_D3AMEN__NORMAL;

    DAC0CF0 = DAC0CF0_EN__ENABLE
            | DAC0CF0_UPDATE__SYSCLK
            | DAC0CF0_LJST__RIGHT_JUSTIFY
            | DAC0CF0_RSTMD__NORMAL;

    SFRPAGE = sfrpage_save;

    DAC0_SetCode(DAC0_MAX_CODE);
}

void DAC1_Init(void)
{
    uint8_t sfrpage_save = SFRPAGE;

    /* DAC1 fixed-voltage test: output full-scale voltage on P3.1. */
    SFRPAGE = 0x20;
    P3MDIN = (P3MDIN & (uint8_t)~P3MDIN_B1__BMASK) | P3MDIN_B1__ANALOG;
    P3MDOUT = (P3MDOUT & (uint8_t)~P3MDOUT_B1__BMASK) | P3MDOUT_B1__OPEN_DRAIN;

    /* Use VDD as DAC0/1 reference so full-scale is close to VDD. */
    SFRPAGE = 0x30;
    DACGCF0 = DACGCF0_D01REFSL__VDD
            | DACGCF0_D23REFSL__VDD
            | DACGCF0_D1SRC__DAC1
            | DACGCF0_D3SRC__DAC3
            | DACGCF0_D1AMEN__NORMAL
            | DACGCF0_D3AMEN__NORMAL;

    DAC1CF0 = DAC1CF0_EN__ENABLE
            | DAC1CF0_UPDATE__SYSCLK
            | DAC1CF0_LJST__RIGHT_JUSTIFY
            | DAC1CF0_RSTMD__NORMAL;

    SFRPAGE = sfrpage_save;

    DAC1_SetCode(DAC1_MIN_CODE);
}

void DAC0_SetCode(uint16_t mycode)
{
    uint8_t sfrpage_save = SFRPAGE;

    if (mycode > DAC0_MAX_CODE)
    {
        mycode = DAC0_MAX_CODE;
    }

    dac0_code = mycode;

    SFRPAGE = 0x30;
    DAC0H = (uint8_t)(mycode >> 8);
    DAC0L = (uint8_t)(mycode & 0xFF);
    SFRPAGE = sfrpage_save;
}

uint16_t DAC0_GetCode(void)
{
    return dac0_code;
}

void DAC1_SetCode(uint16_t mycode)
{
    uint8_t sfrpage_save = SFRPAGE;

    if (mycode > DAC1_MAX_CODE)
    {
        mycode = DAC1_MAX_CODE;
    }

    dac1_code = mycode;

    SFRPAGE = 0x30;
    DAC1H = (uint8_t)(mycode >> 8);
    DAC1L = (uint8_t)(mycode & 0xFF);
    SFRPAGE = sfrpage_save;
}

uint16_t DAC1_GetCode(void)
{
    return dac1_code;
}






/////////////////////////////////////////////////////////


//#define SINE_LENGTH 64

//static uint16_t code sin_table[SINE_LENGTH] =
//{
//    2048, 2249, 2447, 2642, 2831, 3013, 3185, 3346,
//    3495, 3630, 3750, 3853, 3939, 4007, 4056, 4085,
//    4095, 4085, 4056, 4007, 3939, 3853, 3750, 3630,
//    3495, 3346, 3185, 3013, 2831, 2642, 2447, 2249,
//    2048, 1847, 1649, 1454, 1265, 1083, 911, 750,
//    601,  466,  346,  243,  157,  89,   40,   11,
//    0,    11,   40,   89,   157,  243,  346,  466,
//    601,  750,  911, 1083, 1265, 1454, 1649, 1847
//};

//uint16_t sin_idx = 0;

//void DAC0_SinWave(void)
//{
//    DAC0_SetCode(sin_table[sin_idx]);

//    sin_idx++;

//    if (sin_idx >= SINE_LENGTH)
//    {
//        sin_idx = 0;
//    }
//}






