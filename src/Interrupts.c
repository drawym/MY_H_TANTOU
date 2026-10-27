








#include <SI_EFM8LB1_Register_Enums.h>
#include "EFM8LB1_DAC.h"
//-----------------------------------------------------------------------------
// Global Constants
//-----------------------------------------------------------------------------
// Configurator set for HFOSC0/8
#define SYSCLK             3062000

#define SINE_LENGTH        256u
#define DAC_MID_CODE       2048u

/* 消磁包络：500ms 满幅 + 1500ms 线性衰减 -> 归到中点结束
   一个正弦周期 = 256 点 × 54us = 13.82ms（约 72.3Hz），所以：
     500ms  ≈ 36 周期 (497.7ms)
     1500ms ≈ 109 周期 (1506.8ms)
   合计约 2.00s；衰减到 0 时输出已经停在中点，故不再额外保持 */
#define DEFLUX_FULL_CYCLES   36u   //满幅度的周期数  ≈500ms
#define DEFLUX_DECAY_CYCLES  109u  //衰减周期数      ≈1.5s
#define DEFLUX_HOLD_CYCLES   0u    //衰减结束即在中点，不再保持
#define DEFLUX_TOTAL_CYCLES  (DEFLUX_FULL_CYCLES + DEFLUX_DECAY_CYCLES + DEFLUX_HOLD_CYCLES)
#define DEFLUX_AMP_Q8_FULL   256u

static uint16_t sin_idx = 0u;
static uint16_t deflux_cycle_count = 0u;
static uint16_t deflux_amp_q8 = DEFLUX_AMP_Q8_FULL;
static uint16_t deflux_gain_q8 = DEFLUX_AMP_Q8_FULL;   /* 幅值增益 amp*amp_q8/2048，每个正弦周期算一次(省掉采样中断里的32位除法) */
static volatile uint8_t deflux_running = 0u;   // 1=正在执行退磁流程
volatile uint8_t deflux_trigger = 0u;   // 主循环写1请求执行一次
static uint16_t dac_mid_code = DAC_MID_CODE;   // 消磁正弦中点(DAC码)，默认2048(=VREF/2)，可用 deflux_set_mid_mv() 修改

static uint16_t code sin_table[SINE_LENGTH]  =
{
2048,2098,2148,2198,2248,2298,2348,2397,2447,2496,2545,2593,2642,2690,2737,2784,
2831,2877,2923,2968,3012,3056,3100,3143,3185,3226,3267,3307,3346,3385,3422,3459,
3495,3530,3564,3598,3630,3661,3692,3721,3750,3777,3803,3829,3853,3876,3898,3919,
3939,3957,3975,3991,4006,4020,4033,4045,4055,4064,4072,4079,4085,4089,4092,4094,
4095,4094,4092,4089,4085,4079,4072,4064,4055,4045,4033,4020,4006,3991,3975,3957,
3939,3919,3898,3876,3853,3829,3803,3777,3750,3721,3692,3661,3630,3598,3564,3530,
3495,3459,3422,3385,3346,3307,3267,3226,3185,3143,3100,3056,3012,2968,2923,2877,
2831,2784,2737,2690,2642,2593,2545,2496,2447,2397,2348,2298,2248,2198,2148,2098,
2048,1997,1947,1897,1847,1797,1747,1698,1648,1599,1550,1502,1453,1405,1358,1311,
1264,1218,1172,1127,1083,1039,995,952,910,869,828,788,749,710,673,636,
600,565,531,497,465,434,403,374,345,318,292,266,242,219,197,176,
156,138,120,104,89,75,62,50,40,31,23,16,10,6,3,1,
1,1,3,6,10,16,23,31,40,50,62,75,89,104,120,138,
156,176,197,219,242,266,292,318,345,374,403,434,465,497,531,565,
600,636,673,710,749,788,828,869,910,952,995,1039,1083,1127,1172,1218,
1264,1311,1358,1405,1453,1502,1550,1599,1648,1698,1747,1797,1847,1897,1947,1997
};



/* 设置消磁正弦的中点电压（mV），波形摆动轴与结束停留点均改为该中点。
 * mv: 0~2500mV（VREF=2.5V），对应 DAC 码 = mv*4096/2500。
 * 注意：若消磁运行中调用会立即跳到新中点输出。 */
void deflux_set_mid_mv(uint16_t mv)
{
   uint16_t code1;
   if (mv > 2500u) { mv = 2500u; }            // 限幅到 VREF
   code1 = (uint16_t)(((uint32_t)mv * 4096UL) / 2500UL);
   if (code1 > 4095u) { code1 = 4095u; }        // 12bit DAC 上限
   dac_mid_code = code1;
   DAC0_SetCode(dac_mid_code);                // 立即输出到新中点
}

/* 消磁是否还在跑：1=正在消磁，0=空闲（主循环用它实现"消磁结束才回包"） */
uint8_t deflux_busy(void)
{
   return deflux_running;
}

SI_INTERRUPT (TIMER2_ISR, TIMER2_IRQn)
{
   uint16_t sine_code;
   uint16_t out_code;
   int32_t out32;
   int16_t dev;
   uint16_t decay_cycle;
   uint16_t remain_cycle;

   TMR2CN0_TF2H = 0;

   /* 空闲：无触发直接返回，靠 DAC 锁存保持当前输出 */
   if (!deflux_running)
   {
      if (deflux_trigger == 0u)
      {
         return;
      }
      /* 消费触发，启动一次退磁流程 */
      deflux_trigger = 0u;
      deflux_running = 1u;
      sin_idx = 0u;
      deflux_cycle_count = 0u;
      deflux_amp_q8 = DEFLUX_AMP_Q8_FULL;
   }

   /* 运行中：正常生成正弦（不再有 else 分支，流程代码只有这一份） */
   sine_code = sin_table[sin_idx];

   /* 优化1：幅值增益每个正弦周期(256点)只算一次 gain = amp*amp_q8/2048 (≤256)
      原来每个采样点都要跑 2 次 32bit 除法 + 2 次 32bit 乘法：C51 的 32bit 除法是位串行库函数，
      一次上千 cycle，单次中断几乎占满 54us 的采样周期，把主循环的 CPU 全吃光了 */
   if (sin_idx == 0u)
   {
      uint16_t amp;
      amp = dac_mid_code;
      if (amp > (uint16_t)(4096u - dac_mid_code)) { amp = (uint16_t)(4096u - dac_mid_code); }
      deflux_gain_q8 = (uint16_t)(((uint32_t)amp * deflux_amp_q8) / DAC_MID_CODE);
   }

   /* 优化2：每个采样点只做 1 次乘法 + 1 次右移(代替 /256)，满幅/静音路径连乘法都省掉
      注：右移对负数是向下取整，与原 /256 最多差 1 个 DAC 码，对消磁波形无影响 */
   dev = (int16_t)sine_code - (int16_t)DAC_MID_CODE;                /* 相对原轴的偏差 ±2047 */
   if (deflux_gain_q8 == DEFLUX_AMP_Q8_FULL)
   {
      out32 = (int32_t)dac_mid_code + (int32_t)dev;                 /* 满幅：最常见路径，无乘法 */
   }
   else if (deflux_gain_q8 == 0u)
   {
      out32 = (int32_t)dac_mid_code;                                /* 衰减到底：停在中点 */
   }
   else
   {
      out32 = (int32_t)dac_mid_code + (((int32_t)dev * (int32_t)deflux_gain_q8) >> 8);
   }
   /* 12bit 限幅兜底（mid 接近满量程时峰值可能超 4095） */
   if (out32 < 0) { out32 = 0; }
   if (out32 > 4095) { out32 = 4095; }
   out_code = (uint16_t)out32;
   DAC0_SetCode(out_code);

   sin_idx++;
   if (sin_idx >= SINE_LENGTH)
   {
      sin_idx = 0u;
      deflux_cycle_count++;
      if (deflux_cycle_count >= DEFLUX_TOTAL_CYCLES)
      {
         /* 完整一次流程结束：停在中点，回到空闲 */
         deflux_cycle_count = 0u;
         deflux_running = 0u;
         deflux_amp_q8 = 0u;
         DAC0_SetCode(dac_mid_code);
      }
      else if (deflux_cycle_count < DEFLUX_FULL_CYCLES)
      {
         deflux_amp_q8 = DEFLUX_AMP_Q8_FULL;
      }
      else if (deflux_cycle_count < (DEFLUX_FULL_CYCLES + DEFLUX_DECAY_CYCLES))
      {
         decay_cycle = deflux_cycle_count - DEFLUX_FULL_CYCLES + 1u;
         if (decay_cycle >= DEFLUX_DECAY_CYCLES)
         {
            deflux_amp_q8 = 0u;
         }
         else
         {
            remain_cycle = DEFLUX_DECAY_CYCLES - decay_cycle;
            deflux_amp_q8 = (uint16_t)(((uint32_t)remain_cycle * DEFLUX_AMP_Q8_FULL) / DEFLUX_DECAY_CYCLES);
         }
      }
      else
      {
         deflux_amp_q8 = 0u;
      }
   }
}


volatile uint32_t counter = 0;
SI_INTERRUPT (TIMER0_ISR, TIMER0_IRQn)
{
   counter++;
}


