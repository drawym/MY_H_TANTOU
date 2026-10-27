
#include <SI_EFM8LB1_Register_Enums.h>
#include "retargetserial.h"
#include "InitDevice.h"
#include "EFM8LB1_FlashPrimitives.h"
#include "EFM8LB1_FlashUtils.h"
#include "EFM8LB1_DAC.h"
#include "string.h"


/*****************************************************宏定义部分*********************************************************** */
// EFM8LB10F16 是 16KB Flash ，最后一页地址为 0x3E00（页大小512字节）
// 这里使用倒数第1页的起始地址存放用户数据
#define FLASH_STORAGE_ADDR  0x3E00

//接收的最长数据
#define UART_RX_BUFFER_SIZE 20 
//定义帧头
#define FRAME_HEADER 0xAA
//定义零点控制的类型
#define ZERO_CONTROL_TYPE 0x05
//定义消磁功能的类型
#define DEFLUX_TYPE 0xC0
//定义继电器切换的类型
#define RELAY_SWITCH_TYPE 0xD0
//定义调零的类型
#define SET_ZERO_TYPE 0x15

//定义上报ADC_VOL的类型
#define ADC_VOL_TYPE 0x16

//定义调零校准的类型
#define ZERO_CAL_TYPE 0x20

		
/*****************************************************宏定义部分*********************************************************** */		
		


/*****************************************************外部声明部分*********************************************************** */		

extern volatile uint32_t counter ;//延时函数-中断里面
extern volatile uint8_t deflux_trigger ;   // 消磁的逻辑，设置为1则行一次-中断里面
void deflux_set_mid_mv(uint16_t mv);   // 设置消磁正弦中点电压(mV)
extern uint8_t deflux_busy(void);   // Interrupts.c：消磁是否还在跑（0=空闲）

/* 消磁回包状态：0=还没启动，1=已启动，等消磁真正跑完再回包 */
static uint8_t  deflux_wait_reply = 0;
static uint32_t deflux_start_tick = 0;
/* 调零前的等待状态：上位机若不等消磁回包就发调零，这里先等消磁真正结束 */
static uint8_t  zero_wait_started = 0;
static uint32_t zero_wait_tick = 0;
/* counter≈5.9kHz@72MHz → 180000 tick≈30s，给一次消磁(约9~10s)留足余量 */
#define DEFLUX_TIMEOUT_TICK  180000UL
/*****************************************************外部声明部分*********************************************************** */		




/*****************************************************延时函数*********************************************************** */		
/* Timer0 时基：SYSCLK/48/(256-0x01) = 72MHz/48/254 ≈ 5.9kHz，即 1ms ≈ 5.9 个 counter
   原来写 2*ms 是按 counter=2kHz 算的，72MHz 下实际延时只有设计值的 1/3（继电器脉宽会被缩短） */
#define COUNTER_TICKS_PER_MS_NUM   59u
#define COUNTER_TICKS_PER_MS_DEN   10u
/* ms -> counter tick 换算（counter≈5.9kHz@72MHz，1s≈5900 tick） */
#define COUNTER_TICKS(ms)   (((uint32_t)(ms) * COUNTER_TICKS_PER_MS_NUM) / COUNTER_TICKS_PER_MS_DEN)
void delay_ms(uint16_t ms)
{
   uint32_t start = counter;
   uint32_t ticks = ((uint32_t)ms * COUNTER_TICKS_PER_MS_NUM) / COUNTER_TICKS_PER_MS_DEN;

   while((counter - start) < ticks)
   {
   };
}
		





/*****************************************************逻辑控制*********************************************************** */
typedef struct
{
	
   uint16_t zero_control_value; //零点控制值
	

   uint16_t relay_status; //继电器状态 00是档位A 01是档位B
	
   uint16_t zero_target_mv; //调零目标电压(mV)，由零点校准(0x20)更新，默认1259 ，影响调零和消磁的中点
	
		
   uint16_t a_zero_control_value; //A档位的零点控制值	(30A)
	 uint16_t b_zero_control_value; //b档位的零点控制值	(5A)
	
} logic_control_t;

logic_control_t  xdata logic_control={0};

typedef enum
{
   LOGIC_STATE_IDLE = 0,  //空闲
   LOGIC_STATE_ZERO_CONTROL = 1,  //零点控制
   LOGIC_STATE_DEFLUX = 2,  //消磁功能
   LOGIC_STATE_RELAY_SWITCH = 3,  //继电器切换
	
	 LOGIC_SET_ZERO = 4,  //调零操作	
	 LOGIC_ADC_VOL = 5,  //ADC_VOL超量程报警
	 LOGIC_ZERO_CAL = 6,  //零点校准
} logic_state_t;

logic_state_t xdata logic_state; //状态机执行状态
/*****************************************************逻辑控制*********************************************************** */








/*****************************************************flash实现*********************************************************** */
typedef struct
{
   logic_control_t mydata;
} flash_storage_t;

static uint8_t flashChecksum(uint8_t *buf, uint8_t len)
{
   uint8_t sum = 0;
   uint8_t i;
   for (i = 0; i < len; i++)
   {
      sum += buf[i];
   }
   return sum;
}

void saveLogicControl(void)
{
   flash_storage_t storage;
   uint8_t *p;
   uint16_t i;
   FLADDR addr = FLASH_STORAGE_ADDR;

   storage.mydata = logic_control;

   FLASH_PageErase(FLASH_STORAGE_ADDR);

   p = (uint8_t *)&storage;
   for (i = 0; i < sizeof(flash_storage_t); i++)
   {
      FLASH_ByteWrite(addr + i, p[i]);
   }
}

uint8_t readLogicControl(void)
{
   flash_storage_t storage;
   uint8_t *p;
   uint16_t i;
   FLADDR addr = FLASH_STORAGE_ADDR;

   p = (uint8_t *)&storage;
   for (i = 0; i < sizeof(flash_storage_t); i++)
   {
      p[i] = FLASH_ByteRead(addr + i);
   }

   logic_control = storage.mydata;
   return 1;
}

/*****************************************************flash实现*********************************************************** */
















/*****************************************************继电器控制*********************************************************** */

void RELAY_A(void)
{
   P2 |= 0x02;    // PY_A1 P2.1  输出高

   delay_ms(1);

   P2 &= ~0x04; //  RY_A0 = P2.2 输出低

   delay_ms(100);

   P2 |= 0x04;    // PY_A0 P2.2  输出高
}


void RELAY_B(void)
{
   P2 &= ~0x02;    // PY_A1 P2.1  输出低

   delay_ms(1);

    P2 &= ~0x04; //  RY_A0 = P2.2 输出低

   delay_ms(100);

   P2 |= 0x04;    // PY_A0 P2.2  输出高
}

/*****************************************************继电器控制*********************************************************** */






/*****************************************************新增 调零控制 *********************************************************** */
// 读取开关：1=开启（默认，一直读取）；0=关闭。后续如需协议命令控制，直接改这个变量即可
volatile uint8_t adc_zero_enable = 0;
uint16_t xdata adc_zero_value = 0;      // 最近一次 ADC 读数（14bit 右对齐，0~16383）

uint16_t xdata last_dac1_value = 0;      // 调零前的adc读数，失败后恢复这个读数


uint16_t xdata adc_zero_relay = 0;      // 调零前记录当前档位 0 档位A ， 1 档位B


// 闭环调节参数：电压误差死区(mV)，读数电压与目标 1259mV 差小于该值则不调节，防振荡
#define  ADC_CLOSE_DEADBAND_MV  4u

// ADC0 初始化：P2.6 配置为模拟输入，ADC0 软件触发模式（Timer2 已被消磁占用，故不用定时器触发）
void ADC_ZERO_Init(void)
{
   uint8_t sfrpage_save = SFRPAGE;

   // 1) P2.6 设为模拟输入（P2MDIN bit6=0），并从 crossbar 跳过（P2SKIP bit6=1）
   SFRPAGE = 0x20;
   P2MDIN &= (uint8_t)~0x40;   // P2.6 模拟模式
   P2SKIP |= 0x40;             // P2.6 跳过 crossbar

   // 2) ADC0 配置（SFRPAGE 0x00）
   SFRPAGE = 0x00;
   ADC0CN2 = ADC0CN2_ADCM__ADBUSY | ADC0CN2_PACEN__PAC_DISABLED;  // 写 ADBUSY=1 启动转换（软件触发）
   ADC0CN1 = ADC0CN1_ADBITS__14_BIT | ADC0CN1_ADSJST__RIGHT_NO_SHIFT | ADC0CN1_ADRPT__ACC_1;  // 14bit 右对齐单次
   ADC0MX  = ADC0MX_ADC0MX__ADC0P19;   // P2.6 = ADC0.19
   ADC0CF2 = ADC0CF2_GNDSL__GND_PIN | ADC0CF2_REFSL__VDD_PIN | (0x03 << ADC0CF2_ADPWR__SHIFT);  // VDD 参考
   ADC0CF0 = ADC0CF0_ADCLKSEL__SYSCLK | (0x01 << ADC0CF0_ADSC__SHIFT);
   ADC0CF1 = ADC0CF1_ADLPM__LP_DISABLED | (0x03 << ADC0CF1_ADTK__SHIFT);
   ADC0CN0 |= ADC0CN0_ADEN__ENABLED | ADC0CN0_IPOEN__POWER_DOWN;

   SFRPAGE = sfrpage_save;
}

// 软件触发读取一次 ADC0（P2.6），返回 14bit 结果
uint16_t ADC_ZERO_Read(void)
{
   uint8_t sfrpage_save = SFRPAGE;

   SFRPAGE = 0x00;
   ADC0CN0_ADINT = 0;          // 清转换完成标志
   ADC0CN0_ADBUSY = 1;         // 启动一次转换
   while (ADC0CN0_ADINT == 0); // 等待转换完成

   adc_zero_value = (uint16_t)(((uint16_t)ADC0H << 8) | ADC0L);
   SFRPAGE = sfrpage_save;
   return adc_zero_value;
}

// P2.4 电压读取：由协议 0x16 触发，连续读多次取平均，返回 mV。
// 注意：ADC0 单通道复用，读取完成后将 ADC0MX 恢复为 ADC0P19(P2.6)，避免影响 ADC_ZERO(调零)
uint16_t ADC_P24_Read_MV(void)
{
   uint8_t sfrpage_save = SFRPAGE;
   static uint8_t adc_p24_inited = 0;
   uint32_t sum = 0;
   uint16_t avg;
   uint8_t i;

   if (!adc_p24_inited)
   {
      // 1) P2.4 设为模拟输入（P2MDIN bit4=0），并从 crossbar 跳过（P2SKIP bit4=1）
      SFRPAGE = 0x20;
      P2MDIN &= (uint8_t)~0x10;   // P2.4 模拟模式
      P2SKIP |= 0x10;             // P2.4 跳过 crossbar

      // 2) ADC0 配置（VDD 参考、软件触发、14bit 右对齐）
      SFRPAGE = 0x00;
      ADC0CN2 = ADC0CN2_ADCM__ADBUSY | ADC0CN2_PACEN__PAC_DISABLED;
      ADC0CN1 = ADC0CN1_ADBITS__14_BIT | ADC0CN1_ADSJST__RIGHT_NO_SHIFT | ADC0CN1_ADRPT__ACC_1;
      ADC0CF2 = ADC0CF2_GNDSL__GND_PIN | ADC0CF2_REFSL__VDD_PIN | (0x03 << ADC0CF2_ADPWR__SHIFT);
      ADC0CF0 = ADC0CF0_ADCLKSEL__SYSCLK | (0x01 << ADC0CF0_ADSC__SHIFT);
      ADC0CF1 = ADC0CF1_ADLPM__LP_DISABLED | (0x03 << ADC0CF1_ADTK__SHIFT);
      ADC0CN0 |= ADC0CN0_ADEN__ENABLED | ADC0CN0_IPOEN__POWER_DOWN;
      adc_p24_inited = 1;
   }

   // 切换到 P2.4 通道
   SFRPAGE = 0x00;
   ADC0MX = ADC0MX_ADC0MX__ADC0P17;   // P2.4 = ADC0.17

   // 连续读 8 次取平均，滤除噪声
   for (i = 0; i < 8; i++)
   {
      ADC0CN0_ADINT = 0;          // 清转换完成标志
      ADC0CN0_ADBUSY = 1;         // 启动一次转换
      while (ADC0CN0_ADINT == 0); // 等待转换完成
      sum += (uint16_t)(((uint16_t)ADC0H << 8) | ADC0L);
   }
   avg = (uint16_t)(sum / 8);

   // 恢复通道为 P2.6（ADC_ZERO 调零使用），避免后续调零读取错误
   ADC0MX = ADC0MX_ADC0MX__ADC0P19;   // P2.6 = ADC0.19

   SFRPAGE = sfrpage_save;
   return (uint16_t)(((uint32_t)avg * 2500UL) / 16383UL);   // 返回电压 mV
}

/* ---- 电压越限阈值(mV)：P2.4 采样电压超出 [OVL_LOW_MIN, OVL_HIGH_MAX] 就主动上报 0x16 ---- */
/* ！！下面只是占位默认值，请按探头实际输出量程修改 ！！ */
#define OVL_LOW_MIN     200
#define OVL_HIGH_MAX    4090

/* 电压越限主动上报：每 1s 采一次 P2.4，超阈值就主动发一帧
   AA 16 02 hi lo chk   （和原来 0x16 查询的应答帧格式完全一致，数据是 mV）
   1) 只在空闲且消磁已结束时采样：避免打断调零的 P2.6 闭环，也避免消磁期间读数乱跳
   2) 越限期间每 1s 上报一次（持续告警） */
void ADC_OVL_Process(void)
{
   static uint32_t ovl_tick = 0;
   static uint8_t  ovl_inited = 0;
   uint16_t mv;
   unsigned char hi, lo, chk;

   /* 消磁/调零期间不采样，避免抢 ADC 通道 */
   if ((logic_state != LOGIC_STATE_IDLE) || deflux_busy())
   {
      return;
   }

   if (!ovl_inited)
   {
      ovl_inited = 1;
      ovl_tick = counter;
      return;
   }

   if ((counter - ovl_tick) < COUNTER_TICKS(1000u))     /* 1s 一次 */
   {
      return;
   }
   ovl_tick = counter;

   mv = ADC_P24_Read_MV();          /* 读 P2.4，函数内部已把 ADC0MX 还原到 P2.6 */

   if ((mv >= OVL_LOW_MIN) && (mv <= OVL_HIGH_MAX))
   {
      return;                        /* 在范围内，不上报 */
   }

   /* 越限：主动上报（帧格式与 0x16 查询应答一致） */
   hi  = (unsigned char)((mv >> 8) & 0xFF);
   lo  = (unsigned char)(mv & 0xFF);
   chk = (unsigned char)(0x100 - (ADC_VOL_TYPE + 0x02 + hi + lo));
   RETARGET_PRINTF("%c%c%c%c%c%c",
       FRAME_HEADER, ADC_VOL_TYPE, 0x02, hi, lo, chk);
}

//调零计数
uint16_t  set_zero_cnt = 0;
//成功计数
uint16_t xdata zero_OK_cnt = 0;	 
//调零目标电压（mV），默认1259，由零点校准(0x20)更新
uint16_t xdata zero_target_mv = 1259;
#define ZERO_SUCCESS_NUM 10


void delay_us(uint8_t us)
{
	 uint32_t i;
	 for (i = 0; i < us; i++)
   {

   }
}


uint8_t set_zero(void)
{
		//读取ADC数据
		// ---- ADC_ZERO 测试（新增）：开关默认开，一直读取并通过串口发送 ----
		if (adc_zero_enable)
		{
			
			 static uint8_t adc_zero_inited = 0;
			 if (!adc_zero_inited)
			 {
					ADC_ZERO_Init(); 
					adc_zero_inited = 1;
			 }
			 ADC_ZERO_Read();
			 
			 set_zero_cnt=set_zero_cnt+1;
		 
			 
			 
			 // ---- 闭环调节：ADC读数偏大 -> 减小DAC1，偏小 -> 增大DAC1，稳定在 target_adc_vlaue ----
			 {
					uint16_t now_dac1_code;
					uint16_t set_dac1_code;
					uint16_t now_mv;
					int16_t err_mv;
					uint16_t step;

					now_dac1_code = DAC1_GetCode();                 // 当前DAC1码值
					set_dac1_code = now_dac1_code;

					now_mv = (uint16_t)(((uint32_t)adc_zero_value * 2500UL) / 16383UL);   // 换算电压 mV

					err_mv = (int16_t)now_mv - (int16_t)zero_target_mv;   // 目标电压由零点校准(0x20)更新，默认1259
				 

				 
				 
				 
					if (err_mv > (int16_t)ADC_CLOSE_DEADBAND_MV)
					{
						 zero_OK_cnt = 0;               /* 出死区：连续命中计数清零 */
						 /* 电压偏大：减小DAC1 */
						 step = 1;
						 set_dac1_code -= step;
						 DAC1_SetCode(set_dac1_code);
					}
					else if (err_mv < -(int16_t)ADC_CLOSE_DEADBAND_MV)
					{
						 zero_OK_cnt = 0;               /* 出死区：连续命中计数清零 */
						 /* 电压偏小：增大DAC1 */
						 step = 1;
						 set_dac1_code += step;
						 DAC1_SetCode(set_dac1_code);
					}
					/* 死区内：不调节，防振荡 */
					else
					{
					   //成功次数增加
						zero_OK_cnt ++;
						if(zero_OK_cnt >= ZERO_SUCCESS_NUM)
						{
							zero_OK_cnt=0;
						  adc_zero_enable =0;
							adc_zero_inited = 0;
							return 1;  //成功
						}
					}
					

					if(set_zero_cnt >= 2000)
					{
							
						adc_zero_enable =0;
						zero_OK_cnt=0;
						adc_zero_inited = 0;
						set_zero_cnt =0;
						if(  (err_mv >= -((int16_t)ADC_CLOSE_DEADBAND_MV +1) )  &&  (err_mv <= ((int16_t)ADC_CLOSE_DEADBAND_MV +1)  )  )
						{
	
							return 1;  //成功
						}
						else
						{
							return 2;	 //失败
						}
						
						
					}
					
					delay_ms(1);
					
					
					
			 }

			 return 0;
		}
		return 0;	
}

/*****************************************************新增 调零控制 *********************************************************** */






/*****************************************************串口功能实现*********************************************************** */

// UART 接收 / 发送环形缓冲区（中断与主循环之间传递数据）
// 放到 XDATA，避免占用宝贵的内部 DATA RAM
#define UART_RX_RING_SIZE  64
#define UART_TX_RING_SIZE  64

static volatile unsigned char xdata uart_rx_ring[UART_RX_RING_SIZE];
static volatile unsigned char uart_rx_head = 0;   // 中断写
static volatile unsigned char uart_rx_tail = 0;   // 主循环读

static volatile unsigned char xdata uart_tx_ring[UART_TX_RING_SIZE];
static volatile unsigned char uart_tx_head = 0;   // 主循环写
static volatile unsigned char uart_tx_tail = 0;   // 中断读
static volatile bit  uart_tx_busy = 0;            // 1=正在发送中

/**
 * UART0 中断：收字节入 RX 环形缓冲；发字节从 TX 环形缓冲取出。
 * 这样即便 Timer2 消磁中断疯狂占用 CPU，串口收发也不会丢字节。
 */
SI_INTERRUPT (UART0_ISR, UART0_IRQn)
{
   uint8_t SFRPAGE_save = SFRPAGE;
   SFRPAGE = 0x00;

   if (SCON0 & 0x01)  // RI0
   {
      SCON0 &= ~0x01;  // clear RI0
      uart_rx_ring[uart_rx_head] = SBUF0;
      uart_rx_head = (unsigned char)((uart_rx_head + 1) % UART_RX_RING_SIZE);
   }

   if (SCON0 & 0x02)  // TI0
   {
      SCON0 &= ~0x02;  // clear TI0
      if (uart_tx_head != uart_tx_tail)
      {
         SBUF0 = uart_tx_ring[uart_tx_tail];
         uart_tx_tail = (unsigned char)((uart_tx_tail + 1) % UART_TX_RING_SIZE);
         uart_tx_busy = 1;
      }
      else
      {
         uart_tx_busy = 0;  // 缓冲区空，停止发送
      }
   }

   SFRPAGE = SFRPAGE_save;
}

unsigned char uart_getchar(void)
{
    unsigned char c1;
    while (uart_rx_head == uart_rx_tail);  // 等待缓冲区有数据
    c1 = uart_rx_ring[uart_rx_tail];
    uart_rx_tail = (unsigned char)((uart_rx_tail + 1) % UART_RX_RING_SIZE);
    return c1;
}

/**
 * putchar —— 重定向 printf 的输出，改为写入 TX 环形缓冲，由中断发送。
 */
char putchar(char c)
{
   bit need_start;
   // 等待 TX 缓冲区有空间
   while ((unsigned char)((uart_tx_head + 1) % UART_TX_RING_SIZE) == uart_tx_tail);

   IE_EA = 0;  // 关中断，保护临界区
   uart_tx_ring[uart_tx_head] = (unsigned char)c;
   uart_tx_head = (unsigned char)((uart_tx_head + 1) % UART_TX_RING_SIZE);
   need_start = !uart_tx_busy;
   if (need_start)
   {
      uart_tx_busy = 1;
   }
   IE_EA = 1;  // 开中断

   if (need_start)
   {
      SCON0 |= 0x02;  // 置 TI，触发一次发送中断，开始输出
   }
   return c;
}



typedef struct
{
   unsigned char data_value[UART_RX_BUFFER_SIZE];
   unsigned char index;
   //类型
   unsigned char type;
   //数据长度
   unsigned char data_length;
   //校验和
   unsigned char checksum;
   //一帧开始标志
   unsigned char frame_start_flag;
   //一帧完结标志
   unsigned char frame_end_flag;
} uart_rx_buffer_t;

uart_rx_buffer_t uart_rx_buffer={0};



/**
 * VerifyChecksum - 验证协议帧的"和校验"补码字节
 */
unsigned char VerifyChecksum(uart_rx_buffer_t *buf)
{
    unsigned char sum = 0;
    unsigned char i;

    sum += buf->type;
    sum += buf->data_length;
    for (i = 0; i < buf->data_length; i++) {
        sum += buf->data_value[3 + i];
    }
    sum += buf->checksum;

    return (sum == 0) ? 1 : 0;
}




void FrameDataAcquisition(void)
{
      unsigned char c;
      // 非阻塞：RX 缓冲无数据直接返回，避免主循环卡死在串口等待
      if (uart_rx_head == uart_rx_tail)
      {
         return;
      }
      // 改：一次处理完 RX 缓冲中已到的全部字节，防止持续请求时积压丢帧
      while (uart_rx_head != uart_rx_tail)
      {
         c = uart_getchar();

         if (c == FRAME_HEADER && uart_rx_buffer.frame_start_flag == 0)
         {
            uart_rx_buffer.frame_start_flag = 1;  //一帧开始
            //清零
            uart_rx_buffer.index = 0;
            for (uart_rx_buffer.index = 0; uart_rx_buffer.index < UART_RX_BUFFER_SIZE; uart_rx_buffer.index++)
            {
               uart_rx_buffer.data_value[uart_rx_buffer.index] = 0;
            }
            uart_rx_buffer.index = 0;
            //记录帧头，写入数据
            uart_rx_buffer.data_value[uart_rx_buffer.index++] = c;
         }
         else if (uart_rx_buffer.index < UART_RX_BUFFER_SIZE && uart_rx_buffer.frame_start_flag == 1)
         {
            // 重同步（改）：type 位置(index==1)收到新的帧头 AA -> 上一帧残缺，丢弃旧半帧以新 AA 重启
            if (c == FRAME_HEADER && uart_rx_buffer.index == 1)
            {
               continue;
            }
            uart_rx_buffer.data_value[uart_rx_buffer.index++] = c;

            //判断是否到类型
            if (uart_rx_buffer.index == 2)
            {
               uart_rx_buffer.type = c;
            }
            //判断是否到数据长度
            if (uart_rx_buffer.index == 3)
            {
               uart_rx_buffer.data_length = c;
               if (uart_rx_buffer.data_length > UART_RX_BUFFER_SIZE - 4) 
               {
                  uart_rx_buffer.frame_start_flag = 0;
                  uart_rx_buffer.index = 0;
               }
            }
            //判断一帧是否完结，因为数据格式是AA+（比如01）类型+ （比如02）数据长度 + 数据+ 校验和
            if (uart_rx_buffer.index == uart_rx_buffer.data_length + 4)
            {
               //校验和
               uart_rx_buffer.checksum = c; 
               //一帧完结
               uart_rx_buffer.frame_end_flag = 1;
            }
         }

         //拿一帧数据
         if (uart_rx_buffer.frame_end_flag == 1)
         {
            //处理一帧数据-逻辑在这
            if (VerifyChecksum(&uart_rx_buffer))  // 校验正确，做类型分发
            {
               if (uart_rx_buffer.type == ZERO_CONTROL_TYPE)
               {
                  /*对数据长度后面的数据进行拼接*/
                  logic_control.zero_control_value = ((uint16_t)uart_rx_buffer.data_value[3]) << 8  | (uart_rx_buffer.data_value[4]);
                  logic_state = LOGIC_STATE_ZERO_CONTROL;
               }
               else if (uart_rx_buffer.type == DEFLUX_TYPE)
               {
                  logic_state = LOGIC_STATE_DEFLUX;
								 
								 
								  //消磁是调零前的动作
								 	//调零前，记录下当前的档位，跳到档位A
									adc_zero_relay =  logic_control.relay_status;
								  
									//强制切换到档位A，30A档位
									RELAY_A();
								 
									//消磁时，细调零也就是DAC_ZERO要先设置为中点
									DAC1_SetCode(2048);

									deflux_wait_reply = 0;   /* 重新走一遍"启动消磁->等结束->回包" */
								  
								  
               }
               else if (uart_rx_buffer.type == RELAY_SWITCH_TYPE)
               {
                  logic_state = LOGIC_STATE_RELAY_SWITCH;
                  /*只有一个数据，即数据长度是1*/
                  logic_control.relay_status = ((uint16_t)uart_rx_buffer.data_value[3]);
               }
               else if (uart_rx_buffer.type == SET_ZERO_TYPE)
               {
                  logic_state = LOGIC_SET_ZERO;	
                  //记录下当前的adc数据，失败以后可以复原
                  last_dac1_value = DAC1_GetCode();
                  set_zero_cnt =0;
                  zero_OK_cnt =0;
								 
								 

								 
								 
               }
               else if (uart_rx_buffer.type == ADC_VOL_TYPE)
               {
                  logic_state = LOGIC_ADC_VOL;
               }
               else if (uart_rx_buffer.type == ZERO_CAL_TYPE)
               {
                  logic_state = LOGIC_ZERO_CAL;
               }							 
               else
               {
                  logic_state = LOGIC_STATE_IDLE;  //注意logic_state需要在完成后续的状态机后设置为空闲状态
                  // RETARGET_PRINTF("\r\n[ERROR] TYPE-ERROR\r\n");
               }
            } 
            else  // 校验错误
            {
               unsigned char chk;
               chk  = uart_rx_buffer.type;
               chk += 0x02;
               chk += 0xFF;
               chk += 0xFF;
               chk  = (unsigned char)(0x100 - chk);

               RETARGET_PRINTF("%c%c%c%c%c%c", //注意这个得加换行符
                   FRAME_HEADER,
                   uart_rx_buffer.type,
                   0x02,
                   0xFF,
                   0xFF,
                   chk);
            }

            //最终对数据清空
            uart_rx_buffer.frame_end_flag = 0;
            uart_rx_buffer.index = 0;
            uart_rx_buffer.type = 0;
            uart_rx_buffer.data_length = 0;
            uart_rx_buffer.checksum = 0;
            uart_rx_buffer.frame_start_flag = 0;
         }
      }
}
/*****************************************************串口功能实现*********************************************************** */


void main (void)
{

   enter_DefaultMode_from_RESET();
   DAC0_Init();
   DAC1_Init();

   IE_EA = 1; // Enable global interrupts
   IE_ES0 = 1; // Enable UART0 interrupt (收发均用中断+环形缓冲，防止消磁时丢字节)
   SFRPAGE = 0x00;
   SCON0 = 0x50; // UART mode1, REN=1 (TI 不再预置，发送由 putchar 触发中断)



   //flash初始化
   {
      uint8_t ret;
      ret = readLogicControl();
      // 调零目标初始化：Flash 无有效数据(0xFFFF/擦除态)或非法值时恢复默认1259
      if (logic_control.zero_target_mv == 0 || logic_control.zero_target_mv > 2500)
      {
         logic_control.zero_target_mv = 1259;
      }
      // 档位调零DAC值初始化：Flash旧数据/擦除态(0xFFFF)或非法值时恢复默认2048(中点)
      if (logic_control.a_zero_control_value == 0xFFFF || logic_control.a_zero_control_value > DAC1_MAX_CODE)
      {
         logic_control.a_zero_control_value = 2048;
      }
      if (logic_control.b_zero_control_value == 0xFFFF || logic_control.b_zero_control_value > DAC1_MAX_CODE)
      {
         logic_control.b_zero_control_value = 2048;
      }
      zero_target_mv = logic_control.zero_target_mv;   // 运行时目标与Flash同步
      deflux_set_mid_mv(zero_target_mv);   // DAC0(消磁)开机输出到保存的中点
      // RETARGET_PRINTF("\r\n[INIT_Flash_ok]\r\n");
      //打印信息
      // RETARGET_PRINTF("\r\n[INFO] logic_control.zero_control_value = %d, logic_control.relay_status = %d\r\n", logic_control.zero_control_value, logic_control.relay_status);
   }

   // dac数据初始化
   if (logic_control.zero_control_value <= DAC1_MAX_CODE)
   {
      DAC1_SetCode(logic_control.zero_control_value);
   }
   else
   {
      logic_control.zero_control_value = 0;
      DAC1_SetCode(0);
   }


   //继电器初始化
   {
      // 确保 P2.1/P2.2/P2.3 作为普通 GPIO
      P2SKIP |= 0x0E;   // bit1 + bit2 + bit3 = 0x02 + 0x04 + 0x08
      // 配置 P2.1 P2.2 P2.3 为推挽输出 (开漏则改为 P2MDIN & P2MDOUT 对应位置0)
      P2MDIN |= 0x0E;   // 数字输入使能
      P2MDOUT |= 0x0E;  // 推挽输出
      // P2.3 拉高
      P2 |= 0x08;
      //根据logic_control.relay_status判断是否开启继电器
      if (logic_control.relay_status == 0x00)
      {
         RELAY_A();
      }
      else
      {
         RELAY_B();
      }
   }

	 

	 
   while (1)
   {
			
      FrameDataAcquisition();  //串口接收数据
      ADC_OVL_Process();        //电压越限主动上报：每 1s 采一次 P2.4，超阈值主动发 0x16

      //根据逻辑状态处理数据
      switch (logic_state)
      {
				
				
				
         case LOGIC_STATE_IDLE:  //空闲状态
            break;

				 
				 
				 
//				 //零点控制状态===在两个板子联调中没有用到===是上位机控制的
//         case LOGIC_STATE_ZERO_CONTROL:  

//            if(logic_control.zero_control_value>=DAC1_MIN_CODE && logic_control.zero_control_value<=DAC1_MAX_CODE)
//            {
//               unsigned char hi, lo, chk;
//               DAC1_SetCode(logic_control.zero_control_value);
//               saveLogicControl();
//               // RETARGET_PRINTF("\r\n[INFO] zero_control_value = %d\r\n", logic_control.zero_control_value);

//               hi  = (unsigned char)((logic_control.zero_control_value >> 8) & 0xFF);
//               lo  = (unsigned char)(logic_control.zero_control_value & 0xFF);
//               chk = (unsigned char)(0x100 - (ZERO_CONTROL_TYPE + 0x02 + hi + lo));
//               RETARGET_PRINTF("%c%c%c%c%c%c",
//                   FRAME_HEADER, ZERO_CONTROL_TYPE, 0x02, hi, lo, chk);
//            }
//            else
//            {
//               RETARGET_PRINTF("\r\n[ERROR] ZERO_CONTROL_VALUE_ERROR\r\n");
//            }
//            logic_state = LOGIC_STATE_IDLE; 
//            break;
//						
						
						
						
						

						
						
				 //控制继电器的状态	
						
         case LOGIC_STATE_RELAY_SWITCH:  
            /*在这里写继电器的逻辑，也就是控制继电器的开闭*/
            if (logic_control.relay_status == 0x00)
            {
               unsigned char chk;
               RELAY_A();
               saveLogicControl();

							//也要更改DAC_ZERO的值，更改为对应档位A
							DAC1_SetCode(logic_control.a_zero_control_value);
							
               chk = (unsigned char)(0x100 - (RELAY_SWITCH_TYPE + 0x01 + 0x00));
               RETARGET_PRINTF("%c%c%c%c%c",
                   FRAME_HEADER, RELAY_SWITCH_TYPE, 0x01, 0x00, chk);
            }
            else if (logic_control.relay_status == 0x01)
            {
               unsigned char chk;
               RELAY_B();
               saveLogicControl();

							//也要更改DAC_ZERO的值，更改为对应档位B
							DAC1_SetCode(logic_control.b_zero_control_value);
							
               chk = (unsigned char)(0x100 - (RELAY_SWITCH_TYPE + 0x01 + 0x01));
               RETARGET_PRINTF("%c%c%c%c%c",
                   FRAME_HEADER, RELAY_SWITCH_TYPE, 0x01, 0x01, chk);
            }
            else
            {
               /*发送失败的帧*/
               RETARGET_PRINTF("\r\n[ERROR] RELAY_SWITCH_FAILED\r\n");
            }
            logic_state = LOGIC_STATE_IDLE; 
            break;
						
						
						
						
         //消磁状态：启动消磁 -> 等 Timer2 中断里的正弦真正跑完 -> 再回包
         case LOGIC_STATE_DEFLUX:
         {
            unsigned char chk;

            if (deflux_wait_reply == 0)
            {
               /* 第一次进来：把消磁挂起来，之后每圈回来看是否跑完 */
               if (!deflux_busy())          /* 已经在消磁就不要重复触发 */
               {
                  deflux_trigger = 1;
               }
               deflux_start_tick = counter;
               deflux_wait_reply = 1;
               break;                       /* 下一圈再判断是否结束 */
            }

            /* deflux_trigger 被中断取走后 deflux_running=1，跑完自动清 0；
               两个都为 0 才算真正结束；超过 30s 兜底放行，防止中断异常时永远等下去 */
            if ((deflux_trigger != 0u) || deflux_busy())
            {
               if ((counter - deflux_start_tick) < DEFLUX_TIMEOUT_TICK)
               {
                  break;                    /* 还在消磁，继续等 */
               }
            }

            deflux_wait_reply = 0;
            logic_state = LOGIC_STATE_IDLE;

            /* 消磁真正结束才回包：上位机收到这帧就代表可以开始调零，不用再死等固定秒数 */
            chk = (unsigned char)(0x100 - (DEFLUX_TYPE + 0x00));
            RETARGET_PRINTF("%c%c%c%c%c",
                FRAME_HEADER, DEFLUX_TYPE, 0x00, chk);
            break;
         }
												
						
						
						
						
						//调零操作----上位机会先发消磁，再发调零
						
						/*消磁操作必须在（30A）A档位*/
						/*
						
							  逻辑是
								记录当前档位
						
						    先切换到（30A）A档位，消磁，调零，记录A档位下的DAC_ZERO的值，切换到（5A）B档位，执行调零，记录下B档位下DAC_ZERO的值
						
								恢复到之前记录的档位
						*/	
					 case LOGIC_SET_ZERO: 
					 {
							uint8_t zero_30A_ret; //调零返回值
							uint8_t zero_5A_ret;

							/* 消磁还没结束就先等着：不然读到的零点还在被正弦扫动，调零必然调不准 */
							if (deflux_busy() || (deflux_trigger != 0u))
							{
							   if (zero_wait_started == 0)
							   {
							      zero_wait_started = 1;
							      zero_wait_tick = counter;
							   }
							   if ((counter - zero_wait_tick) < DEFLUX_TIMEOUT_TICK)
							   {
							      break;                     /* 等消磁结束再来调零，最多等 30s 兜底 */
							   }
							}
							zero_wait_started = 0;
							adc_zero_enable = 1;
						 
							do
							{
								zero_30A_ret = set_zero();
							} while (zero_30A_ret == 0);   // 循环调零直到成功(1)或失败(2)

							
							if(zero_30A_ret ==1 )       //调零30A档位成功
							{
		
								//保存数据
								logic_control.zero_control_value = DAC1_GetCode();
								//到了这里才完成30A档位的消磁和调零操作，此时记录DAC_ZERO的值
								logic_control.a_zero_control_value = logic_control.zero_control_value;

								
								RELAY_B();
								
								
								
									//5A档再次调零：set_zero() 返回 1=成功 2=失败(超时)，失败则恢复原档位、档位DAC_ZERO并回失败帧
									

										adc_zero_enable = 1;   // 30A成功后已置0，5A重新使能
										do
										{
											zero_5A_ret = set_zero();
										} while (zero_5A_ret == 0);   // 循环调零直到成功(1)或失败(2)
										
										
										if (zero_5A_ret == 1)
										{
												//调零完成记录下5A的DAC_ZERO值，然后进行恢复档位
												logic_control.zero_control_value = DAC1_GetCode();
												logic_control.b_zero_control_value = logic_control.zero_control_value;		
													
												//恢复档位
												if(adc_zero_relay == 0)
												{
													RELAY_A();//
													DAC1_SetCode(logic_control.a_zero_control_value);
												}
												else if(adc_zero_relay == 1)
												{
													RELAY_B();//
													DAC1_SetCode(logic_control.b_zero_control_value);
												}
												
												//保存到flash
												saveLogicControl();
												
												//发送成功信息
												logic_state = LOGIC_STATE_IDLE; 
												{
												 unsigned char chk;
												 chk = (unsigned char)(0x100 - (SET_ZERO_TYPE + 0x01 + 0x01));
												 RETARGET_PRINTF("%c%c%c%c%c",
														 FRAME_HEADER, SET_ZERO_TYPE, 0x01 ,0x01, chk);
												}		
										}
										else if (zero_5A_ret == 2) //第二次失败
										{
											if (adc_zero_relay == 0)
											{
												RELAY_A();
												DAC1_SetCode(logic_control.a_zero_control_value);
											}
											else
											{
												RELAY_B();
												DAC1_SetCode(logic_control.b_zero_control_value);
											}
											saveLogicControl();   // 保留30A已调好的值
											logic_state = LOGIC_STATE_IDLE;
											{
												unsigned char chk;
												chk = (unsigned char)(0x100 - (SET_ZERO_TYPE + 0x01 + 0x09));
												RETARGET_PRINTF("%c%c%c%c%c",
												FRAME_HEADER, SET_ZERO_TYPE, 0x01, 0x09, chk);  //第二次失败发送09
											}
										}
										
										
									
								 //5A档再次调零：set_zero() 返回 1=成功 2=失败(超时)，失败则恢复原档位、档位DAC_ZERO并回失败帧
									

					
								
								
								
								
							}


							
							else if(zero_30A_ret ==2 ) //第一次失败是发送 00 
							{
								
								//保存数据，调零失败也要保存，而不是恢复成上一次的值
								logic_control.zero_control_value = DAC1_GetCode();
								saveLogicControl();						
								
								logic_state = LOGIC_STATE_IDLE; 
								{
								 unsigned char chk;
								 chk = (unsigned char)(0x100 - (SET_ZERO_TYPE + 0x01 + 0x00));
								 RETARGET_PRINTF("%c%c%c%c%c",
										 FRAME_HEADER, SET_ZERO_TYPE, 0x01 ,0x00, chk);
								}		
							}
							
					}
          break;						
						
						

				 
					 
				 

				 
				 //零点校准的操作
				 
				 case LOGIC_ZERO_CAL:  
				 {	 
				 	 //零点校准：采样 ADC_ZERO(P2.6)，电压在 1259mV±200mV 内 -> 成功（并把采样值设为调零目标）；否则失败
				 	 static uint8_t zero_cal_inited = 0;
				 	 uint16_t mv;
				 	 if (!zero_cal_inited)
				 	 {
				 		 ADC_ZERO_Init();
				 		 zero_cal_inited = 1;
				 	 }
				 	 ADC_ZERO_Read();
				 	 mv = (uint16_t)(((uint32_t)adc_zero_value * 2500UL) / 16383UL);   // 换算电压 mV
					 
//						// 打印原始码值(14bit) + 换算电压(mV)，VDD=2.5V 参考
//						RETARGET_PRINTF("ADC_ZERO=%u   %umV \r\n",
//						adc_zero_value,
//						(uint16_t)(((uint32_t)adc_zero_value * 2500UL) / 16383UL));

					 		 			 RETARGET_PRINTF("%c%c%c%c%c",  FRAME_HEADER, ZERO_CAL_TYPE, 0x01, 0x01, 0x01);		
					 
				 	 if (mv >= (uint16_t)(1259u - 500u) && mv <= (uint16_t)(1259u + 500u))
				 	 {
				 		 //成功：将该采样值作为新的调零目标，set_zero 按 zero_target_mv 闭环
				 		 zero_target_mv = mv;
				 		 logic_control.zero_target_mv = mv;   // 持久化到Flash
				 		 saveLogicControl();
				 		 deflux_set_mid_mv(zero_target_mv);   // 消磁中点同步为校准值
				 		 logic_state = LOGIC_STATE_IDLE;
				 		 {
				 			 unsigned char chk;
				 			 chk = (unsigned char)(0x100 - (ZERO_CAL_TYPE + 0x01 + 0x01));
				 			 RETARGET_PRINTF("%c%c%c%c%c",
				 				 FRAME_HEADER, ZERO_CAL_TYPE, 0x01, 0x01, chk);
							 zero_cal_inited = 0;
				 		 }
				 	 }
				 	 else
				 	 {
				 		 //失败
				 		 logic_state = LOGIC_STATE_IDLE;
				 		 {
				 			 unsigned char chk;
				 			 chk = (unsigned char)(0x100 - (ZERO_CAL_TYPE + 0x01 + 0x00));
				 			 RETARGET_PRINTF("%c%c%c%c%c",
				 				 FRAME_HEADER, ZERO_CAL_TYPE, 0x01, 0x00, chk);
							 zero_cal_inited = 0;
				 		 }
				 	 }
				 }
				 break;					 
			

					
					
					
//					//ADC_VOL 电压上报（上位机 1s 请求一次）
//				 case LOGIC_ADC_VOL:  
//				 {
//						unsigned char hi, lo, chk;
//						uint16_t mv = ADC_P24_Read_MV();   // 多读平均返回 mV，函数内已恢复 ADC0MX 为 P2.6
//						hi = (unsigned char)((mv >> 8) & 0xFF);
//						lo = (unsigned char)(mv & 0xFF);
//						chk = (unsigned char)(0x100 - (ADC_VOL_TYPE + 0x02 + hi + lo));
//						RETARGET_PRINTF("%c%c%c%c%c%c",
//								FRAME_HEADER, ADC_VOL_TYPE, 0x02, hi, lo, chk);
//						logic_state = LOGIC_STATE_IDLE;
//				 }
//				 break;
//				 
//				 //后续在这里增加上报ADC_ZERO的

				 
					 
         
					 
      }

   }
}