
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
		
/*****************************************************宏定义部分*********************************************************** */		
		


/*****************************************************外部声明部分*********************************************************** */		

extern volatile uint32_t counter ;//延时函数-中断里面
extern volatile uint8_t deflux_trigger ;   // 消磁的逻辑，设置为1则行一次-中断里面
/*****************************************************外部声明部分*********************************************************** */		




/*****************************************************延时函数*********************************************************** */		
uint32_t start_counter =0;
void delay_ms(uint16_t ms)
{
	start_counter = counter;
	while((counter - start_counter ) < (2*ms))
	{
	};
}
		





/*****************************************************逻辑控制*********************************************************** */
typedef struct
{
   uint16_t zero_control_value; //零点控制值
   uint16_t relay_status; //继电器状态 00是档位A 01是档位B
} logic_control_t;

logic_control_t logic_control={0};

typedef enum
{
   LOGIC_STATE_IDLE = 0,  //空闲
   LOGIC_STATE_ZERO_CONTROL = 1,  //零点控制
   LOGIC_STATE_DEFLUX = 2,  //消磁功能
   LOGIC_STATE_RELAY_SWITCH = 3,  //继电器切换
	
	 LOGIC_SET_ZERO = 4,  //调零操作	
	 LOGIC_ADC_VOL = 5,  //ADC_VOL超量程报警
	
} logic_state_t;

logic_state_t logic_state; //状态机执行状态
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
uint16_t adc_zero_value = 0;      // 最近一次 ADC 读数（14bit 右对齐，0~16383）

uint16_t last_dac1_value = 0;      // 调零前的adc读数，失败后恢复这个读数

// 闭环调节参数：电压误差死区(mV)，读数电压与目标 1259mV 差小于该值则不调节，防振荡
#define ADC_CLOSE_DEADBAND_MV  4u

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

//调零计数
uint16_t set_zero_cnt = 0;
//成功计数
uint16_t zero_OK_cnt = 0;	 
#define ZERO_SUCCESS_NUM 100

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

					err_mv = (int16_t)now_mv - 1259;   // 正=电压偏大
				 
					if (err_mv > (int16_t)ADC_CLOSE_DEADBAND_MV)
					{
						 /* 电压偏大：减小DAC1 */
						 step = 1;
						 set_dac1_code -= step;
						 DAC1_SetCode(set_dac1_code);
					}
					else if (err_mv < -(int16_t)ADC_CLOSE_DEADBAND_MV)
					{
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
							set_zero_cnt = 0;
						  adc_zero_enable =0;
							return 1;  //成功
						}
					}
					
					set_zero_cnt=set_zero_cnt+1;
//					if(set_zero_cnt % 1000 == 0)
//					{
//						// 打印原始码值(14bit) + 换算电压(mV)，VDD=2.5V 参考
//						RETARGET_PRINTF("DAC1 code=%u    ADC_ZERO=%u   %umV \r\n",
//						now_dac1_code,
//						adc_zero_value,
//						(uint16_t)(((uint32_t)adc_zero_value * 2500UL) / 16383UL));
//					}
					
					
					//看看3s内是否可以调节完成，可以的话发送信息
					if(set_zero_cnt % 3000 == 0)
					{
						set_zero_cnt = 0;
						adc_zero_enable =0;
						zero_OK_cnt=0;
						if(  (err_mv >= -((int16_t)ADC_CLOSE_DEADBAND_MV +1) )  &&  (err_mv <= ((int16_t)ADC_CLOSE_DEADBAND_MV +1)  )  )
							return 1;  //成功
						else
							return 2;	 //失败
					}
			 }
			 delay_ms(1);
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
 * CalcChecksum - 计算协议帧的"和校验"补码字节
 */
unsigned char CalcChecksum(unsigned char type, unsigned char len, unsigned char *data_buf)
{
    unsigned char sum = 0;
    unsigned char i;

    sum += type;
    sum += len;
    for (i = 0; i < len; i++) {
        sum += data_buf[i];
    }

    return (unsigned char)(0x100 - sum);
}

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

      //根据逻辑状态处理数据
      switch (logic_state)
      {
				
				
				
         case LOGIC_STATE_IDLE:  //空闲状态
            break;

				 
         case LOGIC_STATE_ZERO_CONTROL:  //零点控制状态

            if(logic_control.zero_control_value>=DAC1_MIN_CODE && logic_control.zero_control_value<=DAC1_MAX_CODE)
            {
               unsigned char hi, lo, chk;
               DAC1_SetCode(logic_control.zero_control_value);
               saveLogicControl();
               // RETARGET_PRINTF("\r\n[INFO] zero_control_value = %d\r\n", logic_control.zero_control_value);

               hi  = (unsigned char)((logic_control.zero_control_value >> 8) & 0xFF);
               lo  = (unsigned char)(logic_control.zero_control_value & 0xFF);
               chk = (unsigned char)(0x100 - (ZERO_CONTROL_TYPE + 0x02 + hi + lo));
               RETARGET_PRINTF("%c%c%c%c%c%c",
                   FRAME_HEADER, ZERO_CONTROL_TYPE, 0x02, hi, lo, chk);
            }
            else
            {
               RETARGET_PRINTF("\r\n[ERROR] ZERO_CONTROL_VALUE_ERROR\r\n");
            }
            logic_state = LOGIC_STATE_IDLE; 
            break;
						
						
						
         case LOGIC_STATE_DEFLUX:  //消磁状态
            deflux_trigger = 1;
            // RETARGET_PRINTF("\r\n[INFO] DEFLUX_DONE\r\n");
            {
               unsigned char chk;
               chk = (unsigned char)(0x100 - (DEFLUX_TYPE + 0x00));
               RETARGET_PRINTF("%c%c%c%c",
                   FRAME_HEADER, DEFLUX_TYPE, 0x00, chk);
            }
            logic_state = LOGIC_STATE_IDLE;
            break;
						
						
						
         case LOGIC_STATE_RELAY_SWITCH:  //继电器状态
            /*在这里写继电器的逻辑，也就是控制继电器的开闭*/
            if (logic_control.relay_status == 0x00)
            {
               unsigned char chk;
               RELAY_A();
               saveLogicControl();
               /*发送成功的帧*/
               // RETARGET_PRINTF("\r\n[INFO] relay_status = %d\r\n", logic_control.relay_status);
               // RETARGET_PRINTF("\r\n[INFO] RELAY_A\r\n");
               chk = (unsigned char)(0x100 - (RELAY_SWITCH_TYPE + 0x01 + 0x00));
               RETARGET_PRINTF("%c%c%c%c%c",
                   FRAME_HEADER, RELAY_SWITCH_TYPE, 0x01, 0x00, chk);
            }
            else if (logic_control.relay_status == 0x01)
            {
               unsigned char chk;
               RELAY_B();
               saveLogicControl();
               /*发送成功的帧*/
               // RETARGET_PRINTF("\r\n[INFO] relay_status = %d\r\n", logic_control.relay_status);
               // RETARGET_PRINTF("\r\n[INFO] RELAY_B\r\n");
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
						
						
						
					 case LOGIC_SET_ZERO:  //调零操作
					 {
							uint8_t zero_ret; //调零返回值
							adc_zero_enable = 1;
							zero_ret = set_zero();
							if(zero_ret ==1 )      //发送成功
							{
								//保存数据
								logic_control.zero_control_value = DAC1_GetCode();
								saveLogicControl();
								
								logic_state = LOGIC_STATE_IDLE; 
								{
								 unsigned char chk;
								 chk = (unsigned char)(0x100 - (SET_ZERO_TYPE + 0x01 + 0x01));
								 RETARGET_PRINTF("%c%c%c%c%c",
										 FRAME_HEADER, SET_ZERO_TYPE, 0x01 ,0x01, chk);
								}		
							}
							else if(zero_ret ==2 ) //发送失败
							{
							
								//保存数据
								logic_control.zero_control_value = last_dac1_value;
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
						
						
					 case LOGIC_ADC_VOL:  //ADC_VOL 电压上报（上位机 1s 请求一次）
					 {
							unsigned char hi, lo, chk;
							uint16_t mv = ADC_P24_Read_MV();   // 多读平均返回 mV，函数内已恢复 ADC0MX 为 P2.6
							hi = (unsigned char)((mv >> 8) & 0xFF);
							lo = (unsigned char)(mv & 0xFF);
							chk = (unsigned char)(0x100 - (ADC_VOL_TYPE + 0x02 + hi + lo));
							RETARGET_PRINTF("%c%c%c%c%c%c",
									FRAME_HEADER, ADC_VOL_TYPE, 0x02, hi, lo, chk);
							logic_state = LOGIC_STATE_IDLE;
					 }
					 break;
      }

   }
}