
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
      }

   }
}