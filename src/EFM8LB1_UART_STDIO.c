
#include <SI_EFM8LB1_Register_Enums.h>
#include "retargetserial.h"
#include "InitDevice.h"
#include "EFM8LB1_FlashPrimitives.h"
#include "EFM8LB1_FlashUtils.h"
#include "string.h"

// EFM8LB10F16 是 16KB Flash 设备，最后一页地址为 0x3E00（页大小512字节）
// 这里使用倒数第1页的起始地址存放用户数据
#define FLASH_STORAGE_ADDR  0x3E00
#define FLASH_STORAGE_MAGIC 0xA5A5

/*延时函数*/
extern volatile uint32_t counter ;
uint32_t start_counter =0;
void delay_ms(uint16_t ms)
{
	// start_counter = counter;
	
	// while((counter - start_counter ) < (2*ms))
	// {
		
	// };
	
}

/***********************************逻辑控制*************************************/



typedef struct
{


   uint16_t zero_control_value; //零点控制值
   uint8_t relay_status; //继电器状态 00是档位A 01是档位B


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

/***********************************逻辑控制*************************************/









/*************************************flash*********************************** */

typedef struct
{
   uint16_t magic;
   logic_control_t mydata;
   uint8_t checksum;
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

   storage.magic = FLASH_STORAGE_MAGIC;
   storage.mydata = logic_control;
   storage.checksum = flashChecksum((uint8_t *)&storage.mydata, sizeof(logic_control_t));

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
   uint8_t calc_chk;
   FLADDR addr = FLASH_STORAGE_ADDR;

   p = (uint8_t *)&storage;
   for (i = 0; i < sizeof(flash_storage_t); i++)
   {
      p[i] = FLASH_ByteRead(addr + i);
   }

   if (storage.magic != FLASH_STORAGE_MAGIC)
   {
      return 0;
   }

   calc_chk = flashChecksum((uint8_t *)&storage.mydata, sizeof(logic_control_t));
   if (storage.checksum != calc_chk)
   {
      return 0;
   }

   logic_control = storage.mydata;
   return 1;
}

/*************************************flash*********************************** */
















/***********************************继电器控制*************************************/

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




/***********************************继电器控制*************************************/












/***********************************串口功能实现*************************************/


void SiLabs_Startup (void)
{
}

unsigned char uart_getchar(void)
{
    unsigned char c1;
    while ((SCON0 & 0x01) == 0); // wait for RI0
    c1 = SBUF0;
    SCON0 &= ~0x01; // clear RI0
    return c1;
}

#define UART_RX_BUFFER_SIZE 20
//定义帧头
#define FRAME_HEADER 0xAA
//定义零点控制的类型
#define ZERO_CONTROL_TYPE 0xA0
//定义消磁功能的类型
#define DEFLUX_TYPE 0xC0
//定义继电器切换的类型
#define RELAY_SWITCH_TYPE 0xD0


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
         // RETARGET_PRINTF("\r\nOK\r\n");
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
               RETARGET_PRINTF("\r\n[ERROR] NUM-ERROR\r\n");
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
               logic_control.relay_status = uart_rx_buffer.data_value[3];
            }
            else
            {
               logic_state = LOGIC_STATE_IDLE;  //注意logic_state需要在完成后续的状态机后设置为空闲状态
               RETARGET_PRINTF("\r\n[ERROR] TYPE-ERROR\r\n");
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

           RETARGET_PRINTF("\r\n%c%c%c%c%c%c\r\n", //注意这个得加换行符
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

/***********************************串口功能实现*************************************/







void main (void)
{

   enter_DefaultMode_from_RESET();

   IE_EA = 1;                          // Enable global interrupts

//   while (1) {}
	
	
	
	
	
   SFRPAGE = 0x00;
   SCON0 = 0x50; // UART mode
   SCON0 |= 0x02; // set TI (transmit ready)


   /*取flash的数据*/
   {
      // uint8_t ret;
      // ret = readLogicControl();
      // if (ret == 0)
      // {
      //    logic_control.zero_control_value = 0;
      //    logic_control.relay_status = 0;
      //    saveLogicControl();
      //    RETARGET_PRINTF("\r\n[INIT_Flash_ok]\r\n");
      //    //打印信息
      //    RETARGET_PRINTF("\r\n[INFO] logic_control.zero_control_value = %d, logic_control.relay_status = %d\r\n", logic_control.zero_control_value, logic_control.relay_status);


      // }
      // else
      //    RETARGET_PRINTF("\r\n[INIT_Flash_error]\r\n");
   }

   /*上电对DAC即P2.6进行初始化，输出之前flash保存的电压*/



   /*上电对继电器进行初始化，关闭继电器，逻辑为RY_A1是P2.1 ， RY_A0 是P2.2 */
   {

      // 确保 P2.1/P2.2 作为普通 GPIO
      P2SKIP |= 0x06;   // bit1 + bit2 = 0x02 + 0x04

      // 配置 P2.1 P2.2 为推挽输出 (开漏则改为 P2MDIN & P2MDOUT 对应位置0)
      P2MDIN |= 0x06;   // 数字输入使能
      P2MDOUT |= 0x06;  // 推挽输出

  
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

            /*在这里写零点控制的逻辑，也就是控制DAC即P2.6，输出对应0-4096的电压*/

            saveLogicControl();
            logic_state = LOGIC_STATE_IDLE; 
            break;
         case LOGIC_STATE_DEFLUX:  //消磁状态

            /*在这里写消磁的逻辑，也就是控制DAC即P30，输出可变幅度的正弦波
              正弦波的逻辑是这样子的，首先先输出满摆幅，即以1/2 4096为零点，上下满幅摆动的正弦波，满摆时间为500ms
              然后以渐摆1.5S的时间到1/2 4096，正弦波的频率为300Hz
            */

            logic_state = LOGIC_STATE_IDLE; 
            break;
         case LOGIC_STATE_RELAY_SWITCH:  //继电器状态

            /*在这里写继电器的逻辑，也就是控制继电器的开闭*/
            if (logic_control.relay_status == 0x00)
            {
               RELAY_A();
               saveLogicControl();
               /*发送成功的帧*/
            }
            else if (logic_control.relay_status == 0x01)
            {
               RELAY_B();
               saveLogicControl();
               /*发送成功的帧*/
            }
            else
            {
               /*发送失败的帧*/
            }
            logic_state = LOGIC_STATE_IDLE; 
            break;
      }

   }
}