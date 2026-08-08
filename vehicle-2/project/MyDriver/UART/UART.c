#include "UART.h"
#include "track.h"
#include "question.h"
#include <stdarg.h>
#include <stdio.h>

char Tx_User_Buf[128];
char Rx_User_Buf[128];
uint8_t ucUART_User_Pos = 0;
uint8_t UART_User_Flag = 0;
uint64_t UART_User_Time = 0;

char Tx_K230_Buf[4] = {0x50,0,0,0x51};
char Rx_K230_Buf[30];
uint8_t ucUART_K230_Pos = 0;
uint8_t UART_K230_Flag = 0;
uint64_t UART_K230_Time = 0;

char Tx_Zigbee_Buf[8] = {0x18,0,0,0,0x19};
char Rx_Zigbee_Buf[30];
uint8_t ucUART_Zigbee_Pos = 0;
uint8_t UART_Zigbee_Flag = 0;
uint64_t UART_Zigbee_Time = 0;


//Cmd 摄像头对单片机的命令
uint8_t Cmd;




//Founction
void UART_User_Proc(void);
void UART_K230_Proc(void);


//========================Debug部分串口=================
void UART2_Send_Bytes(char *buf, int len)
{
  while(len--)
  {
    DL_UART_Main_transmitDataBlocking(UART2_User_INST, *buf);
    buf++;
  }
}


void UART2_SendString(char *str)
{
    while (*str != '\0')  
    {
        DL_UART_Main_transmitDataBlocking(UART2_User_INST, *str++);
    }
}


// 自制的my_printf函数
int my_printf(const char *format, ...)
{
    va_list args;
    int ret;
    
    // 初始化可变参数列表
    va_start(args, format);
    
    // 使用vsprintf将格式化字符串写入缓冲区
    ret = vsprintf(Tx_User_Buf, format, args);
    
    // 结束可变参数列表
    va_end(args);
    
    // 检查是否超出缓冲区大小
    if (ret > 0 && ret < 128)
    {
        // 通过UART2发送格式化后的字符串
        UART2_SendString(Tx_User_Buf);
    }
    
    return ret;
}



uint8_t R[4] = {0x50,0x11,0x62,0x51};

void UART2_User_INST_IRQHandler(void) 
{
	switch(DL_UART_getPendingInterrupt(UART2_User_INST) )
    {
		case DL_UART_IIDX_RX:
			Rx_User_Buf[ucUART_User_Pos++] = DL_UART_Main_receiveData(UART2_User_INST);
			UART_User_Flag = 1;
			
		break;
		default:
			
		break;
		
		
	}
}


//发送数据包(0x50,data1,data2,0x51)
//========================Zigbee部分串口=================
void UART0_Send_Bytes(char *buf, int len)
{
  while(len--)
  {
    DL_UART_Main_transmitDataBlocking(UART0_Zigbee_INST, *buf);
    buf++;
  }
}


void UART0_SendString(char *str)
{
    while (*str != '\0')  
    {
        DL_UART_Main_transmitDataBlocking(UART0_Zigbee_INST, *str++);
    }
}

void UART0_Zigbee_INST_IRQHandler(void) 
{
	switch(DL_UART_getPendingInterrupt(UART0_Zigbee_INST) )
    {
		case DL_UART_IIDX_RX:
			Rx_Zigbee_Buf[ucUART_Zigbee_Pos++] = DL_UART_Main_receiveData(UART0_Zigbee_INST);
			UART_Zigbee_Flag = 1;
			
		break;
		default:
			
		break;
		
		
	}
}

//发送数据包(0x18,data1,data2,0x19)
void Zigbee_Send_Bytes(char buf1,char buf2)
{
	Tx_Zigbee_Buf[1] = buf1;
	Tx_Zigbee_Buf[2] = buf2;
	
	UART0_Send_Bytes(Tx_Zigbee_Buf,4);
}

void UART_Zigbee_Proc(void)
{
	static uint8_t state   = 0;      /* 0:WAIT_HEAD 1:PAYLOAD 2:WAIT_TAIL */
    static uint8_t header  = 0;
    static uint8_t payload[8];
    static uint8_t idx     = 0;
    static uint8_t need    = 0;

    if (!UART_Zigbee_Flag) return;     /* 无新数据 */
    UART_Zigbee_Flag = 0;

    for (uint8_t i = 0; i < ucUART_Zigbee_Pos; ++i)
    {
        uint8_t ch = (uint8_t)Rx_Zigbee_Buf[i];

        switch (state)
        {
        /* 1. 等待包头 --------------------------------------------------*/
        case 0:
            switch (ch)
            {
				case 0x18: header = ch; need = 2;  state = 1; break; /* Cmd */
		
				default:  /* 垃圾字节，忽略 */ break;
            }
            idx = 0;                         /* 新帧重新计数 */
        break;

        /* 2. 收集负载 --------------------------------------------------*/
        case 1:
            if (idx < sizeof(payload))
                payload[idx++] = ch;
            if (idx >= need)                 /* 足够长度即可转尾判 */
                state = 2;
        break;

        /* 3. 校验尾字节 ------------------------------------------------*/
        case 2:
            if ((header == 0x18 && ch == 0x19))
            {
				if(payload[0] == 0x10 && payload[1] == 0x10)
				{
					Q_Num = 1;
					Q_State = 1;
				}
				else if(payload[0] == 0x1A && payload[1] == 0x1A)
				{
					Q1_Stop = 1;
				}
				else if(payload[0] == 0x20 && payload[1] == 0x20)
				{
					Q_Num = 2;
					Q_State = 1;
				}
				else if(payload[0] == 0x2A && payload[1] == 0x2A)
				{
					Q2_Stop = 1;
				}
				else if(payload[0] == 0x30 && payload[1] == 0x30)
				{
					Q_Num = 3;
					Q_State = 1;
				}
				else if(payload[0] == 0x3A && payload[1] == 0x3A)
				{
					Q3_Stop = 1;
				}
				else if(payload[0] == 0x40 && payload[1] == 0x40)
				{
					Q_Num = 4;
					Q_State = 1;
				}
				else if(payload[0] == 0x4A && payload[1] == 0x4A)
				{
					Q4_Stop_Aid = 1;
				}
				else if(payload[0] == 0x4B && payload[1] == 0x4B)
				{
					Q4_Stop_Aid = 0;
				}
				else if(payload[0] == 0x4C && payload[1] == 0x4C)
				{
					Q4_Stop = 1;
				}
                
				
            }
            /* 无论成功与否均复位 */
            state = 0; idx = need = 0;
            break;
        }
    }

    ucUART_Zigbee_Pos = 0;  /* 清空接收缓冲 */
}




//========================K230部分串口=================

void UART1_Send_Bytes(char *buf, int len)
{
	  while(len--)
	  {
		DL_UART_Main_transmitDataBlocking(UART1_K230_INST, *buf);
		buf++;
	  }
}


void UART1_SendString(char *str)
{
    while (*str != '\0')  
    {
        DL_UART_Main_transmitDataBlocking(UART1_K230_INST, *str++);
    }
}

void UART1_K230_INST_IRQHandler(void) 
{
	switch(DL_UART_getPendingInterrupt(UART1_K230_INST) )
    {
		case DL_UART_IIDX_RX:
			Rx_K230_Buf[ucUART_K230_Pos++] = DL_UART_Main_receiveData(UART1_K230_INST);
//			my_printf("%d\r\n",(uint16_t)Rx_K230_Buf[ucUART_K230_Pos]);
			UART_K230_Flag = 1;
			
		break;
		default:
			
		break;
		
		
	}
}

//(0x21,d1,d2,0x22)。语音交互功能

/* ---------------- UART_K230_Proc ----------------- */
void UART_K230_Proc(void)
{
    /* 状态机：0=WAIT_HEAD  1=PAYLOAD  2=WAIT_TAIL */
    static uint8_t state   = 0;
    static uint8_t payload[2];   
    static uint8_t idx     = 0;

    if (!UART_K230_Flag) return;         /* 没有新数据 */
    UART_K230_Flag = 0;                  /* 清标志 */

    for (uint8_t i = 0; i < ucUART_K230_Pos; ++i)
    {
        uint8_t ch = Rx_K230_Buf[i];

        switch (state)
        {
        case 0:
            if (ch == 0x21)
            {
                idx   = 0;               /* 重置计数 */
                state = 1;               /* 转到接收 Payload */
            }
            break;

        case 1:
            if (idx < sizeof(payload))
                payload[idx++] = ch;

            if (idx >= sizeof(payload))
                state = 2;               /* 准备读取尾字节 */
            break;

        /* ---------- 2. 等待帧尾 55 并解析 ---------- */
        case 2:
            if (ch == 0x22)
            {
				my_printf("%d,%d\r\n",(uint16_t)payload[0],(uint16_t)payload[1]);
                if(payload[0] == 0xF1 && payload[1] == 0xF1)
					Q6_Overtake_Flag = 1;
				else if(payload[0] == 0xF2 && payload[1] == 0xF2)
					Q6_Overtaking_Flag = 1;
				else if(payload[0] == 0xF3 && payload[1] == 0xF3)
					Q6_Stop_Flag = 1;
				
				memset(payload, 0, sizeof(payload));
            }
            /* 无论尾字节是否正确，都回到等待头状态 */
            state = 0;
            break;

        default:
            state = 0;
            break;
        }
    }

    ucUART_K230_Pos = 0;   /* 清空接收计数，准备下一批数据 */
}

