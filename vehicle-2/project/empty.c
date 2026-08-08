/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include "ti_msp_dl_config.h"
#include "UART.h"
#include "stdio.h"
#include "motor.h"
#include "oled.h"
#include "control.h"
#include "Encoder.h"
#include "board.h"
#include "servo.h"
#include "key.h"
#include "track.h"
#include "Question.h"
#include "No_Mcu_Ganv_Grayscale_Sensor_Config.h"

int16_t aa = 1000;
int16_t bb = 1000;



//Angle
float Angle_Target = 0;
uint8_t Get_Angle_Flag = 0;
float MyYaw_180_180 = 0;
float MyYaw_0_360 = 0;

int16_t MyGyro= 0;



//PID
volatile uint8_t Track_Flag = 0;
volatile uint8_t Turn_Flag = 0;

//Key
uint8_t Key_Val;
uint8_t Key_Val_Old;

//Dist
uint16_t dist_mm = 399;


//Tim
uint64_t u64_ms = 0;
uint32_t u32_UART_Dly = 0;
uint32_t u32_Dist_Dly = 0;
uint32_t u32_OLED_Dly = 0;
uint32_t u32_Key_Dly = 0;







unsigned short Normal[8];
unsigned char rx_buff[256]={0};

//Question
uint8_t Q_Num = 0;
uint8_t Q_State = 0;




//Founction
void UART_User_Proc(void);
void OLED_Proc(void);
void Data_Proc(void);
void Dist_Proc(void);
void Key_Proc(void);


uint16_t t = 10000;

int main(void)
{
    SYSCFG_DL_init();
	
	
	IMU_Init();
	Motor_Init();
	

	
	PID_Init();
	OLED_Init();
	Gray_Init();
	dl1b_init();
	
	
	delay_ms(100);
	NVIC_ClearPendingIRQ(UART1_K230_INST_INT_IRQN);
	NVIC_ClearPendingIRQ(UART0_Zigbee_INST_INT_IRQN);
	NVIC_ClearPendingIRQ(TIMG_SysTick_INST_INT_IRQN);
	
	NVIC_EnableIRQ(UART1_K230_INST_INT_IRQN);
	NVIC_EnableIRQ(UART0_Zigbee_INST_INT_IRQN);
	NVIC_EnableIRQ(TIMG_SysTick_INST_INT_IRQN);
	
	

	UART2_SendString("You Can Start!");
	
	

    while (1) 
	{

		
//		my_printf("%f,%f\r\n",Turn_OutAngle.Target,Turn_OutAngle.Actual);

					//获取传感器模拟量结果(有黑白值初始化后返回1 没有返回 0)
//			if(Get_Anolog_Value(&sensor,Anolog)){
//			sprintf((char *)rx_buff,"Anolog %d-%d-%d-%d-%d-%d-%d-%d\r\n",Anolog[0],Anolog[1],Anolog[2],Anolog[3],Anolog[4],Anolog[5],Anolog[6],Anolog[7]);
//			UART2_SendString((char *)rx_buff);
//			memset(rx_buff,0,256);
//			}
		
		if(Q_Num == 0 && Q_State == 1)
			Gray_CAL();
		else if(Q_Num == 1 && Q_State == 1)
			Q1_Proc();
		else if(Q_Num == 2 && Q_State == 1)
			Q2_Proc();
		else if(Q_Num == 3 && Q_State == 1)
			Q3_Proc();
		else if(Q_Num == 4 && Q_State == 1)
			Q4_Proc();
		else if(Q_Num == 5 && Q_State == 1)
			Q5_Proc();
		else if(Q_Num == 6 && Q_State == 1)
			Q6_Proc();

		
		
		
		OLED_Proc();
		Key_Proc();
		Gray_Detect();
		Dist_Proc();
		UART_K230_Proc();
//       	UART1_SendString("IIIIIIIIIIIII");
			

		UART_Zigbee_Proc();
		

    }
}

void OLED_Proc(void)
{
	if(u32_OLED_Dly < 1000)
		return;
	u32_OLED_Dly = 0;
	
	OLED_Printf(0,0,OLED_8X16,"Num:%1d State:%1d",Q_Num,Q_State);
	
	
	if(Q_Num == 0 && Q_State == 1)
	{
		OLED_Printf(0,16,OLED_6X8,"%4d-%4d-%4d-%4d",Anolog_white[0],Anolog_white[1],Anolog_white[2],Anolog_white[3]);
		OLED_Printf(0,24,OLED_6X8,"%4d-%4d-%4d-%4d",Anolog_white[4],Anolog_white[5],Anolog_white[6],Anolog_white[7]);
		OLED_Printf(0,32,OLED_6X8,"%4d-%4d-%4d-%4d",Anolog_black[0],Anolog_black[1],Anolog_black[2],Anolog_black[3]);
		OLED_Printf(0,40,OLED_6X8,"%4d-%4d-%4d-%4d",Anolog_black[4],Anolog_black[5],Anolog_black[6],Anolog_black[7]);
		OLED_Printf(0,48,OLED_8X16,"Now:%d",(uint16_t)CAL_Flag);
	}
	else if(Q_Num == 1 && Q_State == 1)
	{
		OLED_Printf(0,16,OLED_8X16,"Speed:%.3f",speed_mpers);
	}
	else if(Q_Num == 2 && Q_State == 1)
	{
		OLED_Printf(0,16,OLED_8X16,"Speed:%.3f",speed_mpers);
	}
	else if(Q_Num == 3 && Q_State == 1)
	{
		OLED_Printf(0,16,OLED_8X16,"Speed:%.3f",speed_mpers);
	}
	else if(Q_Num == 4 && Q_State == 1)
	{
		OLED_Printf(0,16,OLED_8X16,"Speed:%.3f",speed_mpers);
	}
	else if(Q_Num == 5 && Q_State == 1)
	{
		OLED_Printf(0,16,OLED_8X16,"Speed:%.3f",speed_mpers);
	}
//	OLED_Printf(0,16,OLED_8X16,"%lu",u64_ms);
	OLED_Printf(0,32,OLED_8X16,"AA%d",(uint16_t)Q6_Stop_Flag);
	OLED_Printf(0,48,OLED_8X16,"%d-%d-%d-%d-%d-%d-%d-%d",(Digtal>>0)&0x01,(Digtal>>1)&0x01,(Digtal>>2)&0x01,(Digtal>>3)&0x01,(Digtal>>4)&0x01,(Digtal>>5)&0x01,(Digtal>>6)&0x01,(Digtal>>7)&0x01);
	
	OLED_Update();
}




void Dist_Proc(void)
{
	static uint16_t dl1b_distance_mm_old;
	dl1b_get_distance();
	if(dl1b_finsh_flag == 1)
	{
		dl1b_finsh_flag = 0;
		
		dl1b_distance_mm_old = dl1b_distance_mm;
		
		dist_mm = (7*dl1b_distance_mm + 3*dl1b_distance_mm_old)*0.1f;
		uint16_tLimit(&dist_mm,0,400);
//		my_printf("%d mm \r\n",dist_mm);
	}
}

void Key_Proc(void)
{
	if(u32_Key_Dly < 20)
		return;
	u32_Key_Dly = 0;
	
	Key_Val = Read_Key();
	if(Key_Val == Key_Val_Old)
		return;
	switch(Key_Val)
	{
		case 1:
			Q_Num = (Q_Num + 1) % 7;
		break;
		case 2:
			Q_State = (Q_State + 1) % 2;
		break;
		case 3:
			if(Q_Num == 0 && Q_State == 1)
				CAL_Flag = (CAL_Flag + 1) % 4;
		break;
		case 4:
			Q6_Overtaking_Flag = 1;
		break;
	}
	
	
	
	Key_Val_Old = Key_Val;
}


void Data_Proc(void)
{
	

	MyYaw_180_180 = euler.yaw;
	MyYaw_0_360 = my_fmod(MyYaw_180_180 + 360,360);
	
	
	MyGyro = Gyro_Actual.Zdata;
	
}



void TIMG_SysTick_INST_IRQHandler(void)
{
	switch(DL_TimerG_getPendingInterrupt(TIMG_SysTick_INST))
	{
		case DL_TIMER_IIDX_ZERO:
			u64_ms++;
			u32_UART_Dly++;
			u32_OLED_Dly++;
			u32_Dist_Dly++;
			u32_Key_Dly++;
			if(u64_ms % 5 == 0)
			{
				quaternion_update();
				quaternion_to_euler();
				Data_Proc();
				
//				aa = Encoder_Get_L();
//				bb = Encoder_Get_R();
			}
			
			
			
//			if(Speed_Left_Flag)
//				Speed_Left_PI_Control();
			
//			if(Speed_Right_Flag)
//				Speed_Right_PI_Control();
			
			if(Track_Flag)
				Track_PID_Control();
			if(Turn_Flag)
				Turn_PID_Control();
			
			DL_TimerG_clearInterruptStatus(TIMG_SysTick_INST, DL_TIMER_IIDX_ZERO);
		break;
		default:
			
		break;
	}
}
