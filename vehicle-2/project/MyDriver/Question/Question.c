#include "Question.h"
#include "control.h"
#include "encoder.h"
#include "track.h"
#include "UART.h"

//0.3m/s 1100
//0.5m/s 2100
//1.0m/s 72

//Q1变量
uint8_t Q1_Stop = 0;

//Q2变量
uint8_t Q2_Stop = 0;

//Q3变量
uint8_t Q3_Stop = 0;

//Q4变量
uint8_t Q4_Stop_Aid = 0;
uint8_t Q4_Stop = 0;

//Q6变量
uint8_t Q6_Stop = 0;
uint8_t Q6_Stop_Flag = 0;
uint8_t Q6_Overtake_Flag = 0; //下一圈超车
uint8_t Q6_Overtaking_Flag = 0; //立即超车


uint32_t time[2];

float speed_mpers = 0;

float speed_calculate(uint32_t *time_ms, uint8_t Out_Num, uint8_t In_Num)
{
    float dt = (float)(time_ms[1] - time_ms[0])*0.001f;  
    float ds = Out_Num * Out_Track +  In_Num*In_Track;
    return ds / dt;  // 单位 m/s
}




void Q1_Proc(void)
{
	if(!Q1_Stop)
	{
		if(!CH1 || !CH2 ||int16_tAbs(MyGyro) > 1000)
		{
			Track_InGyro.Base = 1000;
			Track_InGyro.Kd = 0.82f;
			Track_OutGray.Kp = 250.0f;
			Track_OutGray.Kd =15.0f;
		}
		else
		{
			Track_InGyro.Base = 1100;
			Track_InGyro.Kd = 0.77f;
			Track_OutGray.Kp = 230.0f;
			Track_OutGray.Kd = 40.0f;
		}
	}
	
	static uint8_t state;
	static uint64_t now_time;
	switch(state)
	{
		case 0:
			if(Stop_Flag)
			{
				Track_OutDist.Kp = 2.3f;Track_OutDist.Ki = 0.4f;
				Track_OutDist.Target = 210 + Fixed_Dist;
				Track_Flag = 1;
				Weigth_Flag = 0;
				state = 1;
			
				time[0] = u64_ms;
			}
			else
			{
				Track_Flag = 1;
				Weigth_Flag = 0;
			}
		break;
		case 1: // 到达B点岔路口
			if(Fork_Flag)
			{
				Weigth_Flag = 1;
				now_time = u64_ms;
				state = 2;
			}
		break;
		case 2: // 通过B点岔路口
			if(!Fork_Flag && (u64_ms - now_time > 100))
			{
				Weigth_Flag = 0;
				state = 3;
			}
		break;
		case 3: // 通过D点岔路口
			if(Fork_Flag && MyYaw_0_360 > 50)
			{
				state = 4;
			}
		break;
		case 4: // 等待Q1_Stop信号
			if(Q1_Stop)
			{
				now_time = u64_ms;
				state = 5;
			}
			else
			{
				Weigth_Flag = 0;
			}
		break;
		case 5: // 停车处理
			if(u64_ms - now_time > 500)
			{
				Load_Motor_A(0);
				Load_Motor_B(0);
		
				Track_Flag = 0;
				state = 6;
			}
			else
			{
				Track_Flag = 0;
				
				Track_OutDist.Kp = 3.1f;
				Track_OutDist.Ki = 0.9f;
				Track_InGyro.Base = 0;
				
				Track_Flag = 1;
			}
		break;
	}	
}

void Q2_Proc(void)
{
//	if(!Q2_Stop)
//	{
//		if(!CH1 || !CH2 ||int16_tAbs(MyGyro) > 1200)
//		{
//			Track_InGyro.Base = 2000;
//			Track_InGyro.Kd = 0.82f;
//			Track_OutGray.Kp = 310.0f;
//			Track_OutGray.Kd =55.0f;
//		}
//		else
//		{
//			Track_InGyro.Base = 2300;
//			Track_InGyro.Kd = 0.77f;
//			Track_OutGray.Kp = 290.0f;
//			Track_OutGray.Kd = 40.0f;
//		}
//	}
	
	
	static uint8_t state;
	static uint64_t now_time;
	switch(state)
	{
		case 0:
			if(Stop_Flag_90)
			{
				Track_OutDist.Kp = 4.2f;Track_OutDist.Ki = 0.2f;
				Track_Flag = 1;
				Weigth_Flag = 0;
				state = 1;
			
				time[0] = u64_ms;
			}
			else
			{
				Track_InGyro.Base = 2300;
				Track_InGyro.Kd = 0.82f;
				Track_OutGray.Kp = 310.0f;
				Track_OutGray.Kd =55.0f;
				Track_OutDist.Kp = 4.5f;Track_OutDist.Ki = 0.3f;
				Track_Flag = 1;
				Weigth_Flag = 0;
			}
		break;
		case 1: // 第一圈到达B点岔路口
			if(Fork_Flag)
			{
				Track_InGyro.Base = 2400;
				Weigth_Flag = 1;
				now_time = u64_ms;
				state = 2;
			}
		break;
		case 2: // 第一圈通过B点岔路口
			if(!Fork_Flag && (u64_ms - now_time > 160))
			{
				Track_InGyro.Base = 2300;
				Weigth_Flag = 0;
				state = 3;
			}
		break;
		case 3: // 第一圈通过D点岔路口
			if(Fork_Flag && (MyYaw_0_360 - 90) > 50)
			{
				state = 4;
			}
		break;
		case 4: // 第一圈结束
			if(Stop_Flag_90)
			{
				state = 5; // 开始第二圈
			}
		break;
		case 5: // 第二圈到达B点岔路口
			if(Fork_Flag)
			{
				Track_InGyro.Base = 2400;
				Weigth_Flag = 1;
				now_time = u64_ms;
				state = 6;
			}
		break;
		case 6: // 第二圈通过B点岔路口
			if(!Fork_Flag && (u64_ms - now_time > 160))
			{
				Track_InGyro.Base = 2300;
				Weigth_Flag = 0;
				state = 7;
			}
		break;
		case 7: // 第二圈通过D点岔路口
			if(Fork_Flag && (MyYaw_0_360 - 90) > 50)
			{
				state = 8;
			}
		break;
		case 8: // 等待Q2_Stop信号
			if(Q2_Stop)
			{
				now_time = u64_ms;
				state = 9;
			}
			else
			{
				Weigth_Flag = 0;
			}
		break;
		case 9: // 停车处理
			if(u64_ms - now_time > 500)
			{
				Load_Motor_A(0);
				Load_Motor_B(0);
		
				Track_Flag = 0;
				state = 10;
			}
			else
			{
				Track_Flag = 0;
				
				Track_OutDist.Kp = 4.9f;
				Track_OutDist.Ki = 0.9f;
				Track_InGyro.Base = 0;
				
				Track_Flag = 1;
			}
		break;
	}
}

void Q3_Proc(void)
{
	static uint8_t state = 0;
	static uint64_t now_time = 0;
	
	switch(state)
	{
		case 0:
			if(Stop_Flag)
			{
				Track_OutDist.Kp = 3.6f;Track_OutDist.Ki = 0.3f;
				Track_Flag = 1;
				Weigth_Flag = 0;
				state = 1;
			
			}
			else
			{
				Track_InGyro.Base = 1600;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 290.0f;
				Track_OutGray.Kd = 40.0f;
				Track_Flag = 1;
				Weigth_Flag = 0;
			}
		break;
		case 1: // 第一圈到达B点岔路口
			if(Fork_Flag)
			{
				Track_OutDist.Kp = 0;Track_OutDist.Ki = 0;
				Weigth_Flag = 1;
				now_time = u64_ms;
				state = 2;
			}
		break;
		case 2: // 第一圈通过B点岔路口
			if(!Fork_Flag && (u64_ms - now_time > 120))
			{
				Track_OutDist.Kp = 3.6f;Track_OutDist.Ki = 0.3f;
				Weigth_Flag = 0;
				state = 3;
			}
		break;
		case 3: // 第一圈通过D点岔路口
			if(Fork_Flag && MyYaw_0_360 > 90)
			{
				state = 4;
			}
		break;
		case 4: // 第一圈结束
			if(Stop_Flag)
			{
				state = 5; // 开始第二圈
			}
		break;
		case 5: // 第二圈到达B点岔路口
			if(Fork_Flag)
			{
				Track_OutDist.Kp = 0;Track_OutDist.Ki = 0;
				Weigth_Flag = 2;
				now_time = u64_ms;
				state = 6;
			}
		break;
		case 6: // 第二圈通过B点岔路口
			if(!Fork_Flag && (u64_ms - now_time > 120))
			{
				Track_OutDist.Kp = 3.6f;Track_OutDist.Ki = 0.3f;
				Track_InGyro.Base = 2600;
				Track_InGyro.Kd = 0.67f;
				Track_OutGray.Kp = 560.0f;
				Track_OutGray.Kd = 60.0f;
				Weigth_Flag = 0;
				state = 7;
			}
		break;
		case 7: // 第二圈通过D点岔路口
			if(Fork_Flag && MyYaw_0_360 > 90)
			{
				Track_OutDist.Kp = 0;Track_OutDist.Ki = 0;
				Track_InGyro.Base = 2000;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 320.0f;
				Track_OutGray.Kd = 40.0f;
				state = 8;
			}
		break;
		case 8: // 第二圈结束
			if(Stop_Flag)
			{
				state = 9; // 开始第三圈
			}
		break;
		case 9: // 第三圈到达B点岔路口
			if(Fork_Flag)
			{
				Weigth_Flag = 1;
				now_time = u64_ms;
				state = 10;
			}
		break;
		case 10: // 第三圈通过B点岔路口
			if(!Fork_Flag && (u64_ms - now_time > 120))
			{
				Track_InGyro.Base = 1700;
				Track_InGyro.Kd = 0.67f;
				Track_OutGray.Kp = 320.0f;
				Track_OutGray.Kd = 40.0f;
				Weigth_Flag = 0;
				state = 11;
			}
		break;
		case 11: // 第三圈通过D点岔路口
			if(Fork_Flag && MyYaw_0_360 > 90)
			{
				Track_OutDist.Kp = 5.1f;Track_OutDist.Ki = 0.5f;
				Track_InGyro.Base = 1900;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 320.0f;
				Track_OutGray.Kd = 40.0f;
				state = 12;
			}
		break;
		case 12: // 等待Q3_Stop信号
			if(Q3_Stop)
			{
				now_time = u64_ms;
				state = 13;
			}
			else
			{
				Weigth_Flag = 0;
			}
		break;
		case 13: // 停车处理
			if(u64_ms - now_time > 500)
			{
				Load_Motor_A(0);
				Load_Motor_B(0);
		
				Track_Flag = 0;
				state = 14;
			}
			else
			{
				Track_Flag = 0;
				
				Track_OutDist.Kp = 3.9f;
				Track_OutDist.Ki = 0.9f;
				Track_InGyro.Base = 0;
				
				Track_Flag = 1;
			}
		break;
	}
	
}


void Q4_Proc(void)
{
	static uint8_t state;
	static uint64_t now_time;
//	my_printf("state: %d\r\n",state);
	switch(state)
	{
		case 0:
			if(Stop_Flag)
			{
				Track_OutDist.Kp = 12.7f;Track_OutDist.Ki = 0.2f;
				Track_InGyro.Base = 3400;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 920.0f;
				Track_OutGray.Kd = 60.0f;
				weight_set[0][0] = -5;
				weight_set[0][1] = -3;
				weight_set[0][2] = -1;
				weight_set[0][3] = 0;
				weight_set[0][4] = 1;
				weight_set[0][5] = 1;
				weight_set[0][6] = 3;
				weight_set[0][7] = 5;
				Track_Flag = 1;
				Weigth_Flag = 0;
				state = 1;
			
				time[0] = u64_ms;
			}
			else
			{
				Track_OutDist.Kp = 0;Track_OutDist.Ki = 0;
				Track_InGyro.Base = 3400;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 980.0f;
				Track_OutGray.Kd = 80.0f;
				Track_Flag = 1;
				Weigth_Flag = 0;
			}
		break;
		case 1: // 第一圈到达B点岔路口
			if(Fork_Flag)
			{
				Track_OutDist.Kp = 0;Track_OutDist.Ki = 0;
				Weigth_Flag = 1;
				Track_InGyro.Base = 4000;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 980.0f;
				Track_OutGray.Kd = 80.0f;
				
				now_time = u64_ms;
				state = 2;
			}
		break;
		case 2: // 第一圈通过B点岔路口
			if(!Fork_Flag && (u64_ms - now_time > 60))
			{
				
				Track_InGyro.Base = 3300;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 920.0f;
				Track_OutGray.Kd = 100.0f;
				Weigth_Flag = 0;
				state = 3;
			}
		break;
		case 3:
			if(MyYaw_0_360 > 70)
			{
				Track_OutDist.Kp = 17.7f;Track_OutDist.Ki = 0.3f;
				Track_InGyro.Base = 3500;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 960.0f;
				Track_OutGray.Kd = 70.0f;
				now_time = u64_ms;
				state = 4;
			}
		break;
		case 4:
			if(u64_ms - now_time > 500)
			{
				Track_OutDist.Kp = 0;Track_OutDist.Ki = 0;
				Track_InGyro.Base = 3400;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 970.0f;
				Track_OutGray.Kd = 100.0f;
				state = 5;
			}
		break;
		case 5: // 第一圈通过D点岔路口
			if(Fork_Flag && MyYaw_0_360 > 80)
			{
				Track_OutDist.Kp = 17.7f;Track_OutDist.Ki = 0.3f;
				Track_InGyro.Base = 3500;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 940.0f;
				Track_OutGray.Kd = 110.0f;
				now_time = u64_ms;
				state = 6;
			}

		break;
		case 6: // 第一圈弯道
			if(u64_ms - now_time > 250)
			{
				Track_OutDist.Kp = 0;Track_OutDist.Ki = 0;
				weight_set[0][0] = -3;
				weight_set[0][1] = -2;
				weight_set[0][2] = -2;
				weight_set[0][3] = -1;
				weight_set[0][4] = 0;
				weight_set[0][5] = 1;
				weight_set[0][6] = 3;
				weight_set[0][7] = 5;
				Track_InGyro.Base = 2500;
				Track_InGyro.Kd = 0.76f;
				Track_OutGray.Kp = 1130.0f;
				Track_OutGray.Kd = 100.0f;
				state = 7;
			}
		break;
		case 7:
			if(Q4_Stop_Aid)
			{
				Track_Flag = 0;
				
				Load_Motor_A(-6000);
				Load_Motor_B(-6000);
				
				delay_ms(200);
				Load_Motor_A(0);
				Load_Motor_B(0);
				
				now_time = u64_ms;
				state = 8;
			}
		break;
		case 8:
			if(!Q4_Stop_Aid)
			{
				
				Track_Flag = 1;
				
				
				weight_set[0][0] = -3;
				weight_set[0][1] = -2;
				weight_set[0][2] = -1;
				weight_set[0][3] = -1;
				weight_set[0][4] = 0;
				weight_set[0][5] = 1;
				weight_set[0][6] = 2;
				weight_set[0][7] = 3;
				Track_InGyro.Base = 3200;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 920.0f;
				Track_OutGray.Kd = 60.0f;
				now_time = u64_ms;
				
				Track_OutDist.Kp = 17.7f;Track_OutDist.Ki = 0.3f;
				delay_ms(100);
				state = 9;
			}
		break;
		case 9:
			if(u64_ms - now_time > 500)
			{
				Track_OutDist.Kp = 0;Track_OutDist.Ki = 0;
				Track_InGyro.Base = 3000;
				Track_InGyro.Kd = 0.76f;
				Track_OutGray.Kp = 920.0f;
				Track_OutGray.Kd = 70.0f;
				weight_set[0][0] = -3;
				weight_set[0][1] = -2;
				weight_set[0][2] = -1;
				weight_set[0][3] = -1;
				weight_set[0][4] = 0;
				weight_set[0][5] = 1;
				weight_set[0][6] = 2;
				weight_set[0][7] = 3;
				state = 10;
			}
		break;
		case 10:
			if(floatAbs(MyYaw_180_180) < 50)
			{
				weight_set[0][0] = -5;
				weight_set[0][1] = -3;
				weight_set[0][2] = -1;
				weight_set[0][3] = 0;
				weight_set[0][4] = 1;
				weight_set[0][5] = 1;
				weight_set[0][6] = 3;
				weight_set[0][7] = 5;
				Track_OutDist.Kp = 11.7f;Track_OutDist.Ki = 0.2f;
				Track_InGyro.Base = 3100;
				Track_InGyro.Kd = 0.76f;
				Track_OutGray.Kp = 940.0f;
				Track_OutGray.Kd = 100.0f;
				now_time = u64_ms;
				state = 11;
			}
		break;
		case 11:
			if(u64_ms - now_time > 200)
			{
				Track_InGyro.Base = 3100;
				Track_InGyro.Kd = 0.76f;
				Track_OutGray.Kp = 990.0f;
				Track_OutGray.Kd = 100.0f;
				Weigth_Flag = 0;
				state = 12;
			}
		break;
		case 12: // 等待Q4_Stop信号
			if(Q4_Stop)
			{
				now_time = u64_ms;
				state = 13;
			}
			else
			{
				Weigth_Flag = 0;
			}
		break;
		case 13: // 停车处理
			if(u64_ms - now_time > 300)
			{
				Load_Motor_A(0);
				Load_Motor_B(0);
		
				Track_Flag = 0;
				state = 14;
			}
			else
			{
				Track_Flag = 0;
				
				Track_OutDist.Kp = 16.9f;
				Track_OutDist.Ki = 0.9f;
				Track_InGyro.Base = 0;
				
				Track_Flag = 1;
			}
		break;
	}


}


void Q5_Proc(void)
{
	static uint8_t state;
	static uint64_t now_time;
	my_printf("state: %d\r\n",state);
	switch(state)
	{
		case 0:
			if(Stop_Flag)
			{
				Track_InGyro.Base = 4900;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 920.0f;
				Track_OutGray.Kd = 60.0f;
				weight_set[0][0] = -5;
				weight_set[0][1] = -3;
				weight_set[0][2] = -1;
				weight_set[0][3] = 0;
				weight_set[0][4] = 1;
				weight_set[0][5] = 1;
				weight_set[0][6] = 3;
				weight_set[0][7] = 5;
				Track_Flag = 1;
				Weigth_Flag = 0;
				state = 1;
			
				time[0] = u64_ms;
			}
			else
			{
				Track_OutDist.Kp = 0;Track_OutDist.Ki = 0;
				Track_InGyro.Base = 4800;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 920.0f;
				Track_OutGray.Kd = 80.0f;
				Track_Flag = 1;
				Weigth_Flag = 0;
			}
		break;
		case 1: // 第一圈到达B点岔路口
			if(Fork_Flag)
			{
				Weigth_Flag = 1;
				Track_InGyro.Base = 4800;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 920.0f;
				Track_OutGray.Kd = 60.0f;
				
				now_time = u64_ms;
				state = 2;
			}
		break;
		case 2: // 第一圈通过B点岔路口
			if(!Fork_Flag && (u64_ms - now_time > 60))
			{
				Track_InGyro.Base = 4500;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 1120.0f;
				Track_OutGray.Kd = 130.0f;
				Weigth_Flag = 0;
				state = 3;
			}
		break;
		case 3:
			if(MyYaw_0_360 > 70)
			{
				Track_InGyro.Base = 4400;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 940.0f;
				Track_OutGray.Kd = 110.0f;
				now_time = u64_ms;
				state = 4;
			}
		break;
		case 4:
			if(u64_ms - now_time > 500)
			{
				Track_InGyro.Base = 4400;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 970.0f;
				Track_OutGray.Kd = 140.0f;
				state = 5;
			}
		break;
		case 5: // 第一圈通过D点岔路口
			if(Fork_Flag && MyYaw_0_360 > 90)
			{
				Track_InGyro.Base = 4900;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 950.0f;
				Track_OutGray.Kd = 80.0f;
				now_time = u64_ms;
				state = 6;
			}

		break;
		case 6: // 第一圈弯道
			if(u64_ms - now_time > 550)
			{
				weight_set[0][0] = -3;
				weight_set[0][1] = -1;
				weight_set[0][2] = 0;
				weight_set[0][3] = 1;
				weight_set[0][4] = 2;
				weight_set[0][5] = 3;
				weight_set[0][6] = 3;
				weight_set[0][7] = 5;
				Track_InGyro.Base = 4200;
				Track_InGyro.Kd = 0.76f;
				Track_OutGray.Kp = 1320.0f;
				Track_OutGray.Kd = 100.0f;
				state = 7;
			}
		break;
		case 7:
			state = 8;
		break;
		case 8:
			state = 9;

		break;
		case 9:
			if(u64_ms - now_time > 300)
			{
				Track_InGyro.Base = 4200;
				Track_InGyro.Kd = 0.76f;
				Track_OutGray.Kp = 1320.0f;
				Track_OutGray.Kd = 100.0f;
				state = 10;
			}
		break;
		case 10:
			if(floatAbs(MyYaw_180_180) < 50)
			{
				Track_InGyro.Base = 4900;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 920.0f;
				Track_OutGray.Kd = 70.0f;
				now_time = u64_ms;
				state = 11;
			}
		break;
		case 11:
			if(u64_ms - now_time > 200)
			{
				Track_InGyro.Base = 4800;
				Track_InGyro.Kd = 0.77f;
				Track_OutGray.Kp = 1120.0f;
				Track_OutGray.Kd = 150.0f;
				Weigth_Flag = 0;
				state = 12;
			}
		break;
		case 12:
			if(Stop_Flag)
			{
				Track_Flag = 0;
				time[1] = u64_ms;
				Load_Motor_A(-6000);
				Load_Motor_B(-6000);
				Load_Motor_A(0);
				Load_Motor_B(0);
				speed_mpers = speed_calculate(time, 1, 0);
				state = 13;
			}
		break;
	}


}

void Q6_Proc(void)
{
	static uint8_t state_normal;
	static uint8_t state_overtaking;
	static uint8_t state_stop;
	static float now_angle;
	static uint64_t now_time;

	if(Q6_Overtaking_Flag == 1)
	{
		switch(state_overtaking)
		{
			case 0:
				Track_Flag = 0;
				Turn_Flag = 1;
				now_angle = MyYaw_180_180;
				Turn_OutAngle.Target = now_angle + 90;
				Turn_InGyro.Base = 0;
				now_time = u64_ms;
				state_overtaking = 1;
				
			break;
			case 1:
				if(floatAbs(Turn_OutAngle.Error0) < 2 && (u64_ms - now_time > 100))
				{
					Turn_InGyro.Base = 2000;
					state_overtaking = 2;
					
				}
			break;
			case 2:
				if(!CH1 || !CH2)
				{
					Turn_InGyro.Base = 0;
					now_angle = MyYaw_180_180;
					Turn_OutAngle.Target = now_angle + 80;
					now_time = u64_ms;
					state_overtaking = 3;
				}
			break;
			case 3:
				if(floatAbs(Turn_OutAngle.Error0) < 5)
				{
					Track_Flag = 1;
					Turn_Flag = 0;
					Track_InGyro.Base = 2000;
					Track_InGyro.Kd = 0.77f;
					Track_OutGray.Kp = 290.0f;
					Track_OutGray.Kd = 40.0f;
					Q6_Overtaking_Flag = 0;
					state_normal = 0;
					state_overtaking = 4;
				}
			break;
		}
	}
	
	else if(Q6_Stop_Flag == 1)
	{
		switch(state_stop)
		{
			case 0: // 等待Q2_Stop信号
				
				Zigbee_Send_Bytes(0xA6,0xA6);
				now_time = u64_ms;
				state_stop = 9;
			break;
			case 1: // 停车处理
				if(u64_ms - now_time > 500)
				{
					Load_Motor_A(0);
					Load_Motor_B(0);
			
					Track_Flag = 0;
					state_normal = 10;
				}
				else
				{
					Track_Flag = 0;
					
					Track_OutDist.Kp = 3.9f;
					Track_OutDist.Ki = 0.9f;
					Track_InGyro.Base = 0;
					
					Track_Flag = 1;
				}
			break;
		}
	}
	
	else
	{
		switch(state_normal)
		{
			case 0:
				if(Stop_Flag_90)
				{
					Track_OutDist.Kp = 4.2f;Track_OutDist.Ki = 0.2f;
					Track_Flag = 1;
					Weigth_Flag = 0;
					state_normal = 1;
				
					time[0] = u64_ms;
				}
				else
				{
					Track_InGyro.Base = 1800;
					Track_InGyro.Kd = 0.77f;
					Track_OutGray.Kp = 320.0f;
					Track_OutGray.Kd = 40.0f;
					Track_OutDist.Kp = 3.1f;Track_OutDist.Ki = 0.5f;
					Track_Flag = 1;
					Weigth_Flag = 0;
				}
			break;
			case 1: // 第一圈到达B点岔路口
				if(Fork_Flag)
				{
					Weigth_Flag = 1;
					now_time = u64_ms;
					state_normal = 2;
				}
			break;
			case 2: // 第一圈通过B点岔路口
				if(!Fork_Flag && (u64_ms - now_time > 100))
				{
					Weigth_Flag = 0;
					state_normal = 3;
				}
			break;
			case 3: // 第一圈通过D点岔路口
				if(Fork_Flag && (MyYaw_0_360 - 90) > 50)
				{
					state_normal = 4;
				}
			break;
			case 4: // 第一圈结束
				if(Stop_Flag_90)
				{
					state_normal = 5; // 开始第二圈
				}
			break;
			case 5: // 第二圈到达B点岔路口
				if(Fork_Flag)
				{
					Weigth_Flag = 1;
					now_time = u64_ms;
					state_normal = 6;
				}
			break;
			case 6: // 第二圈通过B点岔路口
				if(!Fork_Flag && (u64_ms - now_time > 200))
				{
					Weigth_Flag = 0;
					state_normal = 7;
				}
			break;
			case 7: // 第二圈通过D点岔路口
				if(Fork_Flag && (MyYaw_0_360 - 90) > 50)
				{
					state_normal = 8;
				}
			break;
			case 8: // 等待Q2_Stop信号
				if(Q6_Stop)
				{
					now_time = u64_ms;
					state_normal = 9;
				}
				else
				{
					Weigth_Flag = 0;
				}
			break;
			case 9: // 停车处理
				if(u64_ms - now_time > 500)
				{
					Load_Motor_A(0);
					Load_Motor_B(0);
			
					Track_Flag = 0;
					state_normal = 10;
				}
				else
				{
					Track_Flag = 0;
					
					Track_OutDist.Kp = 3.9f;
					Track_OutDist.Ki = 0.9f;
					Track_InGyro.Base = 0;
					
					Track_Flag = 1;
				}
			break;
		}
	}
}