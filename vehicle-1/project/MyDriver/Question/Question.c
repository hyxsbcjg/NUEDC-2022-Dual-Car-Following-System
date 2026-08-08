#include "Question.h"
#include "control.h"
#include "encoder.h"
#include "track.h"
#include "peripheral.h"

//0.3m/s 22
//0.5m/s 36
//1.0m/s 72


//Q6变量
uint8_t Q6_Stop_Flag = 0;



uint32_t time[2];

float speed_mpers = 0;

float speed_calculate(uint32_t *time_ms, uint8_t Out_Num, uint8_t In_Num)
{
    float dt = (float)(time_ms[1] - time_ms[0])*0.001f;  
    float ds = Out_Num * Out_Track +  In_Num*In_Track;
    return ds / dt;  // 单位 m/s
}

//Get_Anolog_Value(&sensor,Anolog)


void Test_Proc(void)
{
	if(int16_tAbs(MyGyro) > 1000)
	{
		Track_MidGyro.Base = 40;
		Track_MidGyro.Kd = 55e-4;
		Track_OutGray.Kp =350.0f;
		Track_OutGray.Kd =50.0f;
	}
	else
	{
		Track_MidGyro.Base = 43;
		Track_MidGyro.Kd = 43e-4;
		Track_OutGray.Kp = 330.0f;
		Track_OutGray.Kd = 40.0f;
	}
	
	static uint8_t state;
	static uint64_t now_time;
	my_printf("state: %d\r\n",state);
	switch(state)
	{
		case 0:
			if(Stop_Flag)
			{
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
		case 1:
			//走外圈
			if(Fork_Flag)
			{
				Weigth_Flag = 1;
				now_time = u64_ms;
				state = 2;
			}
				
		break;
		case 2:
			if(!Fork_Flag && (u64_ms - now_time > 200))
			{
				now_time = u64_ms;
				Weigth_Flag = 0;
				state = 3;
			}
		break;
		case 3:
			if(Fork_Flag && MyYaw_0_360 > 50) //第一次通过D点
			{
				state = 4;
			}
		break;
		case 4:
			if(Stop_Flag)
			{
				state = 5; //跑完第一圈
			}
		break;
		case 5:
			//走内圈
			if(Fork_Flag)
			{
				Weigth_Flag = 2;
				now_time = u64_ms;
				state = 6;
			}
				
		break;
		case 6:
			if(!Fork_Flag && (u64_ms - now_time > 200))
			{
				Weigth_Flag = 0;
				state = 7;
			}
		break;
		case 7:
			if(Fork_Flag && MyYaw_0_360 > 50) //第二次通过D点
			{
				state = 8;
			}
		break;
		case 8:
			if(Stop_Flag) //第二圈结束
			{
				Track_Flag = 0;
				
				Load_Motor_A(0);
				Load_Motor_B(0);
				time[1] = u64_ms;
				
				speed_mpers = speed_calculate(time,1,0);
				my_printf("%f\r\n",speed_mpers);
				my_printf("%u-%u\r\n",time[0],time[1]);
				
				state = 10;
			}
			else
			{
				Weigth_Flag = 0;
			}
		break;
	}
}

void Q1_Proc(void)
{
    if(!CH1 || !CH2 ||int16_tAbs(MyGyro) > 1000)
    {
        Track_MidGyro.Base = 15;
        Track_MidGyro.Kd = 36e-4;
        Track_OutGray.Kp =230.0f;
        Track_OutGray.Kd =15.0f;
    }
    else
    {
        Track_MidGyro.Base = 17;
        Track_MidGyro.Kd = 43e-4;
        Track_OutGray.Kp = 210.0f;
        Track_OutGray.Kd = 40.0f;
    }
    
    static uint8_t state;
    static uint64_t now_time;
    switch(state)
    {
        case 0:
            if(Stop_Flag)
            {
                Zigbee_Send_Bytes(0x10,0x10);
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
                now_time = u64_ms;
                Weigth_Flag = 1;
                state = 2;
            }
        break;
        case 2: // 通过B点岔路口
            if(!Fork_Flag && ((u64_ms - now_time) > 200))
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
        case 4: // 等待停车线
            if(Stop_Flag)
            {
                Track_Flag = 0;
                Zigbee_Send_Bytes(0x1A,0x1A);
                Load_Motor_A(-1500);
                Load_Motor_B(-1500);
                delay_ms(200);
                Load_Motor_A(0);
                Load_Motor_B(0);
        
                time[1] = u64_ms;
                
                speed_mpers = speed_calculate(time,1,0);
//                my_printf("%f\r\n",speed_mpers);
//                my_printf("%u-%u\r\n",time[0],time[1]);
                
                state = 5;
            }
            else
            {
                Weigth_Flag = 0;
            }
        break;
    }
}

void Q2_Proc(void)
{
    if(!CH1 || !CH2 ||int16_tAbs(MyGyro) > 1200)
    {
        Track_MidGyro.Base = 28;
        Track_MidGyro.Kd = 32e-4;
        Track_OutGray.Kp =270.0f;
        Track_OutGray.Kd =15.0f;
    }
    else
    {
        Track_MidGyro.Base = 33;
        Track_MidGyro.Kd = 43e-4;
        Track_OutGray.Kp = 250.0f;
        Track_OutGray.Kd = 50.0f;
    }
    
    static uint8_t state;
    static uint64_t now_time;
    switch(state)
    {
        case 0:
            if(Stop_Flag)
            {
                Zigbee_Send_Bytes(0x20,0x20);
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
        case 1: // 第一圈到达B点岔路口
            if(Fork_Flag)
            {
                Weigth_Flag = 1;
                now_time = u64_ms;
                state = 2;
            }
        break;
        case 2: // 第一圈通过B点岔路口
            if(!Fork_Flag && (u64_ms - now_time > 230))
            {
                Weigth_Flag = 0;
                state = 3;
            }
        break;
        case 3: // 第一圈通过D点岔路口
            if(Fork_Flag && MyYaw_0_360 > 50)
            {
                state = 4;
            }
        break;
        case 4: // 第一圈结束，等待停车线
            if(Stop_Flag)
            {
                state = 5; // 开始第二圈
            }
        break;
        case 5: // 第二圈到达B点岔路口
            if(Fork_Flag)
            {
                Weigth_Flag = 1;
                now_time = u64_ms;
                state = 6;
            }
        break;
        case 6: // 第二圈通过B点岔路口
            if(!Fork_Flag && (u64_ms - now_time > 230))
            {
                Weigth_Flag = 0;
                state = 7;
            }
        break;
        case 7: // 第二圈通过D点岔路口
            if(Fork_Flag && MyYaw_0_360 > 50)
            {
                state = 8;
            }
        break;
        case 8: // 第二圈结束
            if(Stop_Flag)
            {
                Zigbee_Send_Bytes(0x2A,0x2A);
                Track_Flag = 0;
                
                Load_Motor_A(0);
                Load_Motor_B(0);
            
                time[1] = u64_ms;
                
                speed_mpers = speed_calculate(time,2,0);
                my_printf("%f\r\n",speed_mpers);
                my_printf("%u-%u\r\n",time[0],time[1]);
                
                state = 9;
            }
            else
            {
                Weigth_Flag = 0;
            }
        break;
    }
}



void Q3_Proc(void)
{
    static uint8_t  state   = 0;     
    static uint64_t now_time = 0;     
	my_printf("state: %d\r\n",state);
    switch(state)
    {
   
		case 0:
			if(Stop_Flag)               
			{
				Zigbee_Send_Bytes(0x30,0x30); 
				Track_Flag  = 1;         
				Weigth_Flag = 0;        
				state       = 1;        
			}
			else                        
			{
				Track_MidGyro.Kd   = 43e-4;
				Track_OutGray.Kp   = 320.0f;
				Track_OutGray.Kd   = 50.0f;
				Track_MidGyro.Base = 30;
				Track_Flag         = 1;
				Weigth_Flag        = 0;
			}
		break;
	  
		case 1: // 第一圈到达B点岔路口
			if(Fork_Flag)
			{
				Track_MidGyro.Base = 51;
				Weigth_Flag = 1;         
				now_time    = u64_ms;
				state       = 2;    
			}
		break;
	   
		case 2: // 第一圈通过B点岔路口
			if(!Fork_Flag && (u64_ms - now_time > 250))
			{
				Track_MidGyro.Base = 30;
				Weigth_Flag = 0;
				state       = 3;        
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
				state = 5;              
			}
		break;
	   
		case 5: // 第二圈到达B点岔路口
			if(Fork_Flag)
			{
				Track_MidGyro.Base = 51;
				Weigth_Flag = 1;         
				now_time    = u64_ms;
				state       = 6;
			}
		break;
	   
		case 6: // 第二圈通过B点岔路口
			if(!Fork_Flag && (u64_ms - now_time > 250))
			{
				
				Track_MidGyro.Kd   = 43e-4;
				Track_OutGray.Kp   = 250.0f;
				Track_OutGray.Kd   = 25.0f;
				Track_MidGyro.Base = 20;
				Weigth_Flag = 0;
				state       = 7;               
			}
		break;
	 
		case 7: // 第二圈通过D点岔路口
			if(Fork_Flag && MyYaw_0_360 > 80)
			{
				Track_MidGyro.Kd   = 43e-4;
				Track_OutGray.Kp   = 330.0f;
				Track_OutGray.Kd   = 24.0f;
				Track_MidGyro.Base = 32;
				state = 8;               
			}
		break;
		
		case 8: // 第二圈结束
			if(Stop_Flag)
			{
				state = 9;
			}
		break;
		
		case 9: // 第三圈到达B点岔路口
			if(Fork_Flag)
			{
				Track_MidGyro.Base = 51;
				Weigth_Flag = 2;         
				now_time    = u64_ms;
				state       = 10;        
			}
		break;
		
		case 10: // 第三圈通过B点岔路口
			if(!Fork_Flag && (u64_ms - now_time > 270))
			{
				// 加速参数
				Track_MidGyro.Kd   = 53e-4;
				Track_OutGray.Kp   = 510.0f;
				Track_OutGray.Kd   = 70.0f;
				Track_MidGyro.Base = 42;
				Weigth_Flag = 0;
				state       = 11;               
			}
		break;
		
		case 11: // 第三圈通过D点岔路口
			if(Fork_Flag && MyYaw_0_360 > 90)
			{
				Track_MidGyro.Kd   = 43e-4;
				Track_OutGray.Kp   = 250.0f;
				Track_OutGray.Kd   = 50.0f;
				Track_MidGyro.Base = 29;
				state = 12;                   
			}
		break;
		
		case 12: // 第三圈结束
			if(Stop_Flag)
			{
				Zigbee_Send_Bytes(0x3A,0x3A);  
				Track_Flag = 0;                
				Load_Motor_A(0);
				Load_Motor_B(0);
				state = 13;                   
			}
			else
			{
				Weigth_Flag = 0;
			}
		break;
		
		default:
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
				Zigbee_Send_Bytes(0x40,0x40);
				Track_MidGyro.Base = 72;
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
				
				Track_MidGyro.Base = 73;
				Track_MidGyro.Kd = 58e-4;
				
				Track_OutGray.Kp = 970.0f;
				Track_OutGray.Kd = 90.0f;
				Track_Flag = 1;
				Weigth_Flag = 0;
			}
		break;
		case 1: // 第一圈到达B点岔路口
			if(Fork_Flag)
			{
				Weigth_Flag = 1;
				Track_MidGyro.Base = 71;
				Track_MidGyro.Kd = 58e-4;
				
				Track_OutGray.Kp = 980.0f;
				Track_OutGray.Kd = 110.0f;
				
				now_time = u64_ms;
				state = 2;
			}
		break;
		case 2: // 第一圈通过B点岔路口
			if(!Fork_Flag && (u64_ms - now_time > 90))
			{
				Track_MidGyro.Base = 61;
				Track_MidGyro.Kd = 58e-4;
				
				Track_OutGray.Kp = 990.0f;
				Track_OutGray.Kd = 80.0f;
				Weigth_Flag = 0;
				state = 3;
			}
		break;
		case 3:
			if(MyYaw_0_360 > 70)
			{
				Track_MidGyro.Base = 68;
				Track_MidGyro.Kd = 58e-4;
				
				Track_OutGray.Kp = 990.0f;
				Track_OutGray.Kd = 110.0f;
				now_time = u64_ms;
				state = 4;
			}
		break;
		case 4:
			if(u64_ms - now_time > 440)
			{
				Track_MidGyro.Base = 61;
				Track_MidGyro.Kd = 58e-4;
				
				Track_OutGray.Kp = 990.0f;
				Track_OutGray.Kd = 100.0f;
				state = 5;
			}
		break;
		case 5: // 第一圈通过D点岔路口
			if(Fork_Flag && MyYaw_0_360 > 90)
			{
				Track_MidGyro.Base = 69;
				Track_MidGyro.Kd = 58e-4;
				
				Track_OutGray.Kp = 1010.0f;
				Track_OutGray.Kd = 100.0f;
				now_time = u64_ms;
				state = 6;
			}
			else
			{
				Track_MidGyro.Base = 68;
				Track_MidGyro.Kd = 58e-4;
				
				Track_OutGray.Kp = 980.0f;
				Track_OutGray.Kd = 83.0f;
			}
		break;
		case 6: // 第一圈弯道
			if(u64_ms - now_time > 960)
			{
				weight_set[0][0] = -3;
				weight_set[0][1] = -2;
				weight_set[0][2] = -1;
				weight_set[0][3] = 0;
				weight_set[0][4] = 1;
				weight_set[0][5] = 2;
				weight_set[0][6] = 3;
				weight_set[0][7] = 6;
				Track_MidGyro.Base = 34;
				Track_MidGyro.Kd = 61e-4;
				
				Track_OutGray.Kp = 1011.0f;
				Track_OutGray.Kd = 101.0f;
				state = 7;
			}
		break;
		case 7:
			if(Stop_Flag_Aid)
			{
				Track_Flag = 0;
				Zigbee_Send_Bytes(0x4A,0x4A);
				Load_Motor_A(0);
				Load_Motor_B(0);
				Load_Motor_A(-6000);
				Load_Motor_B(-7000);
				delay_ms(200);
				Load_Motor_A(0);
				Load_Motor_B(0);
				
				
				Speed_Left.Out_Now = 0;
				Speed_Right.Out_Now = 0; 
				
				weight_set[0][0] = -1;
				weight_set[0][1] = -1;
				weight_set[0][2] = 0;
				weight_set[0][3] = 1;
				weight_set[0][4] = 1;
				weight_set[0][5] = 2;
				weight_set[0][6] = 3;
				weight_set[0][7] = 5;
				
				
				
				now_time = u64_ms;
				state = 8;
			}
		break;
		case 8:
			if(u64_ms - now_time > 4700)
			{
				Totol_Weight = 3;
				Track_MidGyro.Base = 68;
				Track_MidGyro.Kd = 62e-4;
				
				Track_OutGray.Kp = 990.0f;
				Track_OutGray.Kd = 110.0f;
				Track_Flag = 1;
				now_time = u64_ms;
				state = 9;
			}
			
			
		break;
		case 9:
			if(u64_ms - now_time > 100 && u64_ms - now_time < 300)
			{
				weight_set[0][0] = -4;
				weight_set[0][1] = -2;
				weight_set[0][2] = -1;
				weight_set[0][3] = 0;
				weight_set[0][4] = 1;
				weight_set[0][5] = 2;
				weight_set[0][6] = 3;
				weight_set[0][7] = 6;
				Zigbee_Send_Bytes(0x4B,0x4B);
			}
			else if(u64_ms - now_time > 300)
			{
				Track_MidGyro.Base = 63;
				Track_MidGyro.Kd = 62e-4;
				
				Track_OutGray.Kp = 996.0f;
				Track_OutGray.Kd = 100.0f;
				state = 10;
			}
			else
			{
				Totol_Weight = 0;
			}
		break;
		case 10:
			if(floatAbs(MyYaw_180_180) < 50)
			{
				Track_MidGyro.Base = 68;
				Track_MidGyro.Kd = 58e-4;
				
				Track_OutGray.Kp = 970.0f;
				Track_OutGray.Kd = 80.0f;
				now_time = u64_ms;
				state = 11;
			}
		break;
		case 11:
			if(u64_ms - now_time > 200)
			{
				Track_MidGyro.Base = 71;
				Track_MidGyro.Kd = 58e-4;
				
				Track_OutGray.Kp = 1010.0f;
				Track_OutGray.Kd = 110.0f;
				Weigth_Flag = 0;
				state = 12;
			}
		break;
		case 12:
			if(Stop_Flag)
			{
				Zigbee_Send_Bytes(0x4C,0x4C);
				Track_Flag = 0;
				time[1] = u64_ms - 5000;
				Load_Motor_A(-8000);
				Load_Motor_B(-8000);
				delay_ms(200);
				Load_Motor_A(0);
				Load_Motor_B(0);
				
				speed_mpers = speed_calculate(time, 1, 0);
				state = 13;
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
				
				Track_MidGyro.Base = 85;
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
				Track_MidGyro.Base = 85;
				Track_MidGyro.Kd = 58e-4;
				
				Track_OutGray.Kp = 970.0f;
				Track_OutGray.Kd = 90.0f;
				Track_Flag = 1;
				Weigth_Flag = 0;
			}
		break;
		case 1: // 第一圈到达B点岔路口
			if(Fork_Flag)
			{
				Weigth_Flag = 1;
				Track_MidGyro.Base = 85;
				Track_MidGyro.Kd = 58e-4;
				
				Track_OutGray.Kp = 980.0f;
				Track_OutGray.Kd = 110.0f;
				
				now_time = u64_ms;
				state = 2;
			}
		break;
		case 2: // 第一圈通过B点岔路口
			if(!Fork_Flag && (u64_ms - now_time > 60))
			{
				Track_MidGyro.Base = 73;
				Track_MidGyro.Kd = 58e-4;
				
				Track_OutGray.Kp = 990.0f;
				Track_OutGray.Kd = 40.0f;
				Weigth_Flag = 0;
				state = 3;
			}
		break;
		case 3:
			if(MyYaw_0_360 > 70)
			{
				Track_MidGyro.Base = 73;
				Track_MidGyro.Kd = 58e-4;
				
				Track_OutGray.Kp = 990.0f;
				Track_OutGray.Kd = 110.0f;
				now_time = u64_ms;
				state = 4;
			}
		break;
		case 4:
			if(u64_ms - now_time > 500)
			{
				Track_MidGyro.Base = 62;
				Track_MidGyro.Kd = 58e-4;
				
				Track_OutGray.Kp = 990.0f;
				Track_OutGray.Kd = 100.0f;
				state = 5;
			}
		break;
		case 5: // 第一圈通过D点岔路口
			if(Fork_Flag && MyYaw_0_360 > 90)
			{
				Track_MidGyro.Base = 81;
				Track_MidGyro.Kd = 58e-4;
				
				Track_OutGray.Kp = 1010.0f;
				Track_OutGray.Kd = 100.0f;
				now_time = u64_ms;
				state = 6;
			}
			else
			{
				Track_MidGyro.Base = 65;
				Track_MidGyro.Kd = 58e-4;
				
				Track_OutGray.Kp = 980.0f;
				Track_OutGray.Kd = 83.0f;
			}
		break;
		case 6: // 第一圈弯道
			if(u64_ms - now_time > 670)
			{
				weight_set[0][0] = -3;
				weight_set[0][1] = -1;
				weight_set[0][2] = 0;
				weight_set[0][3] = 2;
				weight_set[0][4] = 3;
				weight_set[0][5] = 3;
				weight_set[0][6] = 3;
				weight_set[0][7] = 5;
				Track_MidGyro.Base = 42;
				Track_MidGyro.Kd = 61e-4;
				
				Track_OutGray.Kp = 1310.0f;
				Track_OutGray.Kd = 130.0f;
				state = 7;
			}
		break;
		case 7:
			if(Stop_Flag_Aid)
			{
				weight_set[0][0] = -5;
				weight_set[0][1] = -3;
				weight_set[0][2] = -1;
				weight_set[0][3] = 0;
				weight_set[0][4] = 1;
				weight_set[0][5] = 1;
				weight_set[0][6] = 3;
				weight_set[0][7] = 5;
				Track_Flag = 0;
				Load_Motor_A(-6000);
				Load_Motor_B(-6000);
				delay_ms(150);
				Load_Motor_A(0);
				Load_Motor_B(0);
				now_time = u64_ms;
				state = 8;
			}
		break;
		case 8:
			if(u64_ms - now_time > 5000)
			{
				Track_Flag = 1;
				Track_MidGyro.Base = 75;
				Track_MidGyro.Kd = 62e-4;
				
				Track_OutGray.Kp = 990.0f;
				Track_OutGray.Kd = 110.0f;
				now_time = u64_ms;
				state = 9;
			}
		break;
		case 9:
			if(u64_ms - now_time > 300)
			{
				Track_MidGyro.Base = 62;
				Track_MidGyro.Kd = 62e-4;
				
				Track_OutGray.Kp = 990.0f;
				Track_OutGray.Kd = 110.0f;
				state = 10;
			}
		break;
		case 10:
			if(floatAbs(MyYaw_180_180) < 50)
			{
				Track_MidGyro.Base = 77;
				Track_MidGyro.Kd = 58e-4;
				
				Track_OutGray.Kp = 970.0f;
				Track_OutGray.Kd = 80.0f;
				now_time = u64_ms;
				state = 11;
			}
		break;
		case 11:
			if(u64_ms - now_time > 200)
			{
				Track_MidGyro.Base = 72;
				Track_MidGyro.Kd = 58e-4;
				
				Track_OutGray.Kp = 1010.0f;
				Track_OutGray.Kd = 110.0f;
				Weigth_Flag = 0;
				state = 12;
			}
		break;
		case 12:
			if(Stop_Flag)
			{
				Track_Flag = 0;
				time[1] = u64_ms - 5000;
				Load_Motor_A(-8000);
				Load_Motor_B(-8000);
				delay_ms(200);
				Load_Motor_A(0);
				Load_Motor_B(0);
				speed_mpers = speed_calculate(time, 1, 0);
				state = 13;
			}
		break;
	}


}



