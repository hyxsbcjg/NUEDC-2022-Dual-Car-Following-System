#include "control.h"
#include "encoder.h"
#include "track.h"




PID_int Track_OutGray,Track_MidGyro;//循迹环


//PID_float Turn_OutAngle; //角度串角速度环
//PID_int Turn_InGyro;

PID_int Speed_Left,Speed_Right; //速度环







void My_GPIO_WriteVal(GPIO_Regs* gpio, uint32_t pins, uint32_t pinsVal)
{
	if(pinsVal)
		gpio->DOUTSET31_0 = pins;
	else
		gpio->DOUTCLR31_0 = pins;
}

int16_t int16_tAbs(int16_t buf)
{
	if(buf < 0) return -buf;
	else return buf;
}

int32_t int32_tAbs(int32_t buf)
{
	if(buf < 0) return -buf;
	else return buf;
}

float floatAbs(float buf)
{
	if(buf < 0) return -buf;
	else return buf;
}

void int16_tLimit(int16_t *Actual,int16_t Min,int16_t Max)
{
	if(*Actual > Max)  *Actual = Max;
	if(*Actual < Min)  *Actual = Min;
}

void int32_tLimit(int32_t *Actual,int32_t Min,int32_t Max)
{
	if(*Actual > Max)  *Actual = Max;
	if(*Actual < Min)  *Actual = Min;
}


void uint16_tLimit(uint16_t *Actual,uint16_t Min,uint16_t Max)
{
	if(*Actual > Max)  *Actual = Max;
	if(*Actual < Min)  *Actual = Min;
}

void floatLimit(float *Actual,float Min,float Max)
{
	if(*Actual > Max)  *Actual = Max;
	if(*Actual < Min)  *Actual = Min;
}


// 浮点取模：计算 a mod b，返回值在 [0, b) 或 [0, |b|) 区间
float my_fmod(float a, float b) 
{
    if (b == 0.0f) return 0.0f; // 防止除以0
    int div = (int)(a / b);
    float result = a - (float)div * b;
    
    // 保证结果在 [0, b) 区间
    if (result < 0)
        result += b;
    
    return result;
}


void PID_Init(void)
{
	//循迹环
	Track_OutGray.Kp = 1020.0f;Track_OutGray.Kd = 0;
	Track_MidGyro.Kp = 252e-4;Track_MidGyro.Kd = 45e-4;
	
	//速度环
	Speed_Left.Kp = 322.0f; Speed_Left.Ki = 16.2f;
	Speed_Right.Kp = 320;Speed_Right.Ki = 17.3f;
	Speed_Right.Target = 30;
	Speed_Left.Target = 30;
}





void Speed_Left_PI_Control(void)
{
	if(u64_ms % 10 == 0)
	{
		
		Speed_Left.Actual = (int32_t)Encoder_Get_L();
		
		
		Speed_Left.Error1 = Speed_Left.Error0;
		Speed_Left.Error0 = Speed_Left.Target - Speed_Left.Actual;
		
		Speed_Left.Out_Delta = Speed_Left.Kp*(Speed_Left.Error0 - Speed_Left.Error1) + Speed_Left.Ki*Speed_Left.Error0;
		
		Speed_Left.Out_Now += Speed_Left.Out_Delta;
		
		int16_tLimit(&Speed_Left.Out_Now,PWM_MIN,PWM_MAX);
		
		Load_Motor_A(Speed_Left.Out_Now);
	}
	
}

void Speed_Right_PI_Control(void)
{
	if(u64_ms % 10 == 0)
	{
		
		Speed_Right.Actual = (int32_t)Encoder_Get_R();
		
		
		Speed_Right.Error1 = Speed_Right.Error0;
		Speed_Right.Error0 = Speed_Right.Target - Speed_Right.Actual;
		
		Speed_Right.Out_Delta = Speed_Right.Kp*(Speed_Right.Error0 - Speed_Right.Error1) + Speed_Right.Ki*Speed_Right.Error0;
		
		Speed_Right.Out_Now += Speed_Right.Out_Delta;
		
		int16_tLimit(&Speed_Right.Out_Now,PWM_MIN,PWM_MAX);
		
		Load_Motor_B(Speed_Right.Out_Now);
	}
	
}

void Track_PID_Control(void)
{
	
	if(u64_ms % 20 == 0)
	{
		
		Track_OutGray.Actual = Totol_Weight;
		
		Track_OutGray.Error1 = Track_OutGray.Error0;
		Track_OutGray.Error0 = -Track_OutGray.Actual;
		
		
		Track_OutGray.DifOut_Old = Track_OutGray.DifOut;
		Track_OutGray.DifOut = (7*Track_OutGray.Kd * (Track_OutGray.Error0 - Track_OutGray.Error1) + 3*Track_OutGray.DifOut_Old)*0.1f;
		
		Track_MidGyro.Target = Track_OutGray.Kp * Track_OutGray.Error0 + Track_OutGray.DifOut;

		Track_MidGyro.Actual_Old = Track_MidGyro.Actual;
		Track_MidGyro.Actual = (7*MyGyro + 3*Track_MidGyro.Actual_Old)*0.1f;
		

		
		Track_MidGyro.Error1 = Track_MidGyro.Error0;
		Track_MidGyro.Error0 = (Track_MidGyro.Target - Track_MidGyro.Actual);

	
		
		Track_MidGyro.DifOut_Old = Track_MidGyro.DifOut;
		Track_MidGyro.DifOut = (7*Track_MidGyro.Kd * (Track_MidGyro.Actual_Old - Track_MidGyro.Actual) + 3*Track_MidGyro.DifOut_Old)*0.1f;
		Track_MidGyro.Out_Now = Track_MidGyro.Kp * Track_MidGyro.Error0  + Track_MidGyro.DifOut;
	}
	
	if(u64_ms % 10 == 0)
	{
		
		Speed_Left.Target = Track_MidGyro.Base - Track_MidGyro.Out_Now;
		Speed_Right.Target = Track_MidGyro.Base + Track_MidGyro.Out_Now;
		
		Speed_Left.Actual = (int32_t)Encoder_Get_L();
		Speed_Right.Actual = (int32_t)Encoder_Get_R();
		
		
		
		Speed_Left.Error1 = Speed_Left.Error0;
		Speed_Right.Error1 = Speed_Right.Error0;
		Speed_Left.Error0 = Speed_Left.Target - Speed_Left.Actual;
		Speed_Right.Error0 = Speed_Right.Target - Speed_Right.Actual;
		
		Speed_Left.Out_Delta = Speed_Left.Kp*(Speed_Left.Error0 - Speed_Left.Error1) + Speed_Left.Ki*Speed_Left.Error0;
		Speed_Right.Out_Delta = Speed_Right.Kp*(Speed_Right.Error0 - Speed_Right.Error1) + Speed_Right.Ki*Speed_Right.Error0;
		
		Speed_Left.Out_Now += Speed_Left.Out_Delta;
		Speed_Right.Out_Now += Speed_Right.Out_Delta; 
		
		int16_tLimit(&Speed_Left.Out_Now,PWM_MIN,PWM_MAX);
		int16_tLimit(&Speed_Right.Out_Now,PWM_MIN,PWM_MAX);
		
		Load_Motor_A(Speed_Left.Out_Now);
		Load_Motor_B(Speed_Right.Out_Now);
	}
}