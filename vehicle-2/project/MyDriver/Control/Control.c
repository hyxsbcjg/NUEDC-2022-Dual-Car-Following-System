#include "control.h"
#include "encoder.h"
#include "track.h"




PID_int Track_OutGray,Track_InGyro,Track_OutDist;//循迹环

PID_float Turn_OutAngle; //转向环
PID_int Turn_InGyro;










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
	Track_OutDist.Kp = 2.1f;Track_OutDist.Ki = 0.3f;
	Track_OutGray.Kp = 224.0f;Track_OutGray.Kd = 0;
	Track_InGyro.Kp = 2.26f;Track_InGyro.Kd = 0.77f;
	Track_InGyro.Base = 0;
	
	//转向环
	Turn_OutAngle.Kp = 125.0f;Turn_OutAngle.Ki = 3.8f;
	Turn_InGyro.Kp = 2.26f;Turn_InGyro.Kd = 0.77f;
	Turn_InGyro.Base = 0;

}



void Turn_PID_Control(void)
{
	if(u64_ms % 20 == 0)
	{
//		Turn_OutAngle.Target = 0;
		Turn_OutAngle.Actual = MyYaw_180_180;
		
		Turn_OutAngle.Error1 = Turn_OutAngle.Error0;
		Turn_OutAngle.Error0 = Turn_OutAngle.Target - Turn_OutAngle.Actual;
		while(Turn_OutAngle.Error0 > 180) Turn_OutAngle.Error0 -= 360;
		while(Turn_OutAngle.Error0 < -180) Turn_OutAngle.Error0 += 360;
		
//		Turn_OutAngle.DifOut_Old = Turn_OutAngle.DifOut;
//		Turn_OutAngle.DifOut = (7*Turn_OutAngle.Kd * (Turn_OutAngle.Error0 - Turn_OutAngle.Error1) + 3*Turn_OutAngle.DifOut_Old)*0.1f;
		
		if(floatAbs(Turn_OutAngle.Error0) < 5 )
			Turn_OutAngle.ErrorInt += Turn_OutAngle.Error0;
		floatLimit(&Turn_OutAngle.ErrorInt,-1000,1000);
		
		
		
		Turn_InGyro.Target = Turn_OutAngle.Kp*Turn_OutAngle.Error0 + Turn_OutAngle.Ki*Turn_OutAngle.ErrorInt;
	}
	
	if(u64_ms % 5 == 0)
	{
		
		Turn_InGyro.Actual_Old = Turn_InGyro.Actual;
		Turn_InGyro.Actual = (7*MyGyro + 3*Turn_InGyro.Actual_Old)*0.1f;
		

		
		Turn_InGyro.Error1 = Turn_InGyro.Error0;
		Turn_InGyro.Error0 = (Turn_InGyro.Target - Turn_InGyro.Actual);

	
		
		Turn_InGyro.DifOut_Old = Turn_InGyro.DifOut;
		Turn_InGyro.DifOut = (7*Turn_InGyro.Kd * (Turn_InGyro.Actual_Old - Turn_InGyro.Actual) + 3*Turn_InGyro.DifOut_Old)*0.1f;
		Turn_InGyro.Out_Now = Turn_InGyro.Kp * Turn_InGyro.Error0  + Turn_InGyro.DifOut;
		
		
		int16_t Pwm_Left = Turn_InGyro.Base - Turn_InGyro.Out_Now;
		int16_t Pwm_Right = Turn_InGyro.Base + Turn_InGyro.Out_Now;
		
		int16_tLimit(&Pwm_Left,PWM_MIN,PWM_MAX);
		int16_tLimit(&Pwm_Right,PWM_MIN,PWM_MAX);
		
		Load_Motor_A(Pwm_Left);
		Load_Motor_B(Pwm_Right);
	}
}



void Track_PID_Control(void)
{
	
	if(u64_ms % 20 == 0)
	{
		Track_OutDist.Target = 200 + Fixed_Dist;
		Track_OutDist.Actual = dist_mm;
		
		Track_OutDist.Error1 = Track_OutDist.Error0;
		Track_OutDist.Error0 = Track_OutDist.Target - Track_OutDist.Actual;
		
		if(int32_tAbs(Track_OutDist.Error0) < 100 )
			Track_OutDist.ErrorInt += Track_OutDist.Error0;
		int32_tLimit(&Track_OutDist.ErrorInt,-800,800);
		
		
		
		Track_OutDist.Out_Now = Track_OutDist.Kp*Track_OutDist.Error0 + Track_OutDist.Ki*Track_OutDist.ErrorInt;
	}
	
	if(u64_ms % 10 == 0)
	{
		
		Track_OutGray.Actual = Totol_Weight;
		
		Track_OutGray.Error1 = Track_OutGray.Error0;
		Track_OutGray.Error0 = -Track_OutGray.Actual;
		
		Track_OutGray.DifOut_Old = Track_OutGray.DifOut;
		Track_OutGray.DifOut = (7*Track_OutGray.Kd * (Track_OutGray.Error0 - Track_OutGray.Error1) + 3*Track_OutGray.DifOut_Old)*0.1f;
		
		Track_InGyro.Target = Track_OutGray.Kp * Track_OutGray.Error0 + Track_OutGray.DifOut;

		
	}
	
	
	
	if(u64_ms % 5 == 0)
	{
		
		Track_InGyro.Actual_Old = Track_InGyro.Actual;
		Track_InGyro.Actual = (7*MyGyro + 3*Track_InGyro.Actual_Old)*0.1f;
		

		
		Track_InGyro.Error1 = Track_InGyro.Error0;
		Track_InGyro.Error0 = (Track_InGyro.Target - Track_InGyro.Actual);

	
		
		Track_InGyro.DifOut_Old = Track_InGyro.DifOut;
		Track_InGyro.DifOut = (7*Track_InGyro.Kd * (Track_InGyro.Actual_Old - Track_InGyro.Actual) + 3*Track_InGyro.DifOut_Old)*0.1f;
		Track_InGyro.Out_Now = Track_InGyro.Kp * Track_InGyro.Error0  + Track_InGyro.DifOut;
		
		
		int32_t Pwm_Left = Track_InGyro.Base - Track_InGyro.Out_Now - Track_OutDist.Out_Now;
		int32_t Pwm_Right = Track_InGyro.Base + Track_InGyro.Out_Now - Track_OutDist.Out_Now;
		
		int32_tLimit(&Pwm_Left,PWM_MIN,PWM_MAX);
		int32_tLimit(&Pwm_Right,PWM_MIN,PWM_MAX);
		
		Load_Motor_A(Pwm_Left);
		Load_Motor_B(Pwm_Right);
	}
}