#ifndef __CONTROL_H
#define __CONTROL_H

#include "ti_msp_dl_config.h"
#include "motor.h"


#define Fixed_Dist 130




typedef struct{
int32_t Target,Actual,Actual_Old,Actual_Old_Old;
float Kp,Ki,Kd;
int32_t Error0,Error1,Error2;
int32_t ErrorInt,Int_Out_Now,Int_Out_Old,Int_Out_End;
int16_t Base;
int16_t Out_Now,Out_Old,Out_End,Out_Delta;//占空比输出
float DifOut,DifOut_Old;
}PID_int;

typedef struct{
float Target,Actual,Actual_Old,Actual_Old_Old;
float Kp,Ki,Kd;
float Error0,Error1,Error2;
float ErrorInt,Int_Out_Now,Int_Out_Old,Int_Out_End;
int16_t Base;
int16_t Out_Now,Out_Old,Out_End,Out_Delta;//占空比输出
float DifOut,DifOut_Old;
}PID_float;

//变量
extern uint64_t u64_ms;
extern float MyYaw_180_180;
extern int16_t MyGyro;
extern uint16_t dist_mm;


extern int32_t SpeedInt_Left;
extern int32_t SpeedInt_Right;
extern volatile uint8_t Speed_Left_Flag;
extern volatile uint8_t Speed_Right_Flag;
extern volatile uint8_t Angle_Gyro_Flag;
extern volatile uint8_t Track_Flag;
extern volatile uint8_t Turn_Flag;


extern PID_int Track_OutGray,Track_InGyro,Track_OutDist;

extern PID_float Turn_OutAngle; //转向环
extern PID_int Turn_InGyro;

void PID_Init(void);
float my_fmod(float a, float b);
float floatAbs(float buf);
void int16_tLimit(int16_t *Actual,int16_t Min,int16_t Max);
void uint16_tLimit(uint16_t *Actual,uint16_t Min,uint16_t Max);



void Turn_PID_Control(void);
void Track_PID_Control(void);
	
void My_GPIO_WriteVal(GPIO_Regs* gpio, uint32_t pins, uint32_t pinsVal);
int16_t int16_tAbs(int16_t buf);
void uint16_tLimit(uint16_t *Actual,uint16_t Min,uint16_t Max);

#endif
