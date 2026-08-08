#ifndef __QUESTION_H
#define __QUESTION_H

#include "ti_msp_dl_config.h"
#include "board.h"


#define Out_Track 5.484955592
#define In_Track 4.884955592

extern float speed_mpers;
extern float Dist;

extern uint8_t Q_Num;
extern uint8_t Q_State;

//Question的变量
extern uint8_t Q1_Stop;
extern uint8_t Q2_Stop;
extern uint8_t Q3_Stop;
extern uint8_t Q4_Stop_Aid;
extern uint8_t Q4_Stop;

extern uint8_t Q6_Stop;
extern uint8_t Q6_Stop_Flag;
extern uint8_t Q6_Overtake_Flag; //下一圈超车
extern uint8_t Q6_Overtaking_Flag; //立即超车


void Test_Proc(void);
void Q1_Proc(void);
void Q2_Proc(void);
void Q3_Proc(void);
void Q4_Proc(void);
void Q5_Proc(void);
void Q6_Proc(void);











#endif