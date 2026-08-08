#ifndef __QUESTION_H
#define __QUESTION_H

#include "ti_msp_dl_config.h"
#include "board.h"


#define Out_Track 5.484955592
#define In_Track 4.884955592

extern float speed_mpers;
extern float Dist;

extern uint8_t Q6_Stop_Flag;


void Test_Proc(void);
void Q1_Proc(void);
void Q2_Proc(void);
void Q3_Proc(void);
void Q4_Proc(void);
void Q5_Proc(void);










#endif