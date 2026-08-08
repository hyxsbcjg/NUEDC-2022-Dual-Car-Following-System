#include "peripheral.h"



//1¿ªÆô 0¹Ø±Õ
void Set_Prompt(uint8_t Val)
{
	if(Val)
		My_GPIO_WriteVal(Peripheral_PORT,Peripheral_In_PIN,0);
	else
		My_GPIO_WriteVal(Peripheral_PORT,Peripheral_In_PIN,1);
}