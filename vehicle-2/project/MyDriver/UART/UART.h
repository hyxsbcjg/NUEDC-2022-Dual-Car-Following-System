#ifndef __UART_H
#define __UART_H

#include "ti_msp_dl_config.h"






void UART1_SendString(char *str);





extern char TxBuf[128];
void UART2_SendString(char *str);
int my_printf(const char *format, ...);
void Zigbee_Send_Bytes(char buf1,char buf2);

void UART_Zigbee_Proc(void);
void UART_K230_Proc(void);

#endif
