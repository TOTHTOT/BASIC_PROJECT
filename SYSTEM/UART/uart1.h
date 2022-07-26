/*
 * uart1.h
 *
 *  Created on: 2022Äê7ÔÂ10ÈÕ
 *      Author: TOTHTOT
 */

#ifndef SYSTEM_UART_UART1_H_
#define SYSTEM_UART_UART1_H_
#include "driverlib.h"
#include <stdio.h>
#include <string.h>
char Uart_Init(uint16_t baseAddress, uint32_t Baudrate);
void u1_printf(unsigned char *ptr);    //Send string.




#endif /* SYSTEM_UART_UART1_H_ */
