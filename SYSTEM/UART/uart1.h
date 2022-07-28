/*
 * uart1.h
 *
 *  Created on: 2022��7��10��
 *      Author: TOTHTOT
 */

#ifndef SYSTEM_UART_UART1_H_
#define SYSTEM_UART_UART1_H_
#include "driverlib.h"
#include <stdio.h>
#include <string.h>
#define Speed_0_3_M1 350
#define Speed_0_3_M2 320

#define USART_REC_LEN           200     //�����������ֽ��� 200
#define USART0_REC_LEN           200     //�����������ֽ��� 200
extern uint8_t  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
extern uint16_t USART_RX_STA;                //����״̬���
extern uint8_t  USART0_RX_BUF[USART0_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
extern uint16_t USART0_RX_STA;                //����״̬���

typedef struct
{
    uint16_t L_or_R;
    uint16_t output;
}S_CAMERA_H;
extern S_CAMERA_H OPENMV_Data;

char Uart_Init(uint16_t baseAddress, uint32_t Baudrate);
void u1_printf(unsigned char *ptr);    //Send string.
void u0_printf(unsigned char *ptr);
void Get_Data_From_Buf(uint8_t *buf, uint8_t times, uint8_t *key_word, S_CAMERA_H *camera);



#endif /* SYSTEM_UART_UART1_H_ */
