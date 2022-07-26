/*
 * uart1.c
 *
 *  Created on: 2022��7��10��
 *      Author: TOTHTOT
 */
#include "uart1.h"
/* ���ڳ�ʼ��
 * ����:baseAddress:ΪUSCI_A0_BASE��USCI_A1_BASE, Baudrate:������
 * */
char Uart_Init(uint16_t baseAddress, uint32_t Baudrate)
{
    float UART_Temp = 0;
    USCI_A_UART_initParam huart = { 0 };
    //���������Ÿ���ΪUARTģʽ
    if (baseAddress == USCI_A0_BASE)         //P3.3, P3.4 = USCI_A0 TXD/RXD
    {
        GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3, GPIO_PIN3);
        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P3, GPIO_PIN4);
    }
    else if (baseAddress == USCI_A1_BASE)    //P4.4, P4.5 = USCI_A1 TXD/RXD
    {
        GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P4, GPIO_PIN4);
        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P4, GPIO_PIN5);
    }
    //��Ŀ�겨���ʽϵ�ʱ��UARTѡ��ʱ��ԴΪACLK����֮ѡ��SMCLK
    //����UCS_getACLK/UCS_getSMCLKǰ����ȷ����UCS_setExternalClockSource���������Ѽӵ�SystemClock_Init������
    if (Baudrate <= 9600)
    {
        huart.selectClockSource = USCI_A_UART_CLOCKSOURCE_ACLK;
        UART_Temp = (float) UCS_getACLK() / Baudrate;
    }
    else
    {
        huart.selectClockSource = USCI_A_UART_CLOCKSOURCE_SMCLK;
        UART_Temp = (float) UCS_getSMCLK() / Baudrate;
    }

    if (UART_Temp < 16)  //����Ƶ����С��16ʱ�����õ�Ƶ����������
        huart.overSampling = USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;
    else    //��֮�����ù���������������
    {
        huart.overSampling = USCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;
        UART_Temp /= 16;
    }

    huart.clockPrescalar = (int) UART_Temp;

    if (huart.overSampling == USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION)
    {   //��Ƶ����������   UCBRSx
        huart.secondModReg = (int) ((UART_Temp - huart.clockPrescalar) * 8);
    }
    else
    {   //���������������� UCBRFx
        huart.firstModReg = (int) ((UART_Temp - huart.clockPrescalar) * 16);
    }

    huart.parity = USCI_A_UART_NO_PARITY;
    huart.msborLsbFirst = USCI_A_UART_LSB_FIRST;
    huart.numberofStopBits = USCI_A_UART_ONE_STOP_BIT;
    huart.uartMode = USCI_A_UART_MODE;

    if (STATUS_FAIL == USCI_A_UART_init(baseAddress, &huart))
    {   //��ʼ����Ӧ����
        return STATUS_FAIL;
    }

    //Enable UART module for operation ʹ�ܶ�Ӧ����
    USCI_A_UART_enable(baseAddress);

    //Enable Receive Interrupt ���ô����ж�
    USCI_A_UART_clearInterrupt(baseAddress, USCI_A_UART_RECEIVE_INTERRUPT);
    USCI_A_UART_enableInterrupt(baseAddress, USCI_A_UART_RECEIVE_INTERRUPT);

    return STATUS_SUCCESS;
}


void u1_printf(unsigned char *ptr)    //Send string.
{
    while(*ptr != '\0')
    {
        USCI_A_UART_transmitData(USCI_A1_BASE,*ptr);
        ptr++;
    }
}

//******************************************************************************
//
//This is the USCI_A0 interrupt vector service routine.
//
//******************************************************************************
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR (void)
{
    uint8_t receivedData = 0;
    switch (__even_in_range(UCA0IV,4))
    {
        //Vector 2 - RXIFG
        case 2:
            receivedData = USCI_A_UART_receiveData(USCI_A0_BASE);
            USCI_A_UART_transmitData(USCI_A0_BASE,receivedData);
            break;
        default:
            break;
    }
}

//******************************************************************************
//
//This is the USCI_A1 interrupt vector service routine.
//
//******************************************************************************
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR (void)
{
    uint8_t receivedData = 0;
    switch (__even_in_range(UCA1IV,4))
    {
        //Vector 2 - RXIFG
        case 2:
            receivedData = USCI_A_UART_receiveData(USCI_A1_BASE);
            USCI_A_UART_transmitData(USCI_A1_BASE,receivedData);
            break;
        default:
            break;
    }
}


/*��дprintf*/
int fputc(int ch,FILE *f)
{
    UCA1TXBUF = ch&0xff;
    while(!(UCA1IFG & UCTXIFG));//�ȴ��������
     return ch;
}
int fputs(const char *_ptr, register FILE *_fp)
{
  unsigned int i, len;

  len = strlen(_ptr);

    for (i = 0; i < len; i++)
    {
        UCA1TXBUF = _ptr[i] & 0xff;
        while (!(UCA1IFG & UCTXIFG))
            ;    //�ȴ��������

    }

  return len;
}



