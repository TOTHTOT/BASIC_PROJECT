/*
 * uart1.c
 *
 *  Created on: 2022��7��10��
 *      Author: TOTHTOT
 */
#include "uart1.h"
#include "stdio.h"
#include "tb6612.h"
#include "stdarg.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "graysensor.h"
#include "beep.h"
#include "oled.h"
extern uint8_t CAR;
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

void u0_printf(unsigned char *ptr)    //Send string.
{
    while(*ptr != '\0')
    {
        USCI_A_UART_transmitData(USCI_A0_BASE,*ptr);
        ptr++;
    }
}


//******************************************************************************
//
//This is the USCI_A0 interrupt vector service routine.
//
//******************************************************************************
uint8_t USART0_RX_BUF[USART0_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.
uint16_t USART0_RX_STA = 0; //����״̬���
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR (void)
{
    uint8_t receivedData = 0;
    uint8_t temp[3];
    uint8_t *p;
//    printf("USCI_A0_ISR\r\n");
    switch (__even_in_range(UCA0IV,4))
    {
    //Vector 2 - RXIFG
    case 2:
        receivedData = USCI_A_UART_receiveData(USCI_A0_BASE);
        USCI_A_UART_transmitData(USCI_A0_BASE, receivedData);
//        printf("USCI_A0_ISR\r\n");
//        if (receivedData == 0x01)
//        {
//            // ���յ�ͷ֡0xfd,��ͷ��ʼ����
//            USART0_RX_STA = 0;
//        }
        if ((USART0_RX_STA & 0x8000) == 0) //����δ���
        {
            if (USART0_RX_STA & 0x4000) //���յ���0x0d
            {
                if (receivedData != 0x0a)
                    USART0_RX_STA = 0; //���մ���,���¿�ʼ
                else
                    USART0_RX_STA |= 0x8000; //���������
            }
            else //��û�յ�0X0D
            {
                if (receivedData == 0x0d)
                    USART0_RX_STA |= 0x4000;
                else
                {
                    USART0_RX_BUF[USART0_RX_STA & 0X3FFF] = receivedData;
                    USART0_RX_STA++;
                    if (USART0_RX_STA > (USART0_REC_LEN - 1))
                        USART0_RX_STA = 0; //�������ݴ���,���¿�ʼ����
                }
            }
        }
        //�����������Ӧ����
        if ((USART0_RX_STA & 0x8000))
        {
            // u0_printf("�������\r\n");
            if(p = strstr(USART0_RX_BUF, "so")!=NULL)
            {
                 p++;
                 memcpy(temp, p, 3);
                graysensor = atoi(temp);
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    printf("gray:%d\r\n", graysensor);
            }
            else if(p = strstr(USART0_RX_BUF, "go")!=NULL)
            {
//                Car_Go_Speed(&Car_1, 200);
            }
            else if (p = strstr(USART0_RX_BUF, "stop") != NULL)
            {
                Car_Stop(&Car_1);
            }
//            if()
//            printf("��������Ϊ:%s", USART_RX_BUF);
            if(USART0_RX_BUF[0]==0x01)
            {
                if(CAR == 1)
                {
                    Get_Data_From_Buf(USART0_RX_BUF, 3, ",", &bluetooth);

                }
                if(CAR == 2)
                {
                    Get_Data_From_Buf(USART0_RX_BUF, 3, ",", &bluetooth);
                    if(bluetooth.stop_flag == 1)
                    {
//                        beep_en = 1;
                        Car_Direction(stop, 1);
                        Car_Direction(stop, 2);
                    }
                }
//                if(OPENMV_Data.L_or_R == 1)
//                {
////                  huidu_l_en = huidu_r_en = 0;
////                  beep_en= 1;
//
//                  printf("���յ�ֹͣ�Ҷ�ָ��\r\n");
//                }
//                if(OPENMV_Data.L_or_R == 1)
//                {
//                    huidu_l_en = huidu_r_en = 0;
//                    beep_en= 1;
//                    printf("���յ�ֹͣ�Ҷ�ָ��\r\n");
//                }

//                printf("LoR:%d\r\n", OPENMV_Data.x);
//                printf("y:%d\r\n", OPENMV_Data.output);
            }

            USART0_RX_STA = 0;
            memset((char*) USART0_RX_BUF, '0', strlen(USART0_RX_BUF));
        }
        break;
    default:
        break;
    }
}

extern float M1_OUTPWM, M2_OUTPWM;
extern float Target_value, Actual_value;
extern float Target_value2, Actual_value2;
//******************************************************************************
//
//This is the USCI_A1 interrupt vector service routine.
//
//******************************************************************************
uint8_t USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.
uint16_t USART_RX_STA = 0; //����״̬���
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR (void)//����ͷ��msp
{
    uint8_t receivedData = 0;
    uint8_t str[50];
    switch (__even_in_range(UCA1IV, 4))
    {
    //Vector 2 - RXIFG
    case 2:
        receivedData = USCI_A_UART_receiveData(USCI_A1_BASE);
//        USCI_A_UART_transmitData(USCI_A1_BASE, receivedData);
        if (receivedData == 0x01)
        {
            // ���յ�ͷ֡0xfd,��ͷ��ʼ����
            USART_RX_STA = 0;
        }
        if ((USART_RX_STA & 0x8000) == 0) //����δ���
        {
            if (USART_RX_STA & 0x4000) //���յ���0x0d
            {
                if (receivedData != 0x0a)
                    USART_RX_STA = 0; //���մ���,���¿�ʼ
                else
                    USART_RX_STA |= 0x8000; //���������
            }
            else //��û�յ�0X0D
            {
                if (receivedData == 0x0d)
                    USART_RX_STA |= 0x4000;
                else
                {
                    USART_RX_BUF[USART_RX_STA & 0X3FFF] = receivedData;
                    USART_RX_STA++;
                    if (USART_RX_STA > (USART_REC_LEN - 1))
                        USART_RX_STA = 0; //�������ݴ���,���¿�ʼ����
                }
            }
        }
        //�����������Ӧ����
        if((USART_RX_STA & 0x8000))
        {
//            printf("��������Ϊ:%s", USART_RX_BUF);
            if (USART_RX_BUF[0] == 0x01)
            {
                if(CAR == 1)
                {
                    Get_Data_From_Buf(USART_RX_BUF, 3, ",", &OPENMV_Data);
                    if (OPENMV_Data.stop_flag == 1)
                    {
//                        printf("ֹͣ\r\n");
                        beep_en = 1;
                        //                    01 2c 30 30 32 2c 31 34 33 2c 31 2c 0d 0a
                        sprintf(str, "%c,%d,%d,%d,\r\n", 0x01, 0, 0, 1);
                        u0_printf(str);
                        Car_Direction(stop, 1);
                        Car_Direction(stop, 2);
                        OPENMV_Data.stop_flag = 0;
                    }
                    else if (OPENMV_Data.stop_flag == 0)
                    {
                        sprintf(str, "%c,%d,%d,%d,\r\n", 0x01, 0, 0, 0);
                        u0_printf(str);
                    }

                }
                if(CAR == 2)
                {
                    Get_Data_From_Buf(USART_RX_BUF, 3, ",", &OPENMV_Data);
                    if (OPENMV_Data.distance < 150) //���泵����С��20cm�ͼ���
                    {
                        //С��ʱ��������Ŀ���ٶ�СЩ
                        printf("ֹͣ\r\n");
//                        speed_pid_m1.target_val = 0;
//                        speed_pid_m2.target_val = 0;
                        Car_Direction(stop, 1);
                        Car_Direction(stop, 2);
                    }
                    if (OPENMV_Data.distance < 200) //���泵����С��20cm�ͼ���
                    {
//
//                        speed_pid_m1.target_val = 0;
//                        speed_pid_m2.target_val = 0;
                        //                    if()
                        //С��ʱ��������Ŀ���ٶ�СЩ
                        printf("����\r\n");
                        //                    Target_value = Target_value - 150;
                        //                    Target_value2 = Target_value2 - 150;
                        if(OPENMV_Data.distance>500)
                        {
                            OPENMV_Data.distance = 500;
                        }
//                        Timer_A_setCompareValue(
//                                TIMER_A0_BASE,
//                                TIMER_A_CAPTURECOMPARE_REGISTER_2,
//                                (int) M2_OUTPWM +(int)(0.15*(OPENMV_Data.distance-200)));
//                        Timer_A_setCompareValue(
//                                TIMER_A0_BASE,
//                                TIMER_A_CAPTURECOMPARE_REGISTER_1,
//                                (int) M2_OUTPWM + (int)(0.15*(OPENMV_Data.distance-200)));
                        oled_change_data_en = 1;
                    }
                }
//                //��ͷ�� �͸��泵��times��һ��
//                if(OPENMV_Data.distance < 200) //���泵����С��20cm�ͼ���
//                {
////                    if()
//                    //С��ʱ��������Ŀ���ٶ�СЩ
//                    printf("����\r\n");
////                    Target_value = Target_value - 150;
////                    Target_value2 = Target_value2 - 150;
//                    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2,(int)M2_OUTPWM-200 );
//                    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1,(int)M2_OUTPWM -200);
//                }
//
//                if(OPENMV_Data.stop_flag == 1)
//                {
//                    printf("ֹͣ\r\n");
//                    beep_en = 1;
////                    01 2c 30 30 32 2c 31 34 33 2c 31 2c 0d 0a
//                    sprintf(str, "%c,%d,%d,%d,\r\n", 0x01, 0, 0, 1);
//                    u0_printf(str);
//                    Car_Direction(stop, 1);
//                    Car_Direction(stop, 2);
//                }
            }
            USART_RX_STA = 0;
            memset((char*)USART_RX_BUF, '0', strlen(USART_RX_BUF));
        }
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

S_CAMERA_H bluetooth;
S_CAMERA_H OPENMV_Data;
/**
 * @name: Get_Data_From_Buf
 * @msg: �ӻ����н�������,����һ���������Ϊ6λ��
 * @param {uint8_t} *buf Ҫ�������ַ���
 * @param {uint8_t} times ��������
 * @param {uint8_t} key_word �����ؼ���
 * @param {S_CAMERA_H} *camera ��Ž�����ɵ����ݵĽṹ��
 * @return {*}
 */
void Get_Data_From_Buf(uint8_t *buf, uint8_t times, uint8_t *key_word, S_CAMERA_H *camera)
{
    uint8_t *temp_p;     //��һ�� "key_word"��λ��
    uint8_t *temp_pp;    //�ڶ��� "key_word"��λ��
    uint8_t temp[6];     //�����������ַ���
    uint8_t i;
    for (i = 0; i < times; i++)
    {
        // u1_printf("i:%d\r\n", i);
        // 01 2c 31 38 32 2c 32 39 2c 0d 0a
        if((temp_p = (uint8_t *)strstr(( char*)buf, ( char*)key_word))!=NULL) //Ѱ�ҵ�һ�� "key_word"
        {
            // u1_printf("11\r\n");
            temp_p++;
            if((temp_pp = (uint8_t *)strstr(( char*)temp_p, ( char*)key_word))!= NULL)//Ѱ�ҵڶ��� "key_word",����еĻ�
            {
                // u1_printf("22\r\n");
                memcpy(temp, temp_p, temp_pp- temp_p);
                if(i == 0)
                    camera->distance = atoi(( char*)temp);         //�ַ���תΪ����
                else if(i == 1)
                    camera->output = atoi(( char*)temp);         //�ַ���ת����
                else if(i==2)
                    camera->stop_flag = atoi((char*)temp);
                else
                    printf("û�ж������������Get_Data_From_Buf\r\n");
                // u1_printf("1:temp:%s, x:%d, y:%d\r\ntemp_p:%s, temp_pp:%s\r\n", temp, camera->x, camera->y, temp_p, temp_pp);
                buf = temp_pp;
                // u1_printf("2:temp:%s, x:%d, y:%d\r\ntemp_p:%s, temp_pp:%s\r\n", temp, camera->x, camera->y, temp_p, temp_pp);
                memset(temp, '\0', sizeof(temp));
            }
            else    //���ݸ�ʽ��ȷ�ǲ���ִ�е���
            {
                printf("���ݸ�ʽ����\r\n");

            }
        }
        else    //buf��û�йؼ���
        {
            printf("cannot find key_word!!!\r\n");
        }
    }
}



