/*
 * uart1.c
 *
 *  Created on: 2022年7月10日
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
/* 串口初始化
 * 参数:baseAddress:为USCI_A0_BASE或USCI_A1_BASE, Baudrate:波特率
 * */
char Uart_Init(uint16_t baseAddress, uint32_t Baudrate)
{
    float UART_Temp = 0;
    USCI_A_UART_initParam huart = { 0 };
    //将所用引脚复用为UART模式
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
    //当目标波特率较低时，UART选用时钟源为ACLK，反之选择SMCLK
    //调用UCS_getACLK/UCS_getSMCLK前需正确调用UCS_setExternalClockSource函数，我已加到SystemClock_Init函数中
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

    if (UART_Temp < 16)  //当分频因子小于16时，采用低频波特率设置
        huart.overSampling = USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION;
    else    //反之，采用过采样波特率设置
    {
        huart.overSampling = USCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION;
        UART_Temp /= 16;
    }

    huart.clockPrescalar = (int) UART_Temp;

    if (huart.overSampling == USCI_A_UART_LOW_FREQUENCY_BAUDRATE_GENERATION)
    {   //低频波特率设置   UCBRSx
        huart.secondModReg = (int) ((UART_Temp - huart.clockPrescalar) * 8);
    }
    else
    {   //过采样波特率设置 UCBRFx
        huart.firstModReg = (int) ((UART_Temp - huart.clockPrescalar) * 16);
    }

    huart.parity = USCI_A_UART_NO_PARITY;
    huart.msborLsbFirst = USCI_A_UART_LSB_FIRST;
    huart.numberofStopBits = USCI_A_UART_ONE_STOP_BIT;
    huart.uartMode = USCI_A_UART_MODE;

    if (STATUS_FAIL == USCI_A_UART_init(baseAddress, &huart))
    {   //初始化对应串口
        return STATUS_FAIL;
    }

    //Enable UART module for operation 使能对应串口
    USCI_A_UART_enable(baseAddress);

    //Enable Receive Interrupt 启用串口中断
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
uint8_t USART0_RX_BUF[USART0_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.
uint16_t USART0_RX_STA = 0; //接收状态标记
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
//            // 接收到头帧0xfd,从头开始接收
//            USART0_RX_STA = 0;
//        }
        if ((USART0_RX_STA & 0x8000) == 0) //接收未完成
        {
            if (USART0_RX_STA & 0x4000) //接收到了0x0d
            {
                if (receivedData != 0x0a)
                    USART0_RX_STA = 0; //接收错误,重新开始
                else
                    USART0_RX_STA |= 0x8000; //接收完成了
            }
            else //还没收到0X0D
            {
                if (receivedData == 0x0d)
                    USART0_RX_STA |= 0x4000;
                else
                {
                    USART0_RX_BUF[USART0_RX_STA & 0X3FFF] = receivedData;
                    USART0_RX_STA++;
                    if (USART0_RX_STA > (USART0_REC_LEN - 1))
                        USART0_RX_STA = 0; //接收数据错误,重新开始接收
                }
            }
        }
        //接收完成作相应处理
        if ((USART0_RX_STA & 0x8000))
        {
            // u0_printf("接收完成\r\n");
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
//            printf("接收数据为:%s", USART_RX_BUF);
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
//                  printf("接收到停止灰度指令\r\n");
//                }
//                if(OPENMV_Data.L_or_R == 1)
//                {
//                    huidu_l_en = huidu_r_en = 0;
//                    beep_en= 1;
//                    printf("接收到停止灰度指令\r\n");
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
uint8_t USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.
uint16_t USART_RX_STA = 0; //接收状态标记
#pragma vector=USCI_A1_VECTOR
__interrupt void USCI_A1_ISR (void)//摄像头和msp
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
            // 接收到头帧0xfd,从头开始接收
            USART_RX_STA = 0;
        }
        if ((USART_RX_STA & 0x8000) == 0) //接收未完成
        {
            if (USART_RX_STA & 0x4000) //接收到了0x0d
            {
                if (receivedData != 0x0a)
                    USART_RX_STA = 0; //接收错误,重新开始
                else
                    USART_RX_STA |= 0x8000; //接收完成了
            }
            else //还没收到0X0D
            {
                if (receivedData == 0x0d)
                    USART_RX_STA |= 0x4000;
                else
                {
                    USART_RX_BUF[USART_RX_STA & 0X3FFF] = receivedData;
                    USART_RX_STA++;
                    if (USART_RX_STA > (USART_REC_LEN - 1))
                        USART_RX_STA = 0; //接收数据错误,重新开始接收
                }
            }
        }
        //接收完成作相应处理
        if((USART_RX_STA & 0x8000))
        {
//            printf("接收数据为:%s", USART_RX_BUF);
            if (USART_RX_BUF[0] == 0x01)
            {
                if(CAR == 1)
                {
                    Get_Data_From_Buf(USART_RX_BUF, 3, ",", &OPENMV_Data);
                    if (OPENMV_Data.stop_flag == 1)
                    {
//                        printf("停止\r\n");
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
                    if (OPENMV_Data.distance < 150) //跟随车距离小于20cm就减速
                    {
                        //小于时立马设置目标速度小些
                        printf("停止\r\n");
//                        speed_pid_m1.target_val = 0;
//                        speed_pid_m2.target_val = 0;
                        Car_Direction(stop, 1);
                        Car_Direction(stop, 2);
                    }
                    if (OPENMV_Data.distance < 200) //跟随车距离小于20cm就减速
                    {
//
//                        speed_pid_m1.target_val = 0;
//                        speed_pid_m2.target_val = 0;
                        //                    if()
                        //小于时立马设置目标速度小些
                        printf("减速\r\n");
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
//                //领头车 和跟随车的times不一样
//                if(OPENMV_Data.distance < 200) //跟随车距离小于20cm就减速
//                {
////                    if()
//                    //小于时立马设置目标速度小些
//                    printf("减速\r\n");
////                    Target_value = Target_value - 150;
////                    Target_value2 = Target_value2 - 150;
//                    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2,(int)M2_OUTPWM-200 );
//                    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1,(int)M2_OUTPWM -200);
//                }
//
//                if(OPENMV_Data.stop_flag == 1)
//                {
//                    printf("停止\r\n");
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


/*重写printf*/
int fputc(int ch,FILE *f)
{
    UCA1TXBUF = ch&0xff;
    while(!(UCA1IFG & UCTXIFG));//等待发送完成
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
            ;    //等待发送完成

    }

  return len;
}

S_CAMERA_H bluetooth;
S_CAMERA_H OPENMV_Data;
/**
 * @name: Get_Data_From_Buf
 * @msg: 从缓存中解析数据,解析一个数据最大为6位数
 * @param {uint8_t} *buf 要解析的字符串
 * @param {uint8_t} times 解析次数
 * @param {uint8_t} key_word 解析关键字
 * @param {S_CAMERA_H} *camera 存放解析完成的数据的结构体
 * @return {*}
 */
void Get_Data_From_Buf(uint8_t *buf, uint8_t times, uint8_t *key_word, S_CAMERA_H *camera)
{
    uint8_t *temp_p;     //第一个 "key_word"的位置
    uint8_t *temp_pp;    //第二个 "key_word"的位置
    uint8_t temp[6];     //解析出来的字符串
    uint8_t i;
    for (i = 0; i < times; i++)
    {
        // u1_printf("i:%d\r\n", i);
        // 01 2c 31 38 32 2c 32 39 2c 0d 0a
        if((temp_p = (uint8_t *)strstr(( char*)buf, ( char*)key_word))!=NULL) //寻找第一个 "key_word"
        {
            // u1_printf("11\r\n");
            temp_p++;
            if((temp_pp = (uint8_t *)strstr(( char*)temp_p, ( char*)key_word))!= NULL)//寻找第二个 "key_word",如果有的话
            {
                // u1_printf("22\r\n");
                memcpy(temp, temp_p, temp_pp- temp_p);
                if(i == 0)
                    camera->distance = atoi(( char*)temp);         //字符串转为整型
                else if(i == 1)
                    camera->output = atoi(( char*)temp);         //字符串转整型
                else if(i==2)
                    camera->stop_flag = atoi((char*)temp);
                else
                    printf("没有定义第三个参数Get_Data_From_Buf\r\n");
                // u1_printf("1:temp:%s, x:%d, y:%d\r\ntemp_p:%s, temp_pp:%s\r\n", temp, camera->x, camera->y, temp_p, temp_pp);
                buf = temp_pp;
                // u1_printf("2:temp:%s, x:%d, y:%d\r\ntemp_p:%s, temp_pp:%s\r\n", temp, camera->x, camera->y, temp_p, temp_pp);
                memset(temp, '\0', sizeof(temp));
            }
            else    //数据格式正确是不会执行到这
            {
                printf("数据格式错误\r\n");

            }
        }
        else    //buf中没有关键字
        {
            printf("cannot find key_word!!!\r\n");
        }
    }
}



