/*
 * tb6612.c
 *
 *  Created on: 2022年7月16日
 *      Author: TOTHTOT
 */
#include "tb6612.h"

void Car_Init(void)
{
    // M1_Ain1
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
    // M1_Ain2
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);


    // M2_Bin1
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
    // M2_Bin2
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);


    // M3_Ain1
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
    // M3_Ain2
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);


    // M4_Bin1
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
    // M4_Bin2
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2);

}

/* Car_Direction
 * 描述:小车方向控制
 * 参数: direction枚举变量 传入小车方向
 * */
void Car_Direction(E_CAR_DIRECTION direction)
{
    switch (direction)
    {
    case forward:
        M1_A1_H;
        M1_A2_L;    //前进
        M2_B1_H;
        M2_B2_L;    //前进
        M3_A1_H;
        M3_A2_L;    //前进
        M4_B1_H;
        M4_B2_L;    //前进
        break;
    case retreat:
        M1_A1_L;
        M1_A2_H;    //后退
        M2_B1_L;
        M2_B2_H;    //后退
        M3_A1_L;
        M3_A2_H;    //后退
        M4_B1_L;
        M4_B2_H;    //后退
        break;
    case turn_left:
        M1_A1_L;
        M1_A2_H;    //后退
        M2_B1_L;
        M2_B2_H;    //后退
        M3_A1_H;
        M3_A2_L;    //前进
        M4_B1_H;
        M4_B2_L;    //前进
        break;
    case turn_right:
        M1_A1_H;
        M1_A2_L;    //前进
        M2_B1_H;
        M2_B2_L;    //前进
        M3_A1_L;
        M3_A2_H;    //后退
        M4_B1_L;
        M4_B2_H;    //后退
        break;
    case stop:
        M1_A1_L;
        M1_A2_L;    //停止
        M2_B1_L;
        M2_B2_L;    //停止
        M3_A1_L;
        M3_A2_L;    //停止
        M4_B1_L;
        M4_B2_L;    //停止
        break;
    default:
        break;
    }
}

