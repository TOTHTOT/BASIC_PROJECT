/*
 * tb6612.h
 *
 *  Created on: 2022Äê7ÔÂ16ÈÕ
 *      Author: TOTHTOT
 */

#ifndef APP_TB6612FNG_TB6612_H_
#define APP_TB6612FNG_TB6612_H_

#include "driverlib.h"

typedef enum
{
    forward,
    retreat,
    turn_left,
    turn_right,
    stop
}E_CAR_DIRECTION;

#define M1_A1_H GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);
#define M1_A1_L GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
#define M1_A2_H GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);
#define M1_A2_L GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);

#define M2_B1_H GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);
#define M2_B1_L GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
#define M2_B2_H GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);
#define M2_B2_L GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);

#define M3_A1_H GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);
#define M3_A1_L GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
#define M3_A2_H GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);
#define M3_A2_L GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);

#define M4_B1_H GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
#define M4_B1_L GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
#define M4_B2_H GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN2);
#define M4_B2_L GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2);
void Car_Init(void);
void Car_Direction(E_CAR_DIRECTION direction);

#endif /* APP_TB6612FNG_TB6612_H_ */
