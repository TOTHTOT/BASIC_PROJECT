/*
 * beep.h
 *
 *  Created on: 2022Äê7ÔÂ28ÈÕ
 *      Author: TOTHTOT
 */

#ifndef APP_BEEP_BEEP_H_
#define APP_BEEP_BEEP_H_

#include "driverlib.h"

#define BEEP_LOW GPIO_setOutputLowOnPin(GPIO_PORT_P7, GPIO_PIN0);
#define BEEP_High GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN0);

extern uint8_t beep_en;
extern uint8_t beep_en_on;

void Bep_Init(void);



#endif /* APP_BEEP_BEEP_H_ */
