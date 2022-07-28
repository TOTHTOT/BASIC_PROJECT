/*
 * beep.c
 *
 *  Created on: 2022Äê7ÔÂ28ÈÕ
 *      Author: TOTHTOT
 */

#include "beep.h"
uint8_t beep_en = 0;
uint8_t beep_en_on = 0;

void Bep_Init(void)
{
    GPIO_setAsOutputPin(GPIO_PORT_P7, GPIO_PIN0);
    GPIO_setOutputHighOnPin(GPIO_PORT_P7, GPIO_PIN0);
}



