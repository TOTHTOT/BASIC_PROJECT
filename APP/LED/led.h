/*
 * led.h
 *
 *  Created on: 2022Äê7ÔÂ10ÈÕ
 *      Author: TOTHTOT
 */

#ifndef APP_LED_LED_H_
#define APP_LED_LED_H_

#include "driverlib.h"

#define LED1 0x01
#define LED2 0x02
#define ALL_LED 0xff
void Led_Init(uint8_t whitch_led);
void Led_Toggle(uint8_t whitch_led);
void Led_On(uint8_t whitch_led);
void Led_Off(uint8_t whitch_led);


#endif /* APP_LED_LED_H_ */
