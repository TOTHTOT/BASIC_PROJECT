/*
 * exit.h
 *
 *  Created on: 2022��7��11��
 *      Author: TOTHTOT
 */

#ifndef APP_EXIT_EXIT_H_
#define APP_EXIT_EXIT_H_

#include "driverlib.h"

#define M1_ENCODE_A GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN0)
#define M1_ENCODE_B GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN0)

#define M2_ENCODE_A GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN2)
#define M2_ENCODE_B GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN1)

#define M3_ENCODE_A GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN6)
#define M3_ENCODE_B GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN2)

#define M4_ENCODE_A GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN3)
#define M4_ENCODE_B GPIO_getInputPinValue(GPIO_PORT_P6, GPIO_PIN3)


extern uint8_t car_state;

void EXIT_Init(void);




#endif /* APP_EXIT_EXIT_H_ */
