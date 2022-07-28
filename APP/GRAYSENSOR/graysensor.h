/*
 * graysensor.h
 *
 *  Created on: 2022Äê7ÔÂ28ÈÕ
 *      Author: TOTHTOT
 */

#ifndef APP_GRAYSENSOR_GRAYSENSOR_H_
#define APP_GRAYSENSOR_GRAYSENSOR_H_

#include "driverlib.h"

#define MID_GRAYSENSOR GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN5)
#define LIFT_GRAYSENSOR GPIO_getInputPinValue(GPIO_PORT_P4, GPIO_PIN3)
#define RIGHT_GRAYSENSOR GPIO_getInputPinValue(GPIO_PORT_P7, GPIO_PIN4)


short Car_Staright_Control(void);


#endif /* APP_GRAYSENSOR_GRAYSENSOR_H_ */
