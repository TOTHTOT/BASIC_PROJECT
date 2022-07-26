/*
 * delay.h
 *
 *  Created on: 2022Äê7ÔÂ10ÈÕ
 *      Author: TOTHTOT
 */

#ifndef SYSTEM_DELAY_DELAY_H_
#define SYSTEM_DELAY_DELAY_H_

#include "driverlib.h"

#define CPU_F ((double)24000000)

#define delay_us(x) __delay_cycles((unsigned long)(CPU_F*(double)x/1000000.0))

#define delay_ms(x) __delay_cycles((unsigned long)(CPU_F*(double)x/1000.0))

void delay_init(void);




#endif /* SYSTEM_DELAY_DELAY_H_ */
