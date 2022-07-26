/*
 * iic.h
 *
 *  Created on: 2022Äê7ÔÂ11ÈÕ
 *      Author: TOTHTOT
 */

#ifndef APP_IIC_IIC_H_
#define APP_IIC_IIC_H_

#include "driverlib.h"

void IIC_Init(unsigned char slave_adderss);

void IIC_RD_Bytes(uint8_t RegAddr, uint8_t length, uint8_t* data);
void IIC_RD_Byte(uint8_t RegAddr, uint8_t* b);
void IIC_WR_Word(uint16_t word);
void IIC_WR_Byte(uint8_t byte);


#endif /* APP_IIC_IIC_H_ */
