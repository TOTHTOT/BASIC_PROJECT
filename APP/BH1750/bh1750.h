/*
 * bh150.h
 *
 *  Created on: 2022年7月11日
 *      Author: TOTHTOT
 */

#ifndef APP_BH1750_BH1750_H_
#define APP_BH1750_BH1750_H_

#include "driverlib.h"

#define u8 unsigned char
#define u16 unsigned short

typedef unsigned char BYTE;
typedef unsigned short WORD;

#define BH1750_ADDERSS 0x46

extern BYTE BUF[8]; /* 接收数据缓存区 */

void BH1750_Init(void);
void Single_Write_BH1750(u8 REG_Address);
void Get_BH1750Data(u8 *BH1750Data_H , u8 *BH1750Data_L);


#endif /* APP_BH1750_BH1750_H_ */
