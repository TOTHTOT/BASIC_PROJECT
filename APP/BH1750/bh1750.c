/*
 * bh1750.c
 * 光照传感器使用的是硬件IIC
 *  Created on: 2022年7月11日
 *      Author: TOTHTOT
 */
#include "bh1750.h"
#include "iic.h"
#include "uart1.h"

BYTE BUF[8]; /* 接收数据缓存区 */

void BH1750_Init(void)
{
    IIC_Init(BH1750_ADDERSS>>1);
    Single_Write_BH1750(0x01);
}

void Single_Write_BH1750(u8 REG_Address)
{
    USCI_B_I2C_setSlaveAddress(USCI_B0_BASE, BH1750_ADDERSS>>1);
    IIC_WR_Word((BH1750_ADDERSS << 8) | REG_Address);
}

void Multiple_read_BH1750(void)
{
    USCI_B_I2C_setSlaveAddress(USCI_B0_BASE, BH1750_ADDERSS >> 1);
    IIC_RD_Bytes(BH1750_ADDERSS >> 1 + 1, 3, &BUF);
}

/* Get_BH1750Data
 * 描述:获取光照强度
 * 参数:BH1750Data_H:数据高位;BH1750Data_L:数据低位;
 * */
void Get_BH1750Data(u8 *BH1750Data_H , u8 *BH1750Data_L)
{
    u16 dis_data = 0;
    float temp = 0.0f;         /* 要显示的数值 */
    Single_Write_BH1750(0x01); /* power on */
    Single_Write_BH1750(0x10); /* H-resolution mode */
    // delay_ms ( 180 ); /* 延时180ms */
    Multiple_read_BH1750(); /* 连续读出数据，存储在BUF中 */
    dis_data = BUF[0];
    dis_data = (dis_data << 8) + BUF[1]; /* 合成数据，即光照数据 */
    temp = (float)dis_data / 1.2;

    *BH1750Data_H = (int)temp >> 8;
    *BH1750Data_L = (int)temp & 0x00ff;
    printf("光照:%f\r\n", temp);
}
