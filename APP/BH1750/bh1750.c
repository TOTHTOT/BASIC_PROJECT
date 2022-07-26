/*
 * bh1750.c
 * ���մ�����ʹ�õ���Ӳ��IIC
 *  Created on: 2022��7��11��
 *      Author: TOTHTOT
 */
#include "bh1750.h"
#include "iic.h"
#include "uart1.h"

BYTE BUF[8]; /* �������ݻ����� */

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
 * ����:��ȡ����ǿ��
 * ����:BH1750Data_H:���ݸ�λ;BH1750Data_L:���ݵ�λ;
 * */
void Get_BH1750Data(u8 *BH1750Data_H , u8 *BH1750Data_L)
{
    u16 dis_data = 0;
    float temp = 0.0f;         /* Ҫ��ʾ����ֵ */
    Single_Write_BH1750(0x01); /* power on */
    Single_Write_BH1750(0x10); /* H-resolution mode */
    // delay_ms ( 180 ); /* ��ʱ180ms */
    Multiple_read_BH1750(); /* �����������ݣ��洢��BUF�� */
    dis_data = BUF[0];
    dis_data = (dis_data << 8) + BUF[1]; /* �ϳ����ݣ����������� */
    temp = (float)dis_data / 1.2;

    *BH1750Data_H = (int)temp >> 8;
    *BH1750Data_L = (int)temp & 0x00ff;
    printf("����:%f\r\n", temp);
}
