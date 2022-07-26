/*
 * iic.c
 *
 *  Created on: 2022年7月11日
 *      Author: TOTHTOT
 */

#include "iic.h"
#include "uart1.h"

#define I2C_BUF_LENGTH 32
static char i2c_buf[I2C_BUF_LENGTH];
static uint8_t i2c_buf_len = 0;
static uint8_t i2c_buf_cur = 0;

static uint8_t *i2c_rx_buf = 0;
static uint8_t i2c_rx_buf_len = 0;

/* IIC_Init
 * 描述:B0_IIC初始化
 * 参数:slave_adderss从机地址
 * */
void IIC_Init(unsigned char slave_adderss)
{
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P3,
    GPIO_PIN0 | GPIO_PIN1);
    /* I2C Master Configuration Parameter */
    USCI_B_I2C_initMasterParam i2cConfig = {
    USCI_B_I2C_CLOCKSOURCE_SMCLK,
                                             UCS_getSMCLK(),
                                             USCI_B_I2C_SET_DATA_RATE_400KBPS };
    /* Initialize USCI_B0 and I2C Master to communicate with slave devices*/
    USCI_B_I2C_initMaster(USCI_B0_BASE, &i2cConfig);

    /* Enable I2C Module to start operations */
    USCI_B_I2C_enable(USCI_B0_BASE);

    // Specify slave address
    USCI_B_I2C_setSlaveAddress(USCI_B0_BASE, slave_adderss);
    USCI_B_I2C_clearInterrupt(USCI_B0_BASE, USCI_B_I2C_RECEIVE_INTERRUPT);
    USCI_B_I2C_enableInterrupt(USCI_B0_BASE, USCI_B_I2C_RECEIVE_INTERRUPT);
    USCI_B_I2C_clearInterrupt(USCI_B0_BASE, USCI_B_I2C_TRANSMIT_INTERRUPT);
    USCI_B_I2C_enableInterrupt(USCI_B0_BASE, USCI_B_I2C_TRANSMIT_INTERRUPT);
}

/* IIC_WR_Byte
 * 描述:IIC写入一个字节
 * 参数:byte:要发送的字节
 * */
void IIC_WR_Byte(uint8_t byte)
{
//    static uint8_t retry;
    while (USCI_B_I2C_isBusBusy(USCI_B0_BASE));
//    {
//        if (retry == 0xff)
//        {
//            printf("USCI_B_I2C_isBusBusy_WR_B\r\n");
//            retry = 0;
//            break;
//        }
//        retry++;
//    }
    USCI_B_I2C_setMode(USCI_B0_BASE, USCI_B_I2C_TRANSMIT_MODE);

    // Initiate start and send first character
    i2c_buf[0] = byte;
    i2c_buf_cur = 1;
    i2c_buf_len = 1;
    USCI_B_I2C_masterSendMultiByteStart(USCI_B0_BASE, i2c_buf[0]);
}

/* IIC_WR_Word
 * 描述:IIC写入一个字
 * 参数:byte:要发送的字
 * */
void IIC_WR_Word(uint16_t word)
{
//    static uint8_t retry;
    while (USCI_B_I2C_isBusBusy(USCI_B0_BASE));
//    {
//        if (retry == 0xff)
//        {
//            printf("USCI_B_I2C_isBusBusy_WR_WD\r\n");
//            retry = 0;
//            break;
//        }
//        retry++;
//    }

    USCI_B_I2C_setMode(USCI_B0_BASE, USCI_B_I2C_TRANSMIT_MODE);

    // Initiate start and send first character
    i2c_buf[0] = word >> 8;
    i2c_buf[1] = (uint8_t) word;
    i2c_buf_cur = 1;
    i2c_buf_len = 2;
    USCI_B_I2C_masterSendMultiByteStart(USCI_B0_BASE, i2c_buf[0]);

    // wait for end

}

/* IIC_RD_Byte
 * 描述:IIC读取一个字节
 * 参数:RegAddr:寄存器地址;b:存放字节的指针
 * */
void IIC_RD_Byte(uint8_t RegAddr, uint8_t* b)
{
//    static uint8_t retry;
    while (USCI_B_I2C_isBusBusy(USCI_B0_BASE));
//    {
//        if (retry == 0xff)
//        {
//            printf("USCI_B_I2C_isBusBusy_RD_B\r\n");
//            break;
//        }
//        retry++;
//    }
    // send address
    USCI_B_I2C_setMode(USCI_B0_BASE, USCI_B_I2C_TRANSMIT_MODE);
    i2c_buf_cur = 1;
    i2c_buf_len = 1;
    USCI_B_I2C_masterSendSingleByte(USCI_B0_BASE, RegAddr);

    // receive
    USCI_B_I2C_setMode(USCI_B0_BASE, USCI_B_I2C_RECEIVE_MODE);
    i2c_rx_buf = b;
    i2c_rx_buf_len = 1;
    USCI_B_I2C_masterReceiveSingleStart(USCI_B0_BASE);
}

/* IIC_RD_Bytes
 * 描述:IIC读取多个字节
 * 参数:RegAddr:寄存器地址;length:长度;b:存放字节的指针
 * */
void IIC_RD_Bytes(uint8_t RegAddr, uint8_t length, uint8_t* data)
{
//    static uint8_t retry;
    while (USCI_B_I2C_isBusBusy(USCI_B0_BASE));
//    {
//        if (retry == 0xff)
//        {
//            printf("USCI_B_I2C_isBusBusy_RD_BS\r\n");
//            retry = 0;
//            break;
//        }
//        retry++;
//    }
    // send address
    USCI_B_I2C_setMode(USCI_B0_BASE, USCI_B_I2C_TRANSMIT_MODE);
    i2c_buf_cur = 1;
    i2c_buf_len = 1;
    USCI_B_I2C_masterSendSingleByte(USCI_B0_BASE, RegAddr);

    // receive
    USCI_B_I2C_setMode(USCI_B0_BASE, USCI_B_I2C_RECEIVE_MODE);
    i2c_rx_buf = data;
    i2c_rx_buf_len = length;
    USCI_B_I2C_masterReceiveMultiByteStart(USCI_B0_BASE);
}

#pragma vector = USCI_B0_VECTOR
__interrupt void USCI_B0_ISR(void)
{
    switch (__even_in_range(UCB0IV, 12))
    {
    case USCI_I2C_UCTXIFG:
        if (i2c_buf_cur < i2c_buf_len)
        {
            USCI_B_I2C_masterSendMultiByteNext( USCI_B0_BASE,
                                               i2c_buf[i2c_buf_cur]);
            i2c_buf_cur++;
        }
        else
        {
            USCI_B_I2C_masterSendMultiByteStop(USCI_B0_BASE);
            //Clear master interrupt status
            USCI_B_I2C_clearInterrupt(USCI_B0_BASE,
            USCI_B_I2C_TRANSMIT_INTERRUPT);
        }
        break;
    case USCI_I2C_UCRXIFG:
        i2c_rx_buf_len--;
        if (i2c_rx_buf_len)
        {
            if (i2c_rx_buf_len == 1)
            {
                //Initiate end of reception -> Receive byte with NAK
                *i2c_rx_buf++ = USCI_B_I2C_masterReceiveMultiByteFinish(
                USCI_B0_BASE);
                printf("res1:%x\r\n", i2c_rx_buf);
            }
            else
            {
                //Keep receiving one byte at a time
                *i2c_rx_buf++ = USCI_B_I2C_masterReceiveMultiByteNext(
                USCI_B0_BASE);
                printf("res2:%x\r\n", i2c_rx_buf);
            }
        }
        else
        {
            //Receive last byte
            *i2c_rx_buf = USCI_B_I2C_masterReceiveMultiByteNext(USCI_B0_BASE);
            printf("res3:%x\r\n", i2c_rx_buf);
        }
        break;
    }
}

