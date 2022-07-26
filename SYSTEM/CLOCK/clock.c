/*
 * clock.c
 *
 *  Created on: 2022年7月10日
 *      Author: TOTHTOT
 */
#include "clock.h"
uint8_t returnValue = 0;
/* 时钟初始化
 * */
void Clock_Init(void)
{
    //Set VCore = 1 for 12MHz clock
        PMM_setVCore(PMM_CORE_LEVEL_1);//主频提高后，VCore电压也需要随之配置

        //Initializes the XT1 and XT2 crystal frequencies being used
        UCS_setExternalClockSource(UCS_XT1_CRYSTAL_FREQUENCY,UCS_XT2_CRYSTAL_FREQUENCY);//设置外部时钟源的频率，没什么实际设定

        //Initialize XT1. Returns STATUS_SUCCESS if initializes successfully
        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,GPIO_PIN4 + GPIO_PIN5);//XT1口不作为普通IO



        //Startup HF XT2 crystal Port select XT2
        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,GPIO_PIN2 + GPIO_PIN3);//XT2口不作为普通IO

        //Initialize XT2. Returns STATUS_SUCCESS if initializes successfully
        returnValue = UCS_turnOnXT2WithTimeout(UCS_XT2_DRIVE_4MHZ_8MHZ,UCS_XT2_TIMEOUT);//启动XT2

        //Set DCO FLL reference = REFO
        UCS_initClockSignal(UCS_FLLREF,UCS_XT2CLK_SELECT,UCS_CLOCK_DIVIDER_1);//XT2作为FLL参考

        //Set Ratio and Desired MCLK Frequency  and initialize DCO
        UCS_initFLLSettle(UCS_MCLK_DESIRED_FREQUENCY_IN_KHZ,UCS_MCLK_FLLREF_RATIO);//MCLK设置为24MHz

        //Set ACLK = REFO
        UCS_initClockSignal(UCS_ACLK,UCS_REFOCLK_SELECT,UCS_CLOCK_DIVIDER_1);//ACLK设置为32.768kHz

        UCS_initClockSignal(UCS_SMCLK,UCS_DCOCLK_SELECT,UCS_CLOCK_DIVIDER_1);//SMCLK设置为24MHz, UCS_DCOCLK_SELECT作为SMCLK时钟源

}


