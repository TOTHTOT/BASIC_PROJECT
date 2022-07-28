/*
 * clock.c
 *
 *  Created on: 2022��7��10��
 *      Author: TOTHTOT
 */
#include "clock.h"
uint8_t returnValue = 0;
/* ʱ�ӳ�ʼ��
 * */
void Clock_Init(void)
{

    PMM_setVCore(PMM_CORE_LEVEL_3);     //����Ƶ������Ҫ�ϸߵĺ��ĵ�ѹ

        //XT1���Ÿ���
        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN4);
        GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN5);

        //����XT1
        UCS_turnOnLFXT1(UCS_XT1_DRIVE_3,UCS_XCAP_3);

        //XT2���Ÿ���
        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5, GPIO_PIN2);
        GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P5, GPIO_PIN3);

        //����XT2
        UCS_turnOnXT2(UCS_XT2_DRIVE_4MHZ_8MHZ);

        //XT2��ΪFLL�ο�ʱ�ӣ���8��Ƶ����50��Ƶ 4MHz / 8 * 50 = 25MHz
        UCS_initClockSignal(UCS_FLLREF, UCS_XT2CLK_SELECT, UCS_CLOCK_DIVIDER_8);
        UCS_initFLLSettle(25000, 50);

        //XT1��ΪACLKʱ��Դ = 32768Hz
        UCS_initClockSignal(UCS_ACLK, UCS_XT1CLK_SELECT, UCS_CLOCK_DIVIDER_1);

        //DCOCLK��ΪMCLKʱ��Դ = 25MHz
        UCS_initClockSignal(UCS_MCLK, UCS_DCOCLK_SELECT, UCS_CLOCK_DIVIDER_1);

        //DCOCLK��ΪSMCLKʱ��Դ = 25MHz
        UCS_initClockSignal(UCS_SMCLK, UCS_DCOCLK_SELECT, UCS_CLOCK_DIVIDER_1);

        //�����ⲿʱ��Դ��Ƶ�ʣ�ʹ���ڵ���UCS_getMCLK, UCS_getSMCLK �� UCS_getACLKʱ�ɵõ���ȷֵ
        UCS_setExternalClockSource(32768, 4000000);

//    //Set VCore = 1 for 12MHz clock
//        PMM_setVCore(PMM_CORE_LEVEL_1);//��Ƶ��ߺ�VCore��ѹҲ��Ҫ��֮����
//
//        //Initializes the XT1 and XT2 crystal frequencies being used
//        UCS_setExternalClockSource(UCS_XT1_CRYSTAL_FREQUENCY,UCS_XT2_CRYSTAL_FREQUENCY);//�����ⲿʱ��Դ��Ƶ�ʣ�ûʲôʵ���趨
//
//        //Initialize XT1. Returns STATUS_SUCCESS if initializes successfully
//        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,GPIO_PIN4 + GPIO_PIN5);//XT1�ڲ���Ϊ��ͨIO
//
//
//
//        //Startup HF XT2 crystal Port select XT2
//        GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,GPIO_PIN2 + GPIO_PIN3);//XT2�ڲ���Ϊ��ͨIO
//
//        //Initialize XT2. Returns STATUS_SUCCESS if initializes successfully
//        returnValue = UCS_turnOnXT2WithTimeout(UCS_XT2_DRIVE_4MHZ_8MHZ,UCS_XT2_TIMEOUT);//����XT2
//
//        //Set DCO FLL reference = REFO
//        UCS_initClockSignal(UCS_FLLREF,UCS_XT2CLK_SELECT,UCS_CLOCK_DIVIDER_1);//XT2��ΪFLL�ο�
//
//        //Set Ratio and Desired MCLK Frequency  and initialize DCO
//        UCS_initFLLSettle(UCS_MCLK_DESIRED_FREQUENCY_IN_KHZ,UCS_MCLK_FLLREF_RATIO);//MCLK����Ϊ24MHz
//
//        //Set ACLK = REFO
//        UCS_initClockSignal(UCS_ACLK,UCS_REFOCLK_SELECT,UCS_CLOCK_DIVIDER_1);//ACLK����Ϊ32.768kHz
//
//        UCS_initClockSignal(UCS_SMCLK,UCS_DCOCLK_SELECT,UCS_CLOCK_DIVIDER_1);//SMCLK����Ϊ24MHz, UCS_DCOCLK_SELECT��ΪSMCLKʱ��Դ
//        UCS_setExternalClockSource(32768, 4000000);
}


