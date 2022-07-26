/*
 * timer.c
 *
 *  Created on: 2022��7��10��
 *      Author: TOTHTOT
 */

#include "timer.h"

/* TB0��ʼ��
 * ����40M/10/4000 = 1ms
 * */
void TimerB_Init(void)
{
    Timer_B_initUpModeParam paramB = {0};
    paramB.clockSource = TIMER_B_CLOCKSOURCE_SMCLK;
    paramB.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_10;
    paramB.timerPeriod = 4000;
    paramB.timerInterruptEnable_TBIE = TIMER_B_TBIE_INTERRUPT_DISABLE;
    paramB.captureCompareInterruptEnable_CCR0_CCIE = TIMER_B_CCIE_CCR0_INTERRUPT_ENABLE;
    paramB.timerClear = TIMER_B_DO_CLEAR;
    paramB.startTimer = true;
    Timer_B_initUpMode(TIMER_B0_BASE,&paramB);
    Timer_B_startCounter( TIMER_B0_BASE,TIMER_B_UP_MODE);
}

/* TA0��ʼ��
 * 4·PWM���Ƶ������
 * 40Mhz/1/2000=20khz
 * */
void TimerA0_PWM_Init(void)
{
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1,GPIO_PIN2+GPIO_PIN3+GPIO_PIN4+GPIO_PIN5);//��ʼ��GPIO
    Timer_A_outputPWMParam param;
    param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;//ѡ��ʱ��Դ
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;//ʱ��Դ����Ƶ
    param.timerPeriod = 1200;//��ʱ��װ��ֵ
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;//��ʼ������ȽϼĴ���1����Ӧͨ��TA0.1
    param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;//ѡ�����ģʽ
    param.dutyCycle = 500;//
    Timer_A_outputPWM(TIMER_A0_BASE,&param);

    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2;//��ʼ������ȽϼĴ���2����Ӧͨ��TA0.2
    Timer_A_outputPWM(TIMER_A0_BASE,&param);
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;//��ʼ������ȽϼĴ���2����Ӧͨ��TA0.3
    Timer_A_outputPWM(TIMER_A0_BASE,&param);
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;//��ʼ������ȽϼĴ���2����Ӧͨ��TA0.4
    Timer_A_outputPWM(TIMER_A0_BASE,&param);
}

/* TA1PWM��ʼ��
 * �������
 * 40M/40/20000=50hz
 * */
void TimerA1_PWM_Init(void)
{
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN0);
    Timer_A_outputPWMParam param1;
    param1.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;//ѡ��ʱ��Դ
    param1.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_40;//ʱ��Դ����Ƶ
    param1.timerPeriod = 20000;//��ʱ��װ��ֵ
    param1.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;//��ʼ������ȽϼĴ���1����Ӧͨ��TA0.1
    param1.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;//ѡ�����ģʽ
    param1.dutyCycle = 0;//
    Timer_A_outputPWM(TIMER_A1_BASE,&param1);

//    Timer_A_initUpModeParam param = {0};
//    param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
//    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_10;
//    param.timerPeriod = 4000;
//    param.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_DISABLE;
//    param.captureCompareInterruptEnable_CCR0_CCIE = TIMER_A_CCIE_CCR0_INTERRUPT_ENABLE;
//    param.timerClear = TIMER_A_DO_CLEAR;
//    param.startTimer = true;
//    Timer_A_initUpMode(TIMER_A1_BASE,&param);
//    Timer_A_startCounter( TIMER_A1_BASE,TIMER_A_UP_MODE);

}

/* TA2��ʼ��
 * ���벶��
 * 40/1
 * */
void TimerA2_Capture_Init(void)
{
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2, GPIO_PIN4 + GPIO_PIN5);

    Timer_A_initContinuousModeParam param0 = {0};
    param0.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;
    param0.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;
    param0.timerInterruptEnable_TAIE = TIMER_A_TAIE_INTERRUPT_ENABLE;
    param0.timerClear = TIMER_A_DO_CLEAR;
    param0.startTimer = true;
    Timer_A_initContinuousMode(TIMER_A2_BASE,&param0);

    Timer_A_initCaptureModeParam param1 = {0};
    param1.captureRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;
    param1.captureMode = TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE;
    param1.captureInputSelect = TIMER_A_CAPTURE_INPUTSELECT_CCIxA;
    param1.synchronizeCaptureSource = TIMER_A_CAPTURE_SYNCHRONOUS;
    param1.captureInterruptEnable = TIMER_A_CAPTURECOMPARE_INTERRUPT_ENABLE;
    param1.captureOutputMode = TIMER_A_OUTPUTMODE_OUTBITVALUE;
    Timer_A_initCaptureMode(TIMER_A2_BASE,&param1);

    param1.captureRegister =  TIMER_A_CAPTURECOMPARE_REGISTER_2;//Two capture intputs PWM cannot start or end in the same time.
    param1.captureMode = TIMER_A_CAPTUREMODE_RISING_AND_FALLING_EDGE;
    Timer_A_initCaptureMode(TIMER_A2_BASE,&param1);
}

