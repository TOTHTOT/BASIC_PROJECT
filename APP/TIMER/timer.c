/*
 * timer.c
 *
 *  Created on: 2022年7月10日
 *      Author: TOTHTOT
 */

#include "timer.h"
#include "led.h"
#include "uart1.h"
#include "string.h"
#include "tb6612.h"
#include "graysensor.h"
#include "beep.h"
#include "oled.h"
#include "exit.h"
#define TIMER_PERIOD 615*4000

/* TB0初始化
 * 产生61,200,000/12/51000 = 25ms
 * */
void TimerB_Init(void)
{
    //Start timer
    Timer_B_clearTimerInterrupt(TIMER_B0_BASE);

    Timer_B_initUpModeParam param = { 0 };
    //
    param.clockSource = TIMER_B_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_6;
    param.timerPeriod = 51000;
    param.timerInterruptEnable_TBIE = TIMER_B_TBIE_INTERRUPT_DISABLE;
    param.captureCompareInterruptEnable_CCR0_CCIE =
    TIMER_B_CAPTURECOMPARE_INTERRUPT_ENABLE;
    param.timerClear = TIMER_B_DO_CLEAR;
    param.startTimer = true;
    Timer_B_initUpMode(TIMER_B0_BASE, &param);

    //Enter LPM0, enable interrupts
    __bis_SR_register(GIE);

    //For debugger
    __no_operation();

}

/* TA0初始化
 * 4路PWM控制电机驱动
 * 40Mhz/1/2000=20khz
 * */
void TimerA0_PWM_Init(void)
{
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P1,GPIO_PIN2+GPIO_PIN3+GPIO_PIN4+GPIO_PIN5);//初始化GPIO
    Timer_A_outputPWMParam param;
    param.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;//选择时钟源
    param.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_1;//时钟源不分频
    param.timerPeriod = 1200;//定时器装载值
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;//初始化捕获比较寄存器1，对应通道TA0.1       对应M1
    param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;//选择输出模式
    param.dutyCycle = 500;//
    Timer_A_outputPWM(TIMER_A0_BASE,&param);

    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2;//初始化捕获比较寄存器2，对应通道TA0.2       对应M2
    Timer_A_outputPWM(TIMER_A0_BASE,&param);
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;//初始化捕获比较寄存器2，对应通道TA0.3       对应M3
    Timer_A_outputPWM(TIMER_A0_BASE,&param);
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;//初始化捕获比较寄存器2，对应通道TA0.4       对应M4
    Timer_A_outputPWM(TIMER_A0_BASE,&param);

    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, 0);
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2, 0);
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, 0);
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, 0);
}

/* TA1PWM初始化
 * 舵机控制
 * 40M/40/20000=50hz
 * */
void TimerA1_PWM_Init(void)
{
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,GPIO_PIN0);
    Timer_A_outputPWMParam param1;
    param1.clockSource = TIMER_A_CLOCKSOURCE_SMCLK;//选择时钟源
    param1.clockSourceDivider = TIMER_A_CLOCKSOURCE_DIVIDER_40;//时钟源不分频
    param1.timerPeriod = 20000;//定时器装载值
    param1.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;//初始化捕获比较寄存器1，对应通道TA0.1
    param1.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;//选择输出模式
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

/* TA2初始化
 * 输入捕获
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

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMERB0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMERB0_VECTOR)))
#endif

void TIMERB0_ISR(void)
{
    short Motor_Straight_Control_Num = 0; //用来处理巡线
    static uint8_t stop_count = 0;                    // 小车停止计数值
    static uint8_t stop_count_target = 10;            //小车停止计数目标值,随情况而定
    static uint8_t i;
    float actual_speed = 0.0f;
    int output;
    static pid_i = 0;
    //每50ms的中断用来计算pid
    if (i == 40)
    {
        i = 0;
        Led_Toggle(LED1);
    }
    i++;
    pid_i++;
    if (beep_en == 1)
    {
        BEEP_LOW;
        beep_en_on++;
        if (beep_en_on > 40)
            beep_en = 0;
    }
    else
    {
        BEEP_High;
    }
////    Motor_Straight_Control_Num += Car_Staright_Control();
//    if (Car_1.pid_en == 1)
//    {
//        pid_i = 0;
//        Motor_Straight_Control_Num = Car_Staright_Control();
//        Car_1.motro_state[0].speed_output_value = Car_Speed_PID(
//                &Car_1.motro_state[0]);
//
//        Car_1.motro_state[0].speed_output_value_finally =
//                Car_1.motro_state[0].speed_output_value / 0.6
//                        - (Motor_Straight_Control_Num * 1.0);
//        Car_1.motro_state[1].speed_output_value = Car_Speed_PID(
//                &Car_1.motro_state[1]);
//
//        Car_1.motro_state[1].speed_output_value_finally =
//                Car_1.motro_state[1].speed_output_value / 0.6
//                        + (Motor_Straight_Control_Num * 1.0);
//        Motor_Output(Car_1);
//        printf("M1:%f, M2:%f\r\n",
//               ((Car_1.motro_state[0].encode_num / Car_MOTOR_PULSE_PER_CYCLE)
//                       * (60.0 * 1000.0 / Car_PID_CYCLE)) / 0.6,
//
//               ((Car_1.motro_state[1].encode_num / Car_MOTOR_PULSE_PER_CYCLE)
//                       * (60.0 * 1000.0 / Car_PID_CYCLE)) / 0.6);
//        Car_1.motro_state[0].encode_num = 0;
//        Car_1.motro_state[1].encode_num = 0;
//    }
//    不使用PID根据单位时间内脉冲时间计算速度
//    if(car_state == 1)
//    {
////        printf("%d, %d\r\n", Car_1.motro_state[0].encode_num, Car_1.motro_state[1].encode_num);
////        Car_1.motro_state[0].encode_num = 0;
////        Car_1.motro_state[1].encode_num = 0;
//    }
}


