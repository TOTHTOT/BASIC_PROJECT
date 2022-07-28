/*
 * timer.c
 *
 *  Created on: 2022��7��10��
 *      Author: TOTHTOT
 */

#include "timer.h"
#include "led.h"
#include "uart1.h"
#include "tb6612.h"
#include "graysensor.h"

#define TIMER_PERIOD 615*4000

/* TB0��ʼ��
 * ����61,200,000/24/51000 = 50ms
 * */
void TimerB_Init(void)
{
    //Start timer
    Timer_B_clearTimerInterrupt(TIMER_B0_BASE);

    Timer_B_initUpModeParam param = { 0 };
    //
    param.clockSource = TIMER_B_CLOCKSOURCE_SMCLK;
    param.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_24;
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
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_1;//��ʼ������ȽϼĴ���1����Ӧͨ��TA0.1       ��ӦM1
    param.compareOutputMode = TIMER_A_OUTPUTMODE_RESET_SET;//ѡ�����ģʽ
    param.dutyCycle = 500;//
    Timer_A_outputPWM(TIMER_A0_BASE,&param);

    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_2;//��ʼ������ȽϼĴ���2����Ӧͨ��TA0.2       ��ӦM2
    Timer_A_outputPWM(TIMER_A0_BASE,&param);
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_3;//��ʼ������ȽϼĴ���2����Ӧͨ��TA0.3       ��ӦM3
    Timer_A_outputPWM(TIMER_A0_BASE,&param);
    param.compareRegister = TIMER_A_CAPTURECOMPARE_REGISTER_4;//��ʼ������ȽϼĴ���2����Ӧͨ��TA0.4       ��ӦM4
    Timer_A_outputPWM(TIMER_A0_BASE,&param);

    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, 0);
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2, 0);
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, 0);
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, 0);
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

//TIMERB0_VECTOR
void Timer_A_Init(void)
{

}



#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMERB0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMERB0_VECTOR)))
#endif
void TIMERB0_ISR (void)
{
    short Motor_Straight_Control_Num = 0; //��������Ѳ��
    static uint8_t stop_count = 0;                    // С��ֹͣ����ֵ
    static uint8_t stop_count_target = 10;            //С��ֹͣ����Ŀ��ֵ,���������
    static uint8_t i;
    float actual_speed = 0.0f;
    int output;
    //ÿ50ms���ж���������pid
    if(i == 10)
    {
        i= 0;
        Led_Toggle(LED1);
    }
    i++;
    if (Car_1.pid_en == 1)
    {
        Motor_Straight_Control_Num = Car_Staright_Control();
        //speed only
        Car_1.motro_state[0].speed_output_value = Car_Speed_PID(
                &Car_1.motro_state[0]);

        Car_1.motro_state[0].speed_output_value_finally =
                Car_1.motro_state[0].speed_output_value / 0.6 - (Motor_Straight_Control_Num*1.0);

        Car_1.motro_state[1].speed_output_value = Car_Speed_PID(
                &Car_1.motro_state[1]);

        Car_1.motro_state[1].speed_output_value_finally =
                Car_1.motro_state[1].speed_output_value / 0.6 + (Motor_Straight_Control_Num*1.0);

//        Car_1.motro_state[2].speed_output_value = Car_Speed_PID(
//                &Car_1.motro_state[2]);

//        Car_1.motro_state[2].speed_output_value_finally =
//                Car_1.motro_state[2].speed_output_value / 0.6;
//
//        Car_1.motro_state[3].speed_output_value = Car_Speed_PID(
//                &Car_1.motro_state[3]);
//
//        Car_1.motro_state[3].speed_output_value_finally =
//                Car_1.motro_state[3].speed_output_value / 0.6;

        Motor_Output(Car_1);
        printf("%f, %f\r\n",
               ((Car_1.motro_state[0].encode_num/Car_MOTOR_PULSE_PER_CYCLE) * (60.0 * 1000.0 / 50))/0.6,

               ((Car_1.motro_state[1].encode_num/Car_MOTOR_PULSE_PER_CYCLE) * (60.0 * 1000.0 / 50))/0.6);
        Car_1.motro_state[0].encode_num = 0;
        Car_1.motro_state[1].encode_num = 0;
    }

//    actual_speed = ((float)Car_1.motro_state[1].encode_num /Car_MOTOR_PULSE_PER_CYCLE) * (60.0 * 1000.0 / 50); // rpm
//    Car_1.motro_state[1].encode_num = 0;
//    printf("actual_speed:%f\r\n", actual_speed);
//    if (Car_1.pid_en == 1)
//    {
//        Car_1.motro_state[0].distance = ((Car_1.motro_state[0].total_encode_num
//                / Car_MOTOR_PULSE_PER_CYCLE) * Car_PI * Car_CheLunZhiJing); //С����Car_PID_CYCLEʱ����ת����������/һȦ���������ٳ���ֱ������·��
//        Car_1.motro_state[1].distance = ((Car_1.motro_state[1].total_encode_num
//                / Car_MOTOR_PULSE_PER_CYCLE) * Car_PI * Car_CheLunZhiJing); //С����Car_PID_CYCLEʱ����ת����������/һȦ���������ٳ���ֱ������·��
//        Car_1.motro_state[2].distance = ((Car_1.motro_state[2].total_encode_num
//                / Car_MOTOR_PULSE_PER_CYCLE) * Car_PI * Car_CheLunZhiJing); //С����Car_PID_CYCLEʱ����ת����������/һȦ���������ٳ���ֱ������·��
//        Car_1.motro_state[3].distance = ((Car_1.motro_state[3].total_encode_num
//                / Car_MOTOR_PULSE_PER_CYCLE) * Car_PI * Car_CheLunZhiJing); //С����Car_PID_CYCLEʱ����ת����������/һȦ���������ٳ���ֱ������·��
////        printf("1:%f, 2:%f, 3:%f, 4:%f\r\n", (float)Car_1.motro_state[0].total_encode_num, (float)Car_1.motro_state[1].total_encode_num, (float)Car_1.motro_state[2].total_encode_num, (float)Car_1.motro_state[3].total_encode_num);
//
////        printf("���߾���:%f, %f, %f, %f\r\n", Car_1.motro_state[0].distance,
////                  Car_1.motro_state[1].distance, Car_1.motro_state[2].distance,
////                  Car_1.motro_state[3].distance);
//        if (Car_1.direction == forward)
//        {
//            Location_Speed_Control(&Car_1);
//            Car_1.motro_state[0].speed_output_value_finally =
//                    Car_1.motro_state[0].speed_output_value
//                            + Motor_Straight_Control_Num;
//            Car_1.motro_state[1].speed_output_value_finally =
//                    Car_1.motro_state[1].speed_output_value
//                            - Motor_Straight_Control_Num;
//            Car_1.motro_state[2].speed_output_value_finally =
//                    Car_1.motro_state[2].speed_output_value
//                            + Motor_Straight_Control_Num;
//            Car_1.motro_state[3].speed_output_value_finally =
//                    Car_1.motro_state[3].speed_output_value
//                            - Motor_Straight_Control_Num;
////            printf("outvalue:%\r\n", Car_1.motro_state[0].speed_output_value_finally);
//            Motor_Output(Car_1);
//            // �ڷǳ��ӽ�Ŀ��λ��ʱֹͣ����,����˷�ʱ���������׵����,ֻ���һ��������������
//            if ((Car_1.motro_state[0].distance
//                    <= Car_1.motro_state[0].target_distance + 10)
//                    && (Car_1.motro_state[0].distance
//                            >= Car_1.motro_state[0].target_distance - 10))
//            {
//                stop_count++;
//                if (stop_count >= stop_count_target)
//                {
//                    stop_count = 0;
//                    Car_1.direction = stop; //С���л���ֹͣ״̬
////                    printf("���߾���:%f, %f, %f, %f\r\n",
////                              Car_1.motro_state[0].distance,
////                              Car_1.motro_state[1].distance,
////                              Car_1.motro_state[2].distance,
////                              Car_1.motro_state[3].distance);
//                }
//            }
//        }
//        if (Car_1.direction == stop)
//        {
////            u1_printf("ֹͣ\r\n");
//            Car_Stop(&Car_1);
//            stop_count = 0;
//        }

//    }
}


