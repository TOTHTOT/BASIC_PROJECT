//*****************************************************************************
#include "driverlib.h"
#include "config.h"
#include "clock.h"
#include "uart1.h"
#include "led.h"
#include "delay.h"
#include "timer.h"
#include "iic.h"
#include "oled.h"
#include "exit.h"
#include "bh1750.h"
#include "mpu6050.h"
#include "tb6612.h"
#include "pid.h"
#include "beep.h"
#include "graysensor.h"
//#include "ioi2c.h"

#if USE_MPU6050
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#endif
//*****************************************************************************

short OUT_PWM(uint8_t motor);

uint16_t status;
uint8_t clockValue;
short m1_out = 0, m2_out = 0;
void main(void)
{
    u8 *oled_str;
    short M1_OUTPWM, M2_OUTPWM;

    uint32_t loop_times = 0;
    uint8_t huidu_l_delay = 0, huidu_r_delay = 0;
    WDT_A_hold(WDT_A_BASE);         //关闭看门狗
    Clock_Init();                   //设置时钟
    Uart_Init(USCI_A1_BASE, 115200);                   //串口1初始化波特率115200
    Uart_Init(USCI_A0_BASE, 115200);                   //串口1初始化波特率115200
    Led_Init(ALL_LED);              //初始化2个LED
    TimerA0_PWM_Init();             //A0产生4路PWM
    TimerB_Init();                  //产生50ms中断
    EXIT_Init();                    //外部中断初始化
    Car_Init();
    Bep_Init();

#if USE_OLED
    OLED_Init();
    delay_ms(500);
    OLED_Clear();
    OLED_ShowString(0, 0, "MSP430F5529 OLED", 1);
    OLED_ShowString(0, 2, "   2021-08-05", 1);
#endif
    __bis_SR_register(GIE);
    Car_Struct_Init(&Car_1);
    PID_Init();
    Car_Direction(zhengzhuan, 1);
    Car_Direction(zhengzhuan, 2);

//    M1_OUTPWM = Speed_To_Pwm(200);
//    M2_OUTPWM = Speed_To_Pwm(200);
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1,
                            Speed_0_3_M1);                    //500
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2,
                            Speed_0_3_M2);                    //320

    printf("进入主循环\r\n");
    beep_en= 1;

    while (1)
    {
        loop_times++;

        //sprintf((char*)oled_str, "speed:%d", Car_1.motro_state[1].max_speed);
        //OLED_ShowString(0, 6, oled_str, 1);
//        M1_OUTPWM = Speed_To_Pwm(200) - Car_Staright_Control();
//        M2_OUTPWM = Speed_To_Pwm(200) + Car_Staright_Control();
        Timer_A_setCompareValue(TIMER_A0_BASE,
                                TIMER_A_CAPTURECOMPARE_REGISTER_1,
                                OUT_PWM(1));
        Timer_A_setCompareValue(TIMER_A0_BASE,
                                TIMER_A_CAPTURECOMPARE_REGISTER_2,
                                OUT_PWM(2));
        if(huidu_l_en == 0)
        {
            huidu_l_delay++;
            if(huidu_l_delay == 2)
            {
                huidu_l_en = 1;
                huidu_l_delay = 0;
            }
        }
        if (huidu_r_en == 0)
        {
            huidu_r_delay++;
            if (huidu_r_delay == 2)
            {
                huidu_r_en = 1;
                huidu_r_delay = 0;
            }
        }
        if(loop_times == 30000)
            loop_times = 0;
//        Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, Speed_0_3_M1 + (OPENMV_Data.output*7));
//        Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2, Speed_0_3_M2-(OPENMV_Data.output*7));
//        printf("m1:%d\r\n", Speed_0_3_M1 + (OPENMV_Data.output * 5));
//        printf("m2:%d\r\n", Speed_0_3_M2 - (OPENMV_Data.output * 5));

//        m1_out = m2_out = 300;
//        m1_out = Car_Staright_Control();
//        m2_out = Car_Staright_Control();
//        Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1,  m1_out);
//        Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2,  m2_out);
//        loop_times++;
//        printf("encode\r\n");
//        printf("encode%d\r\n", Car_1.motro_state[1].encode_num);

        delay_ms(5);
    }
}

short OUT_PWM(uint8_t motor)
{
    short result;
    switch(motor)
    {
    case 1:
        result = Speed_0_3_M1;
        if (huidu_l_en == 1)
        {
            result = result - Car_Staright_Control();

        }
        break;
    case 2:
        result = Speed_0_3_M2;
        if (huidu_r_en == 1)
        {
            result = result + Car_Staright_Control();
        }
        break;
    }
    return result;
}

