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
//#include "ioi2c.h"

#if USE_MPU6050
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#endif
//*****************************************************************************

uint16_t status;
uint8_t clockValue;
void main (void)
{
    u8 *oled_str;

    uint32_t loop_times = 0;
    WDT_A_hold(WDT_A_BASE);         //关闭看门狗
    Clock_Init();                   //设置时钟
    Uart_Init(USCI_A1_BASE, 115200);//串口1初始化波特率115200
    Led_Init(ALL_LED);              //初始化2个LED
    TimerA0_PWM_Init();             //A0产生4路PWM
    TimerB_Init();                  //产生50ms中断
    EXIT_Init();                    //外部中断初始化
    Car_Init();


#if USE_OLED
    OLED_Init();
    delay_ms(500);
    OLED_Clear();
    OLED_ShowString(0, 0, "MSP430F5529 OLED", 1);
    OLED_ShowString(0, 2, "   2021-08-05", 1);
#endif
    Car_Struct_Init(&Car_1);
    PID_Init();
    Car_Direction(zhengzhuan, 1);
    Car_Direction(zhengzhuan, 2);
    Car_Direction(zhengzhuan, 3);
    Car_Direction(zhengzhuan, 4);
//    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, 500);
//    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2, 400);
//    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_3, 500);
//    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, 500);

    printf("进入主循环\r\n");
//    Car_Go(60, &Car_1, 50);
    Car_Go_Speed(&Car_1, 220);
    while (1)
    {

        sprintf(oled_str, "encode:%d", Car_1.motro_state[0].encode_num);
        OLED_ShowString(0, 6, oled_str, 1);
        loop_times++;
        printf("encode%d\r\n", Car_1.motro_state[0].encode_num);
        printf("encode%d\r\n", Car_1.motro_state[1].encode_num);

        delay_ms(500);
    }
}
