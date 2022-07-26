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
//#include "ioi2c.h"


#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
//*****************************************************************************


uint16_t status;
uint8_t clockValue;
void main (void)
{
    uint32_t loop_times = 0;
    char err;
    WDT_A_hold(WDT_A_BASE);         //关闭看门狗
    Clock_Init();                   //设置时钟
    Uart_Init(USCI_A1_BASE, 115200);//串口1初始化波特率115200
    Led_Init(ALL_LED);              //初始化2个LED
    TimerA0_PWM_Init();             //A0产生4路PWM
    __bis_SR_register(GIE);         //开启全局中断

    EXIT_Init();
    Car_Init();
    Car_Direction(forward);
#if USE_BH1750
    BH1750_Init();
#endif
    //Verify if the Clock settings are as expected
/*    clockValue = UCS_getMCLK();
    clockValue = UCS_getACLK();
    clockValue = UCS_getSMCLK();*/

#if USE_OLED
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "MSP430F5529 OLED", 1);
    OLED_ShowString(0, 2, "   2021-08-05", 1);
#endif
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_4, 500);
    printf("进入主循环\r\n");
    while (1)
    {
        Led_Toggle(LED1);
        loop_times++;
        printf("loop_times%d\r\n", loop_times);
        delay_ms(500);

    }
}
