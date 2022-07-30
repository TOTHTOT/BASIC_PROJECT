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
uint8_t CAR = 1;
short OUT_PWM(uint8_t motor);
void oled_change_data(void);

uint16_t status;
uint8_t clockValue;
short m1_out = 0, m2_out = 0;
float M1_OUTPWM, M2_OUTPWM;
float Target_value = 0, Actual_value = 0.0;
float Target_value2 =0, Actual_value2 = 0.0;
uint8_t str[50];
uint8_t pid_en = 0;
uint16_t run_time;
uint16_t run_time_t;
uint8_t oled_str_time[20];
uint8_t oled_str_distance[20];
uint8_t oled_str_target_speed[20];
void main(void)
{

//200约等于0.5m/s
//    float Kp = 2.1, Ki= 0.06;
//    float Target_value = 170, Actual_value = 0.0;
//    float Target_value2 =170, Actual_value2 = 0.0;
    float err, integral;
    float err2, integral2;




    uint32_t loop_times = 0;
//    uint8_t huidu_l_delay = 0, huidu_r_delay = 0;
    WDT_A_hold(WDT_A_BASE);         //关闭看门狗
    Clock_Init();                   //设置时钟
    Uart_Init(USCI_A1_BASE, 115200);                   //串口1初始化波特率115200    和OPENMV通信
    Uart_Init(USCI_A0_BASE, 115200);                   //串口0初始化波特率115200    和蓝牙通信
    Led_Init(ALL_LED);              //初始化2个LED
    TimerA0_PWM_Init();             //A0产生4路PWM
    TimerB_Init();                  //产生50ms中断
    EXIT_Init();                    //外部中断初始化
    Bep_Init();
    Car_Init();
    __bis_SR_register(GIE);
    Car_Struct_Init(&Car_1);
    PID_Init();
    Car_Direction(fanzhuan, 1);
    Car_Direction(fanzhuan, 2);
    OLED_Init();
    OLED_Clear();
    OLED_ShowString(0, 0, "time:", 1);
    OLED_ShowString(0, 2, "speed:", 1);
    if(CAR == 1)
    {
        OLED_ShowString(80, 0, "Car1", 1);
    }
    else if(CAR == 2)
    {
        OLED_ShowString(80, 0, "Car2", 1);
        OLED_ShowString(0, 4, "juli", 1);
    }

//    M1_OUTPWM = Speed_To_Pwm(200);
//    M2_OUTPWM = Speed_To_Pwm(200);
//    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1,
//                            500);                    //500
//    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2,
//                            500);                    //320


    // u0_printf("进入主循环\r\n");
    beep_en= 1;
    int pianyi;
    int distance_err;
    char K;
    if(CAR==1)
    {
        K=0;
    }
    if(CAR==2)
    {
        K=1;
    }
    while (1)
    {
        loop_times++;
//        计算PID
        if(loop_times == 10)
        {
//            printf("pid_en%d",pid_en);
            if(pid_en==1)
            {
                //sprintf(str, "%d", OPENMV_Data.distance);
                //u0_printf(str);
                pianyi = OPENMV_Data.output;
                //            printf("pianyi:%d\r\n",pianyi);
//                if(pianyi>100)
//                {
//                    pianyi=100;
//                }
//                if(pianyi<-100)
//                {
//                    pianyi=-100;
//                }
                if (OPENMV_Data.distance > 400)
                {
                    OPENMV_Data.distance=400;
                }
                distance_err=0.1*(OPENMV_Data.distance-200)*K;

                Actual_value = -((float) M1_encode_num
                        / Car_MOTOR_PULSE_PER_CYCLE)
                        * (60.0 * 1000.0 / Car_PID_CYCLE); // rpm
                Actual_value2 = -((float) M2_encode_num
                        / Car_MOTOR_PULSE_PER_CYCLE)
                        * (60.0 * 1000.0 / Car_PID_CYCLE); // rpm
                //            if(pianyi>)
                M1_OUTPWM = speed_pid_realize(&speed_pid_m1, Actual_value)
                        - pianyi;
                M2_OUTPWM = speed_pid_realize(&speed_pid_m2, Actual_value2)
                        + pianyi;
                if (M1_OUTPWM > 600)
                    M1_OUTPWM = 600;
                if(M1_OUTPWM<0)
                {
                    M1_OUTPWM=abs(M1_OUTPWM);
                }
                Timer_A_setCompareValue(TIMER_A0_BASE,
                                        TIMER_A_CAPTURECOMPARE_REGISTER_1,
                                        (int) M1_OUTPWM);

                if (M2_OUTPWM > 600)
                    M2_OUTPWM = 600;
                if(M2_OUTPWM<0)
                {
                    M2_OUTPWM=abs(M2_OUTPWM);
                }
                Timer_A_setCompareValue(TIMER_A0_BASE,
                                        TIMER_A_CAPTURECOMPARE_REGISTER_2,
                                        (int) M2_OUTPWM);
                printf("%f, %f, %f ,%f,%d\r\n",Actual_value, Actual_value2, M1_OUTPWM, M2_OUTPWM,pianyi);//
                pianyi = 0;
                //            printf("%f, %f, %f ,%f\r\n",Actual_value, Actual_value2, M1_OUTPWM, M2_OUTPWM);//

                //            printf("%f, %f, %f ,%f\r\n",speed_pid_m1.target_val, Actual_value, M1_OUTPWM, speed_pid_m1.err);//320
                //            printf("%f, %f, %f ,%f\r\n",speed_pid_m2.target_val, Actual_value2, M2_OUTPWM, speed_pid_m2.err);//320

                M1_encode_num = 0;
                M2_encode_num = 0;
                run_time++;
                if(run_time == 20)
                {
                    run_time = 0;
                    run_time_t++;
                    oled_change_data_en = 1;

                    printf("%d\r\n", run_time_t);
                }

            }
            loop_times = 0;
        }
      oled_change_data();
      delay_ms(5);
    }
}

void oled_change_data(void)
{
    if(oled_change_data_en == 1)
    {

        oled_change_data_en = 0;
        sprintf(oled_str_time, "%d", run_time_t);
        sprintf(oled_str_target_speed, "%d", oled_speed);
        printf("speed2:%d\r\n", oled_speed);
        OLED_ShowString(50, 2, oled_str_target_speed, 1);  //显示速度
        OLED_ShowString(50, 0, oled_str_time, 1);  //显示时间  run_time_t
        if(CAR == 2)
        {
//            sprintf(oled_str_distance, "%d   ", OPENMV_Data.distance);
//            OLED_ShowString(50, 4, oled_str_distance, 1);  //显示时间  run_time_t
                        sprintf(oled_str_distance, "%d   ", OPENMV_Data.distance);
                        OLED_ShowString(50, 4, oled_str_distance, 1);  //显示时间  run_time_t
        }

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
            OLED_ShowNum(50, 0, run_time_t, 3, 1);
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

