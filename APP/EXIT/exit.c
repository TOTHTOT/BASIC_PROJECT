/*
 * exit.c
 *
 *  Created on: 2022年7月11日
 *      Author: TOTHTOT
 */
#include "exit.h"
#include "uart1.h"
#include "delay.h"
#include "bh1750.h"
#include "mpu6050.h"
#include "config.h"
#include "tb6612.h"
#include "pid.h"
#include "oled.h"
char i;
extern float Target_value, Actual_value;
extern float Target_value2, Actual_value2;
short M1_encode_num,M2_encode_num;
float Angle_Balance, Gyro_Balance;           //平衡倾角 平衡陀螺仪 转向陀螺仪
        /* EXIT_Init
         * 描述:外部中断初始化,两个按键
         * */

/*void Get_Angle(void)
 {
 Read_DMP();                      //===读取加速度、角速度、倾角
 Angle_Balance=Pitch;             //===更新平衡倾角
 Gyro_Balance=gyro[i];            //===更新平衡角速度
 }*/
void EXIT_Init(void)
{
    //Enable internal resistance as pull-down resistance and interrupt enabled
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN1);   //按键1
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);   //按键2
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN1,
                             GPIO_HIGH_TO_LOW_TRANSITION);   //按键1
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN1,
                             GPIO_HIGH_TO_LOW_TRANSITION);   //按键2

    // IFG cleared
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN1);    //按键1
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);    //按键2

#if USE_MPU6050
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2,GPIO_PIN4);   //MPU6050中断
    GPIO_enableInterrupt(GPIO_PORT_P2,GPIO_PIN4);
    GPIO_selectInterruptEdge(GPIO_PORT_P2,GPIO_PIN1,GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterrupt(GPIO_PORT_P2,GPIO_PIN1);
#endif

#if USE_MOTOR
    // 软件编码器GPIO口
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN0);   //电机1A
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN2);   //电机2A
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN2);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN3);   //电机3A
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN3);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN6);   //电机4A
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN6);

    // 软件编码器中断边沿
    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN0,
                             GPIO_HIGH_TO_LOW_TRANSITION);   //电机1A
    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN2,
                             GPIO_HIGH_TO_LOW_TRANSITION);   //电机2A
    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN3,
                             GPIO_HIGH_TO_LOW_TRANSITION);   //电机3A
    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN6,
                             GPIO_HIGH_TO_LOW_TRANSITION);   //电机4A
    // 清除中断标志位
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN0);    //电机1A
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN2);    //电机2A
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN4);    //电机3A
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN6);    //电机4A

    //设置电机B相输入
    GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN0);     //电机1B
    GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN1);     //电机1B
    GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN2);     //电机1B
    GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN3);     //电机1B
#endif

}


//控制小车状态信息是否打印在串口
uint8_t car_state = 1;
extern uint8_t pid_en;
#pragma vector=PORT1_VECTOR         //P1口中断向量
__interrupt void Port_1(void)      //声明中断服务程序，名为Port_1
{
    switch (P1IV)
    {
    case P1IV_P1IFG0:
        break;
    case P1IV_P1IFG1:
        delay_ms(10);                //消抖
        if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == 0)
        {
            //按下启动

//            speed_pid_m1.target_val = 200;
//            speed_pid_m2.target_val = 200;
            pid_en = 1;

            printf("启动\r\n");
        }
        GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);
        break;
    case P1IV_P1IFG2:
        break;
    case P1IV_P1IFG3:
        break;
    case P1IV_P1IFG4:
        break;
    case P1IV_P1IFG5:
        break;
    case P1IV_P1IFG6:
        break;
    case P1IV_P1IFG7:
        break;
    default:
        break;
    }
}
static uint8_t key_press_times = 1;
#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    uint8_t h, l;
    short key_press_time;
    switch (P2IV)
    {
    case P2IV_P2IFG0:       //对M1脉冲计数,在计算完速度后清零encode_num
        if (M1_ENCODE_B == 0)
        {
            M1_encode_num++;

//            Car_1.motro_state[0].encode_num++;
//            Car_1.motro_state[0].total_encode_num += Car_1.motro_state[0].encode_num; //总脉冲数相加
        }
        else if (M1_ENCODE_B == 1)
        {
            M1_encode_num--;
//            Car_1.motro_state[0].encode_num--;
//            Car_1.motro_state[0].total_encode_num += Car_1.motro_state[0].encode_num; //总脉冲数相加
        }
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN0);
        break;
    case P2IV_P2IFG1:
        delay_ms(10);      //消抖
        if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN1) == 0)
        {
//            key_press_times++;
            if(key_press_times == 1)
            {
                oled_speed = 150;
                key_press_times = 2;
                speed_pid_m1.target_val = oled_speed;
                speed_pid_m2.target_val = oled_speed;
            }
            else if(key_press_times == 2)
            {
                oled_speed = 200;
                key_press_times = 3;
                speed_pid_m1.target_val = oled_speed;
                speed_pid_m2.target_val = oled_speed;
            }
            else if(key_press_times == 3)
            {
                oled_speed = 300;
                key_press_times = 1;
                speed_pid_m1.target_val = oled_speed;
                speed_pid_m2.target_val = oled_speed;

            }
            oled_change_data_en = 1;
            printf("speed1%d\r\n", oled_speed);

//
//            //按下修改速度
//            while(GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN1) == 0)
//            {
//                key_press_time++;
//                delay_ms(1);
//            }
//            if(key_press_time>500)
//            {
//                printf("speed:200\r\n");
//                set_pid_target(&speed_pid_m1, 200);
//                set_pid_target(&speed_pid_m2, 200);
//            }
//            else if (key_press_time < 300)
//            {
//
//                delay_ms(100);
//                if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN1) == 0)
//                {
//                    printf("speed:300\r\n");
//                    set_pid_target(&speed_pid_m1, 300);
//                    set_pid_target(&speed_pid_m2, 300);
//                }
//                else
//                {
//                    printf("speed:150\r\n");
//                    set_pid_target(&speed_pid_m1, 150);
//                    set_pid_target(&speed_pid_m2, 150);
//                }
//            }
            printf("按钮1按下\r\n");
        }
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN1);
        break;
    case P2IV_P2IFG2:
        if (M2_ENCODE_B == 0)
        {
            M2_encode_num--;
//            Car_1.motro_state[1].encode_num--;
//            Car_1.motro_state[1].total_encode_num += Car_1.motro_state[1].encode_num; //总脉冲数相加
        }
        else if (M2_ENCODE_B == 1)
        {
            M2_encode_num++;
//            Car_1.motro_state[1].encode_num++;
//            Car_1.motro_state[1].total_encode_num += Car_1.motro_state[1].encode_num; //总脉冲数相加
        }
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN2);
        break;
    case P2IV_P2IFG3:
        if (M2_ENCODE_B == 0)
        {
            Car_1.motro_state[3].encode_num++;
            Car_1.motro_state[3].total_encode_num +=
                    Car_1.motro_state[3].encode_num; //总脉冲数相加
        }
        else if (M2_ENCODE_B == 1)
        {
            Car_1.motro_state[3].encode_num--;
            Car_1.motro_state[3].total_encode_num +=
                    Car_1.motro_state[3].encode_num; //总脉冲数相加
        }
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN3);
        break;
    case P2IV_P2IFG4:
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN4);
        break;
    case P2IV_P2IFG5:
        break;
    case P2IV_P2IFG6:
        if (M2_ENCODE_B == 0)
        {
            Car_1.motro_state[2].encode_num++;
            Car_1.motro_state[2].total_encode_num +=
                    Car_1.motro_state[2].encode_num; //总脉冲数相加
        }
        else if (M2_ENCODE_B == 1)
        {
            Car_1.motro_state[2].encode_num--;
            Car_1.motro_state[2].total_encode_num +=
                    Car_1.motro_state[2].encode_num; //总脉冲数相加
        }
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN2);
        break;
    case P2IV_P2IFG7:
        break;
    default:
        break;
    }
}

