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
char i;
float Angle_Balance,Gyro_Balance;           //平衡倾角 平衡陀螺仪 转向陀螺仪
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
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2,GPIO_PIN1);   //按键1
    GPIO_enableInterrupt(GPIO_PORT_P2,GPIO_PIN1);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN1);   //按键2
    GPIO_enableInterrupt(GPIO_PORT_P1,GPIO_PIN1);

    GPIO_selectInterruptEdge(GPIO_PORT_P2,GPIO_PIN1,GPIO_HIGH_TO_LOW_TRANSITION);   //按键1
    GPIO_selectInterruptEdge(GPIO_PORT_P1,GPIO_PIN1,GPIO_HIGH_TO_LOW_TRANSITION);   //按键2


    // IFG cleared
    GPIO_clearInterrupt(GPIO_PORT_P2,GPIO_PIN1);    //按键1
    GPIO_clearInterrupt(GPIO_PORT_P1,GPIO_PIN1);    //按键2

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
    GPIO_selectInterruptEdge(GPIO_PORT_P2,GPIO_PIN0,GPIO_LOW_TO_HIGH_TRANSITION);//电机1A
    GPIO_selectInterruptEdge(GPIO_PORT_P2,GPIO_PIN2,GPIO_LOW_TO_HIGH_TRANSITION);//电机2A
    GPIO_selectInterruptEdge(GPIO_PORT_P2,GPIO_PIN3,GPIO_LOW_TO_HIGH_TRANSITION);//电机3A
    GPIO_selectInterruptEdge(GPIO_PORT_P2,GPIO_PIN6,GPIO_LOW_TO_HIGH_TRANSITION);//电机4A
    // 清除中断标志位
    GPIO_clearInterrupt(GPIO_PORT_P2,GPIO_PIN0);    //电机1A
    GPIO_clearInterrupt(GPIO_PORT_P2,GPIO_PIN2);    //电机2A
    GPIO_clearInterrupt(GPIO_PORT_P2,GPIO_PIN4);    //电机3A
    GPIO_clearInterrupt(GPIO_PORT_P2,GPIO_PIN6);    //电机4A

    //设置电机B相输入
    GPIO_setAsInputPin(GPIO_PORT_P6,GPIO_PI0);     //电机1B
    GPIO_setAsInputPin(GPIO_PORT_P6,GPIO_PI1);     //电机1B
    GPIO_setAsInputPin(GPIO_PORT_P6,GPIO_PI2);     //电机1B
    GPIO_setAsInputPin(GPIO_PORT_P6,GPIO_PI3);     //电机1B
#endif

}

#pragma vector=PORT1_VECTOR         //P1口中断向量
__interrupt void Port_1 (void)      //声明中断服务程序，名为Port_1
{
    switch (P1IV)
        {
        case P1IV_P1IFG0:
            break;
        case P1IV_P1IFG1:
                delay_ms(10);//消抖
                if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == 0)
                {
                    printf("按钮2按下\r\n");
                }
                GPIO_clearInterrupt(GPIO_PORT_P1,GPIO_PIN1);
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

#pragma vector=PORT2_VECTOR
__interrupt void Port_2(void)
{
    uint8_t h, l;
    float pitch, roll, yaw; //欧拉角
    switch (P2IV)
    {
    case P2IV_P2IFG0:
        break;
    case P2IV_P2IFG1:
            delay_ms(10);      //消抖
            if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN1) == 0)
            {
                Get_BH1750Data(&h, &l );
                printf("按钮1按下\r\n");
            }
            GPIO_clearInterrupt(GPIO_PORT_P2,GPIO_PIN1);
        break;
    case P2IV_P2IFG2:
        break;
    case P2IV_P2IFG3:
        break;
    case P2IV_P2IFG4:
       /* Get_Angle();
        i++;
        printf("gyro:%d, %d, %d\r\naccel:%d, %d, %d, Pitch:%f\r\nRoll:%f",
               gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], Pitch, Roll);
        printf("temp:%d\r\n", Read_Temperature());
*/
        GPIO_clearInterrupt(GPIO_PORT_P2,GPIO_PIN4);
        break;
    case P2IV_P2IFG5:
        break;
    case P2IV_P2IFG6:
        break;
    case P2IV_P2IFG7:
        break;
    default:
        break;
    }
}





