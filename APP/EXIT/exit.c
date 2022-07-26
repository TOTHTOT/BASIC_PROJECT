/*
 * exit.c
 *
 *  Created on: 2022��7��11��
 *      Author: TOTHTOT
 */
#include "exit.h"
#include "uart1.h"
#include "delay.h"
#include "bh1750.h"
#include "mpu6050.h"
#include "config.h"
char i;
float Angle_Balance,Gyro_Balance;           //ƽ����� ƽ�������� ת��������
/* EXIT_Init
 * ����:�ⲿ�жϳ�ʼ��,��������
 * */


/*void Get_Angle(void)
{
    Read_DMP();                      //===��ȡ���ٶȡ����ٶȡ����
    Angle_Balance=Pitch;             //===����ƽ�����
    Gyro_Balance=gyro[i];            //===����ƽ����ٶ�
}*/
void EXIT_Init(void)
{
    //Enable internal resistance as pull-down resistance and interrupt enabled
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2,GPIO_PIN1);   //����1
    GPIO_enableInterrupt(GPIO_PORT_P2,GPIO_PIN1);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1,GPIO_PIN1);   //����2
    GPIO_enableInterrupt(GPIO_PORT_P1,GPIO_PIN1);

    GPIO_selectInterruptEdge(GPIO_PORT_P2,GPIO_PIN1,GPIO_HIGH_TO_LOW_TRANSITION);   //����1
    GPIO_selectInterruptEdge(GPIO_PORT_P1,GPIO_PIN1,GPIO_HIGH_TO_LOW_TRANSITION);   //����2


    // IFG cleared
    GPIO_clearInterrupt(GPIO_PORT_P2,GPIO_PIN1);    //����1
    GPIO_clearInterrupt(GPIO_PORT_P1,GPIO_PIN1);    //����2

#if USE_MPU6050
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2,GPIO_PIN4);   //MPU6050�ж�
    GPIO_enableInterrupt(GPIO_PORT_P2,GPIO_PIN4);
    GPIO_selectInterruptEdge(GPIO_PORT_P2,GPIO_PIN1,GPIO_HIGH_TO_LOW_TRANSITION);
    GPIO_clearInterrupt(GPIO_PORT_P2,GPIO_PIN1);
#endif

#if USE_MOTOR
    // ���������GPIO��
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN0);   //���1A
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN0);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN2);   //���2A
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN2);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN3);   //���3A
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN3);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN6);   //���4A
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN6);

    // ����������жϱ���
    GPIO_selectInterruptEdge(GPIO_PORT_P2,GPIO_PIN0,GPIO_LOW_TO_HIGH_TRANSITION);//���1A
    GPIO_selectInterruptEdge(GPIO_PORT_P2,GPIO_PIN2,GPIO_LOW_TO_HIGH_TRANSITION);//���2A
    GPIO_selectInterruptEdge(GPIO_PORT_P2,GPIO_PIN3,GPIO_LOW_TO_HIGH_TRANSITION);//���3A
    GPIO_selectInterruptEdge(GPIO_PORT_P2,GPIO_PIN6,GPIO_LOW_TO_HIGH_TRANSITION);//���4A
    // ����жϱ�־λ
    GPIO_clearInterrupt(GPIO_PORT_P2,GPIO_PIN0);    //���1A
    GPIO_clearInterrupt(GPIO_PORT_P2,GPIO_PIN2);    //���2A
    GPIO_clearInterrupt(GPIO_PORT_P2,GPIO_PIN4);    //���3A
    GPIO_clearInterrupt(GPIO_PORT_P2,GPIO_PIN6);    //���4A

    //���õ��B������
    GPIO_setAsInputPin(GPIO_PORT_P6,GPIO_PI0);     //���1B
    GPIO_setAsInputPin(GPIO_PORT_P6,GPIO_PI1);     //���1B
    GPIO_setAsInputPin(GPIO_PORT_P6,GPIO_PI2);     //���1B
    GPIO_setAsInputPin(GPIO_PORT_P6,GPIO_PI3);     //���1B
#endif

}

#pragma vector=PORT1_VECTOR         //P1���ж�����
__interrupt void Port_1 (void)      //�����жϷ��������ΪPort_1
{
    switch (P1IV)
        {
        case P1IV_P1IFG0:
            break;
        case P1IV_P1IFG1:
                delay_ms(10);//����
                if(GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == 0)
                {
                    printf("��ť2����\r\n");
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
    float pitch, roll, yaw; //ŷ����
    switch (P2IV)
    {
    case P2IV_P2IFG0:
        break;
    case P2IV_P2IFG1:
            delay_ms(10);      //����
            if (GPIO_getInputPinValue(GPIO_PORT_P2, GPIO_PIN1) == 0)
            {
                Get_BH1750Data(&h, &l );
                printf("��ť1����\r\n");
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





