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
#include "tb6612.h"
#include "pid.h"
#include "oled.h"
char i;
extern float Target_value, Actual_value;
extern float Target_value2, Actual_value2;
short M1_encode_num,M2_encode_num;
float Angle_Balance, Gyro_Balance;           //ƽ����� ƽ�������� ת��������
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
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN1);   //����1
    GPIO_enableInterrupt(GPIO_PORT_P2, GPIO_PIN1);
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);   //����2
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN1,
                             GPIO_HIGH_TO_LOW_TRANSITION);   //����1
    GPIO_selectInterruptEdge(GPIO_PORT_P1, GPIO_PIN1,
                             GPIO_HIGH_TO_LOW_TRANSITION);   //����2

    // IFG cleared
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN1);    //����1
    GPIO_clearInterrupt(GPIO_PORT_P1, GPIO_PIN1);    //����2

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
    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN0,
                             GPIO_HIGH_TO_LOW_TRANSITION);   //���1A
    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN2,
                             GPIO_HIGH_TO_LOW_TRANSITION);   //���2A
    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN3,
                             GPIO_HIGH_TO_LOW_TRANSITION);   //���3A
    GPIO_selectInterruptEdge(GPIO_PORT_P2, GPIO_PIN6,
                             GPIO_HIGH_TO_LOW_TRANSITION);   //���4A
    // ����жϱ�־λ
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN0);    //���1A
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN2);    //���2A
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN4);    //���3A
    GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN6);    //���4A

    //���õ��B������
    GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN0);     //���1B
    GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN1);     //���1B
    GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN2);     //���1B
    GPIO_setAsInputPin(GPIO_PORT_P6, GPIO_PIN3);     //���1B
#endif

}


//����С��״̬��Ϣ�Ƿ��ӡ�ڴ���
uint8_t car_state = 1;
extern uint8_t pid_en;
#pragma vector=PORT1_VECTOR         //P1���ж�����
__interrupt void Port_1(void)      //�����жϷ��������ΪPort_1
{
    switch (P1IV)
    {
    case P1IV_P1IFG0:
        break;
    case P1IV_P1IFG1:
        delay_ms(10);                //����
        if (GPIO_getInputPinValue(GPIO_PORT_P1, GPIO_PIN1) == 0)
        {
            //��������

//            speed_pid_m1.target_val = 200;
//            speed_pid_m2.target_val = 200;
            pid_en = 1;

            printf("����\r\n");
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
    case P2IV_P2IFG0:       //��M1�������,�ڼ������ٶȺ�����encode_num
        if (M1_ENCODE_B == 0)
        {
            M1_encode_num++;

//            Car_1.motro_state[0].encode_num++;
//            Car_1.motro_state[0].total_encode_num += Car_1.motro_state[0].encode_num; //�����������
        }
        else if (M1_ENCODE_B == 1)
        {
            M1_encode_num--;
//            Car_1.motro_state[0].encode_num--;
//            Car_1.motro_state[0].total_encode_num += Car_1.motro_state[0].encode_num; //�����������
        }
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN0);
        break;
    case P2IV_P2IFG1:
        delay_ms(10);      //����
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
//            //�����޸��ٶ�
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
            printf("��ť1����\r\n");
        }
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN1);
        break;
    case P2IV_P2IFG2:
        if (M2_ENCODE_B == 0)
        {
            M2_encode_num--;
//            Car_1.motro_state[1].encode_num--;
//            Car_1.motro_state[1].total_encode_num += Car_1.motro_state[1].encode_num; //�����������
        }
        else if (M2_ENCODE_B == 1)
        {
            M2_encode_num++;
//            Car_1.motro_state[1].encode_num++;
//            Car_1.motro_state[1].total_encode_num += Car_1.motro_state[1].encode_num; //�����������
        }
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN2);
        break;
    case P2IV_P2IFG3:
        if (M2_ENCODE_B == 0)
        {
            Car_1.motro_state[3].encode_num++;
            Car_1.motro_state[3].total_encode_num +=
                    Car_1.motro_state[3].encode_num; //�����������
        }
        else if (M2_ENCODE_B == 1)
        {
            Car_1.motro_state[3].encode_num--;
            Car_1.motro_state[3].total_encode_num +=
                    Car_1.motro_state[3].encode_num; //�����������
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
                    Car_1.motro_state[2].encode_num; //�����������
        }
        else if (M2_ENCODE_B == 1)
        {
            Car_1.motro_state[2].encode_num--;
            Car_1.motro_state[2].total_encode_num +=
                    Car_1.motro_state[2].encode_num; //�����������
        }
        GPIO_clearInterrupt(GPIO_PORT_P2, GPIO_PIN2);
        break;
    case P2IV_P2IFG7:
        break;
    default:
        break;
    }
}

