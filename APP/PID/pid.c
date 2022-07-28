/*
 * pid.c
 *
 *  Created on: 2022��7��27��
 *      Author: TOTHTOT
 */

#include "pid.h"
#include "uart1.h"
#include "tb6612.h"

void PID_Init(void)
{
    uint8_t i;
    /* �ٶ���س�ʼ������ */
    Car_1.motro_state[0].seppd.target_val = 0.0;
    Car_1.motro_state[0].seppd.actual_val = 0.0;
    Car_1.motro_state[0].seppd.err = 0.0;
    Car_1.motro_state[0].seppd.err_last = 0.0;
    Car_1.motro_state[0].seppd.integral = 0.0;

    Car_1.motro_state[0].seppd.Kp = 1;
    Car_1.motro_state[0].seppd.Ki = 0.0;
    Car_1.motro_state[0].seppd.Kd = 0.0;

    /* �ٶ���س�ʼ������ */
    Car_1.motro_state[1].seppd.target_val = 0.0;
    Car_1.motro_state[1].seppd.actual_val = 0.0;
    Car_1.motro_state[1].seppd.err = 0.0;
    Car_1.motro_state[1].seppd.err_last = 0.0;
    Car_1.motro_state[1].seppd.integral = 0.0;
    Car_1.motro_state[1].seppd.Kp = 1;
    Car_1.motro_state[1].seppd.Ki = 0.0;//0.05
    Car_1.motro_state[1].seppd.Kd = 0.0;//0.007

//    for(i = 0; i < 4; i++)
//    {
//        Car_1.motro_state[i].seppd.Kp = 1.9;
//        Car_1.motro_state[i].seppd.Ki = 0.03;
//        Car_1.motro_state[i].seppd.Kd = 0.05;
//    }

}

void set_pid_target(_pid *pid, float temp_val)
{
    pid->target_val = temp_val; // ���õ�ǰ��Ŀ��ֵ
}

float speed_pid_realize(_pid *pid, float actual_val)
{
    /*����Ŀ��ֵ��ʵ��ֵ�����*/
    pid->err = pid->target_val - actual_val;

    if ((pid->err < 0.5f) && (pid->err > -0.5f)) //��1��ô�����������1���ӣ�λ�ò�Ϊ1�����ӵ��ܳ�
    {
        pid->err = 0.0f;
        printf("into min err");
    }

    pid->integral += pid->err; // ����ۻ�

    /*�����޷�*/
    if (pid->integral >= 2000)
    {
        pid->integral = 2000;
    }
    else if (pid->integral < -2000)
    {
        pid->integral = -2000;
    }

    /*PID�㷨ʵ��*/
    pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->err_last);

    /*����*/
    pid->err_last = pid->err;

    /*���ص�ǰʵ��ֵ*/
//    printf("%f, %f\r\n", pid->target_val, pid->actual_val);
    return pid->actual_val;
}

float location_pid_realize(_pid *pid, float actual_val) //λ�û����Kp����Ҳ����
{
    /*����Ŀ��ֵ��ʵ��ֵ�����*/
    pid->err = pid->target_val - actual_val;

    //    /* �趨�ջ����� */   //�⻷�������Բ�Ҫ
    //    if((pid->err >= -0.1) && (pid->err <= 0.1))
    //    {
    //      pid->err = 0;
    //      pid->integral = 0;
    //    }

    pid->integral += pid->err; // ����ۻ�

    /*PID�㷨ʵ��*/
    pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->err_last);
    // printf("                        pid_target:%f, pid_actual:%f\r\n", pid->target_val, pid->actual_val);

    /*����*/
    pid->err_last = pid->err;

    /*���ص�ǰʵ��ֵ*/
    return pid->actual_val;
}




