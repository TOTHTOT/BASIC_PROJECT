/*
 * pid.h
 *
 *  Created on: 2022��7��27��
 *      Author: TOTHTOT
 */

#ifndef APP_PID_PID_H_
#define APP_PID_PID_H_

#include "driverlib.h"

typedef struct
{
    float target_val; //Ŀ��ֵ
    float actual_val; //ʵ��ֵ
    float err;        //����ƫ��ֵ
    float err_last;   //������һ��ƫ��ֵ
    float Kp, Ki, Kd; //������������֡�΢��ϵ��
    float integral;   //�������ֵ
} _pid;


void PID_Init(void);
void set_pid_target(_pid *pid, float temp_val);
float speed_pid_realize(_pid *pid, float actual_val);
float location_pid_realize(_pid *pid, float actual_val);


#endif /* APP_PID_PID_H_ */
