/*
 * pid.h
 *
 *  Created on: 2022年7月27日
 *      Author: TOTHTOT
 */

#ifndef APP_PID_PID_H_
#define APP_PID_PID_H_

#include "driverlib.h"

typedef struct
{
    float target_val; //目标值
    float actual_val; //实际值
    float err;        //定义偏差值
    float err_last;   //定义上一个偏差值
    float Kp, Ki, Kd; //定义比例、积分、微分系数
    float integral;   //定义积分值
} _pid;


void PID_Init(void);
void set_pid_target(_pid *pid, float temp_val);
float speed_pid_realize(_pid *pid, float actual_val);
float location_pid_realize(_pid *pid, float actual_val);


#endif /* APP_PID_PID_H_ */
