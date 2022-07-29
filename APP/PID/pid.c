/*
 * pid.c
 *
 *  Created on: 2022年7月27日
 *      Author: TOTHTOT
 */

#include "pid.h"
#include "uart1.h"
#include "tb6612.h"

_pid speed_pid_m1;
_pid speed_pid_m2;


void PID_Init(void)
{
    /* 速度相关初始化参数 */
    speed_pid_m1.Kp=1;
    speed_pid_m1.Ki=0.222;
    speed_pid_m1.Kd=0;
    speed_pid_m1.target_val = 0;
    speed_pid_m1.output = 0.0;
    speed_pid_m1.err = 0.0;
    speed_pid_m1.err_last = 0.0;
    speed_pid_m1.integral = 0.0;

    /* 速度相关初始化参数 */
    speed_pid_m2.target_val = 0;
    speed_pid_m2.output = 0.0;
    speed_pid_m2.err = 0.0;
    speed_pid_m2.err_last = 0.0;
    speed_pid_m2.integral = 0.0;
    speed_pid_m2.Kp = 1;
    speed_pid_m2.Ki = 0.222;//0.05
    speed_pid_m2.Kd = 0.0;//0.007

}

void set_pid_target(_pid *pid, float temp_val)
{
    pid->target_val = temp_val; // 设置当前的目标值
}

float speed_pid_realize(_pid *pid, float actual_val)
{
    /*计算目标值与实际值的误差*/
    pid->err = pid->target_val - actual_val;

    if ((pid->err < 0.5f) && (pid->err > -0.5f)) //差1这么多可以吗？运行1分钟，位置差为1个轮子的周长
    {
        pid->err = 0.0f;
        printf("into min err");
    }

    pid->integral += pid->err; // 误差累积

    /*积分限幅*/
    if (pid->integral >= 2000)
    {
        pid->integral = 2000;
    }
    else if (pid->integral < -2000)
    {
        pid->integral = -2000;
    }

    /*PID算法实现*/
    pid->output = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->err_last);

    /*误差传递*/
    pid->err_last = pid->err;

    /*返回当前实际值*/
//    printf("%f, %f\r\n", pid->target_val, pid->actual_val);
    return pid->output;
}

float location_pid_realize(_pid *pid, float actual_val) //位置环光个Kp好像也可以
{

}


//
//
///*
// * pid.c
// *
// *  Created on: 2022年7月27日
// *      Author: TOTHTOT
// */
//
//#include "pid.h"
//#include "uart1.h"
//#include "tb6612.h"
//
//_pid speed_pid_m1;
//_pid speed_pid_m2;


//void PID_Init(void)
//{
//    /* 速度相关初始化参数 */
//    speed_pid_m1.Kp=1;
//    speed_pid_m1.Ki=0.222;
//    speed_pid_m1.Kd=0;
//    speed_pid_m1.target_val = 200;
//    speed_pid_m1.output = 0.0;
//    speed_pid_m1.err = 0.0;
//    speed_pid_m1.err_last = 0.0;
//    speed_pid_m1.integral = 0.0;
//
//    /* 速度相关初始化参数 */
//    speed_pid_m2.target_val = 200;
//    speed_pid_m2.output = 0.0;
//    speed_pid_m2.err = 0.0;
//    speed_pid_m2.err_last = 0.0;
//    speed_pid_m2.integral = 0.0;
//    speed_pid_m2.Kp = 1;
//    speed_pid_m2.Ki = 0.222;//0.05
//    speed_pid_m2.Kd = 0.0;//0.007
//
//}
//
//void set_pid_target(_pid *pid, float temp_val)
//{
//    pid->target_val = temp_val; // 设置当前的目标值
//}
//
//float speed_pid_realize(_pid *pid, float actual_val)
//{
//    /*计算目标值与实际值的误差*/
//    pid->err = pid->target_val - actual_val;
//
//    if ((pid->err < 0.5f) && (pid->err > -0.5f)) //差1这么多可以吗？运行1分钟，位置差为1个轮子的周长
//    {
//        pid->err = 0.0f;
//        printf("into min err");
//    }
//
//    pid->integral += pid->err; // 误差累积
//
//    /*积分限幅*/
//    if (pid->integral >= 2000)
//    {
//        pid->integral = 2000;
//    }
//    else if (pid->integral < -2000)
//    {
//        pid->integral = -2000;
//    }
//
//    /*PID算法实现*/
//    pid->output = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->err_last);
//
//    /*误差传递*/
//    pid->err_last = pid->err;
//
//    /*返回当前实际值*/
////    printf("%f, %f\r\n", pid->target_val, pid->actual_val);
//    return pid->output;
//}
//
//float location_pid_realize(_pid *pid, float actual_val) //位置环光个Kp好像也可以
//{
//    /*计算目标值与实际值的误差*/
////    pid->err = pid->target_val - actual_val;
//
//    //    /* 设定闭环死区 */   //外环死区可以不要
//    //    if((pid->err >= -0.1) && (pid->err <= 0.1))
//    //    {
//    //      pid->err = 0;
//    //      pid->integral = 0;
//    //    }
//
////    pid->integral += pid->err; // 误差累积
////
////    /*PID算法实现*/
////    pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->integral + pid->Kd * (pid->err - pid->err_last);
////    // printf("                        pid_target:%f, pid_actual:%f\r\n", pid->target_val, pid->actual_val);
////
////    /*误差传递*/
////    pid->err_last = pid->err;
////
////    /*返回当前实际值*/
////    return pid->actual_val;
//}






