/*
 * tb6612.h
 *
 *  Created on: 2022年7月16日
 *      Author: TOTHTOT
 */

#ifndef APP_TB6612FNG_TB6612_H_
#define APP_TB6612FNG_TB6612_H_

#include "driverlib.h"
#include "pid.h"

//小车方向输出IO口宏定义
#define Car_IO_HIGH 1
#define Car_IO_LOW 0
#define M1_AIN1_H GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);
#define M1_AIN1_L GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
#define M1_AIN2_H GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);
#define M1_AIN2_L GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);


#define M2_BIN1_H GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);
#define M2_BIN1_L GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
#define M2_BIN2_H GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);
#define M2_BIN2_L GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);


#define M3_AIN1_H GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);
#define M3_AIN1_L GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
#define M3_AIN2_H GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);
#define M3_AIN2_L GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);


#define M4_BIN1_H GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
#define M4_BIN1_L GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
#define M4_BIN2_H GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN2);
#define M4_BIN2_L GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2);


#define M1_AIN2 PCout(1)

#define M2_BIN1 PCout(2)
#define M2_BIN2 PCout(3)

#define M3_AIN1 PCout(4)
#define M3_AIN2 PCout(5)

#define M4_BIN1 PBout(0)
#define M4_BIN2 PBout(1)


// 小车相关配置宏定义
#define Car_CheChang 20                                                // 小车长度单位CM
#define Car_CheLunZhiJing 4.8                                          //小车车轮直径单位CM
#define Car_CheKuan 15                                                 //小车车宽单位CM
#define Car_MAXPWM 1250 * 0.8                                          //定义输出最大PWM的百分之八十
#define Car_PI 3.142                                                   //定义圆周率
#define Car_MAXSPEED 120                                               //定义最大转速RPM
#define Car_FREQUENCY_DOUBLE 1.0                                         //倍频
#define Car_MOTOR_REDUCTION_RATIO 20                                   //减速比
#define Car_MOTOR_LINE_NUM 13                                          //线圈线数
#define Car_MOTOR_COIL (Car_MOTOR_REDUCTION_RATIO *Car_MOTOR_LINE_NUM)   //转一圈输出脉冲
#define Car_MOTOR_PULSE_PER_CYCLE (Car_FREQUENCY_DOUBLE *Car_MOTOR_COIL) //每转一圈产生的脉冲数 4*260 13*20=260
#define Car_PID_CYCLE 50


// 小车方向枚举
typedef enum
{
    zhengzhuan,
    fanzhuan,
    forward,
    retreat,
    turn_right,
    turn_left,
    stop
} E_CAR_DIRECTION;

// 电机状态结构体
typedef struct
{
    int encode_num;                   //脉冲数
    float max_speed;                  //最大速度
    float distance;                   //以及走过的距离
    float target_distance;            //目标距离,在设定距离时设置
    int total_encode_num;             //总脉冲数
    float speed_output_value;         //输出PWM值
    float speed_output_value_finally; //经过寻迹最终输出的PWM值
    _pid seppd;                       //速度环
    _pid location;                    //位置环
} S_MOTOR_STATE;
// 小车状态结构体
typedef struct
{
    E_CAR_DIRECTION direction;
    S_MOTOR_STATE motro_state[4];
    uint16_t car_battery_voltage; //小车电池电压原始电压值未换算
    char pid_en;             // pid计算使能,在达到目标后停止计算PID,也可以挂起任务来实现停止计算PID
} S_CAR_STATE;

extern S_CAR_STATE Car_1;


#define M1_A1_H GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN6);
#define M1_A1_L GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
#define M1_A2_H GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN5);
#define M1_A2_L GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);

#define M2_B1_H GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN0);
#define M2_B1_L GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
#define M2_B2_H GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN7);
#define M2_B2_L GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);

#define M3_A1_H GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN1);
#define M3_A1_L GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
#define M3_A2_H GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN2);
#define M3_A2_L GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);

#define M4_B1_H GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN7);
#define M4_B1_L GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
#define M4_B2_H GPIO_setOutputHighOnPin(GPIO_PORT_P3, GPIO_PIN2);
#define M4_B2_L GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2);

void Car_Init(void);
void Car_Direction(E_CAR_DIRECTION direction, uint8_t motor);
void Car_Struct_Init(S_CAR_STATE *car);
void Location_Speed_Control(S_CAR_STATE *car);
void Motor_Output(S_CAR_STATE car);
float Car_Location_PID(S_MOTOR_STATE *motor);
float Car_Speed_PID(S_MOTOR_STATE *motor);

void Car_Go(uint32_t location_cm, S_CAR_STATE *car, uint16_t max_speed);
void Car_Stop(S_CAR_STATE *car);
void Car_Go_Speed(S_CAR_STATE *car, uint16_t max_speed);

#endif /* APP_TB6612FNG_TB6612_H_ */
