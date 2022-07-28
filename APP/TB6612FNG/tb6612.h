/*
 * tb6612.h
 *
 *  Created on: 2022��7��16��
 *      Author: TOTHTOT
 */

#ifndef APP_TB6612FNG_TB6612_H_
#define APP_TB6612FNG_TB6612_H_

#include "driverlib.h"
#include "pid.h"

//С���������IO�ں궨��
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


// С��������ú궨��
#define Car_CheChang 20                                                // С�����ȵ�λCM
#define Car_CheLunZhiJing 4.8                                          //С������ֱ����λCM
#define Car_CheKuan 15                                                 //С������λCM
#define Car_MAXPWM 1250 * 0.8                                          //����������PWM�İٷ�֮��ʮ
#define Car_PI 3.142                                                   //����Բ����
#define Car_MAXSPEED 120                                               //�������ת��RPM
#define Car_FREQUENCY_DOUBLE 1.0                                         //��Ƶ
#define Car_MOTOR_REDUCTION_RATIO 20                                   //���ٱ�
#define Car_MOTOR_LINE_NUM 13                                          //��Ȧ����
#define Car_MOTOR_COIL (Car_MOTOR_REDUCTION_RATIO *Car_MOTOR_LINE_NUM)   //תһȦ�������
#define Car_MOTOR_PULSE_PER_CYCLE (Car_FREQUENCY_DOUBLE *Car_MOTOR_COIL) //ÿתһȦ������������ 4*260 13*20=260
#define Car_PID_CYCLE 50


// С������ö��
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

// ���״̬�ṹ��
typedef struct
{
    int encode_num;                   //������
    float max_speed;                  //����ٶ�
    float distance;                   //�Լ��߹��ľ���
    float target_distance;            //Ŀ�����,���趨����ʱ����
    int total_encode_num;             //��������
    float speed_output_value;         //���PWMֵ
    float speed_output_value_finally; //����Ѱ�����������PWMֵ
    _pid seppd;                       //�ٶȻ�
    _pid location;                    //λ�û�
} S_MOTOR_STATE;
// С��״̬�ṹ��
typedef struct
{
    E_CAR_DIRECTION direction;
    S_MOTOR_STATE motro_state[4];
    uint16_t car_battery_voltage; //С����ص�ѹԭʼ��ѹֵδ����
    char pid_en;             // pid����ʹ��,�ڴﵽĿ���ֹͣ����PID,Ҳ���Թ���������ʵ��ֹͣ����PID
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
