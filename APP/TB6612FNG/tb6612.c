/*
 * tb6612.c
 *
 *  Created on: 2022年7月16日
 *      Author: TOTHTOT
 */
#include "tb6612.h"
#include "pid.h"
#include "uart1.h"
#include "graysensor.h"

S_CAR_STATE Car_1;

void Car_Init(void)
{
    //小车方向控制初始化
    // M1_Ain1
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN6);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN6);
    // M1_Ain2
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN5);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN5);


    // M2_Bin1
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN0);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN0);
    // M2_Bin2
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN7);


    // M3_Ain1
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN1);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN1);
    // M3_Ain2
    GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN2);


    // M4_Bin1
    GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN7);
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN7);
    // M4_Bin2
    GPIO_setAsOutputPin(GPIO_PORT_P3, GPIO_PIN2);
    GPIO_setOutputLowOnPin(GPIO_PORT_P3, GPIO_PIN2);

}

/**
 * @name: Car_Struct_Init
 * @msg: 结构体的初始化
 * @param {S_CAR_STATE} *car
 * @return {*}
 */
void Car_Struct_Init(S_CAR_STATE *car)
{
    uint8_t i;
    for (i = 0; i < 4; i++)
    {
        car->motro_state[i].distance = 0.0;
        car->motro_state[i].encode_num = 0;
        car->motro_state[i].max_speed = 0;
        car->motro_state[i].speed_output_value = 0.0f;
        car->motro_state[i].speed_output_value_finally = 0.0f;
        car->motro_state[i].target_distance = 0.0f;
        car->motro_state[i].total_encode_num = 0;
    }
    PID_Init(); // pid参数复位
    car->direction = stop;
    car->pid_en = 0; // PID计算完停止
}

/**
 * @name:  Car_Direction
 * @msg: 小车方向控制,方向没有调整
 * @param {E_CAR_DIRECTION} direction 控制正反转
 * @param {u8} motor    选择控制的电机
 * @return {*}
 */
void Car_Direction(E_CAR_DIRECTION direction, uint8_t motor)
{
    switch (motor)
    {
    case 1:
        if (direction == zhengzhuan)
        {
            M1_AIN1_L;
            M1_AIN2_H; //正转
           printf("1 front\r\n");
        }
        else if (direction == fanzhuan)
        {
            M1_AIN1_H;
            M1_AIN2_L; //反转
//            printf("2\r\n");
        }
        else
        {
            M1_AIN1_L;
            M1_AIN2_L; //停止
        }
        break;
    case 2:
        if (direction == zhengzhuan)
        {

            M2_BIN1_L;
            M2_BIN2_H; // 正转
//            printf("3\r\n");
        }
        else if (direction == fanzhuan)
        {
            M2_BIN1_H;
            M2_BIN2_L; //反转
//            printf("4\r\n");
        }
        else
        {
            M2_BIN1_L;
            M2_BIN2_L; //停止
        }
        break;
    case 3:
        if (direction == zhengzhuan)
        {
            M3_AIN1_L;
            M3_AIN2_H; //正转
//            printf("5\r\n");
        }
        else if (direction == fanzhuan)
        {
            M3_AIN1_H;
            M3_AIN2_L; //反转
        }
        else
        {
            M3_AIN1_L;
            M3_AIN2_L; //停止
        }
        break;
    case 4:
        if (direction == zhengzhuan)
        {
            M4_BIN1_H;
            M4_BIN2_L; //正转
        }
        else if (direction == fanzhuan)
        {
            M4_BIN1_L;
            M4_BIN2_H; //反转
        }
        else
        {
            M4_BIN1_L;
            M4_BIN2_L; //停止
        }
        break;
    }
}

/**
 * @name: Car_Go
 * @msg: 小车要走的距离
 * @param {u32} location_cm 小车行驶距离
 * @param {S_CAR_STATE} *car 小车结构体
 * @param {u16} max_speed   小车最大速度,单位RPM,用的还是结构体没改
 * @return {*}
 */
void Car_Go(uint32_t location_cm, S_CAR_STATE *car, uint16_t max_speed)
{
    float car_location = 0.0;
    uint8_t i;
    Car_Struct_Init(car);
    car_location = (location_cm / (Car_CheLunZhiJing * Car_PI)) * (Car_MOTOR_PULSE_PER_CYCLE); //距离转换成脉冲数
    set_pid_target(&car->motro_state[0].location, car_location);
    set_pid_target(&car->motro_state[1].location, car_location);
    set_pid_target(&car->motro_state[2].location, car_location);
    set_pid_target(&car->motro_state[3].location, car_location);
    printf("go staright,distance: %d, %f\r\n", location_cm, car_location);
    car->direction = forward;
    for(i = 0; i < 4;i++)
        car->motro_state[i].max_speed = max_speed;
    // car.
    car->motro_state[0].target_distance = location_cm;
    car->pid_en = 1;
}

void Car_Go_Speed(S_CAR_STATE *car, uint16_t max_speed)
{
    PID_Init();
    printf("pid初始化\r\n");
    set_pid_target(&car->motro_state[0].seppd, max_speed);
    set_pid_target(&car->motro_state[1].seppd, max_speed);
//    set_pid_target(&car->motro_state[2].seppd, max_speed);
//    set_pid_target(&car->motro_state[3].seppd, max_speed);
    car->direction = forward;
    car->pid_en = 1;
}
/**
 * @name: Car_Stop
 * @msg: 小车停止
 * @param {S_CAR_STATE} *car
 * @return {*}
 */
void Car_Stop(S_CAR_STATE *car)
{
    uint8_t i;
    Car_Struct_Init(car);
    for (i = 0; i < 4; i++)
    {
        Car_Direction(stop, i);
    }
    car->pid_en = 0;
    car->direction = stop;
    Timer_A_setCompareValue(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_1, 0);
    Timer_A_setCompareValue(TIMER_A0_BASE,TIMER_A_CAPTURECOMPARE_REGISTER_2, 0);

    printf("停止\r\n");
}

/**
 * @name: Car_Speed_PID
 * @msg: 计算速度pid
 * @param {S_MOTOR_STATE} *motor
 * @return {*}
 */
float  Car_Speed_PID(S_MOTOR_STATE *motor)
{
    float actual_speed = 0.0;
    float cont_value = 0.0;
    //单位时间内的脉冲数转换成速度
    actual_speed = ((float)motor->encode_num /Car_MOTOR_PULSE_PER_CYCLE) * (60.0 * 1000.0 / Car_PID_CYCLE); // rpm
//    printf("actual2_speed:%f\r\n", actual_speed);
    cont_value = speed_pid_realize(&motor->seppd, actual_speed);
//    printf("con2t_value:%f\r\n", cont_value);
    return cont_value;
}


/**
 * @name: Car_Location_PID
 * @msg: 计算位置pid
 * @param {S_MOTOR_STATE} *motor
 * @return {*}
 */
float Car_Location_PID(S_MOTOR_STATE *motor)
{
    float cont_value = 0.0;
    float actual_location = 0.0;

    actual_location = motor->total_encode_num;
    cont_value = location_pid_realize(&motor->location, actual_location);
    if (cont_value > motor->max_speed) //速度限制
    {
        cont_value = motor->max_speed;
    }
    else if (cont_value < -motor->max_speed)
    {
        cont_value = -motor->max_speed;
    }
//    printf("cont:%f\r\n", cont_value);
    return cont_value;
}


/**
 * @name: Location_Speed_Control
 * @msg: 位置速度环控制
 * @param {S_CAR_STATE *} car
 * @return {*}
 */
void Location_Speed_Control(S_CAR_STATE *car)
{
    static uint8_t location_control_count;
    float Location_1_Outval, Location_2_Outval, Location_3_Outval, Location_4_Outval;
    if (location_control_count >= 1)
    {
        location_control_count = 0;
        Location_1_Outval = Car_Location_PID(&car->motro_state[0]);
        Location_2_Outval = Car_Location_PID(&car->motro_state[1]);
        Location_3_Outval = Car_Location_PID(&car->motro_state[2]);
        Location_4_Outval = Car_Location_PID(&car->motro_state[3]);
//         printf("Location_1_Outval:%f, Motor_1_PulseSigma:%d\r\n", Location_1_Outval, Motor_1_PulseSigma);
        // printf("Location_2_Outval:%f, Motor_2_PulseSigma:%d\r\n", Location_2_Outval, Motor_2_PulseSigma);
        // 设置目标值
        set_pid_target(&car->motro_state[0].seppd, Location_1_Outval);
        set_pid_target(&car->motro_state[1].seppd, Location_2_Outval);
        set_pid_target(&car->motro_state[2].seppd, Location_3_Outval);
        set_pid_target(&car->motro_state[3].seppd, Location_4_Outval);
    }
    location_control_count++;
    car->motro_state[0].speed_output_value = Car_Speed_PID(&car->motro_state[0]);
    car->motro_state[1].speed_output_value = Car_Speed_PID(&car->motro_state[1]);
    car->motro_state[2].speed_output_value = Car_Speed_PID(&car->motro_state[2]);
    car->motro_state[3].speed_output_value = Car_Speed_PID(&car->motro_state[3]);
}


/**
 * @name: Motor_Output
 * @msg: PWM输出
 * @param {S_CAR_STATE} car
 * @return {*}
 */
void Motor_Output(S_CAR_STATE car)
{
//    printf("m2_1:%d\r\n", (int)car.motro_state[1].speed_output_value_finally);

//    if (car.motro_state[0].speed_output_value_finally >= 0) //正转
//    {
//        Car_Direction(zhengzhuan, 1);
//    }
//    else
//    {
//        Car_Direction(fanzhuan, 1);
//        car.motro_state[0].speed_output_value_finally = -car.motro_state[0].speed_output_value_finally;
//    }
    car.motro_state[0].speed_output_value_finally = car.motro_state[0].speed_output_value_finally > Car_MAXPWM ? Car_MAXPWM : car.motro_state[0].speed_output_value_finally; //是否超出最大PWM限幅

//    if (car.motro_state[1].speed_output_value_finally >= 0) //正转
//    {
//        Car_Direction(zhengzhuan, 2);
//    }
//    else
//    {
//        Car_Direction(fanzhuan, 2);
//        car.motro_state[1].speed_output_value_finally = -car.motro_state[1].speed_output_value_finally;
//    }
    car.motro_state[1].speed_output_value_finally = car.motro_state[1].speed_output_value_finally > Car_MAXPWM ? Car_MAXPWM : car.motro_state[1].speed_output_value_finally; //是否超出最大PWM限幅

//    printf("m2:%d\r\n", (int)car.motro_state[1].speed_output_value_finally);
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_1, (int)car.motro_state[0].speed_output_value_finally);
    Timer_A_setCompareValue(TIMER_A0_BASE, TIMER_A_CAPTURECOMPARE_REGISTER_2, (int)car.motro_state[1].speed_output_value_finally);

//    TIM_SetCompare1(TIM2, (int)car.motro_state[0].speed_output_value_finally); //设置M1的PWM
//    TIM_SetCompare2(TIM2, (int)car.motro_state[1].speed_output_value_finally); //设置M2的PWM
//    TIM_SetCompare3(TIM2, (int)car.motro_state[2].speed_output_value_finally); //设置M3的PWM
//    TIM_SetCompare4(TIM2, (int)car.motro_state[3].speed_output_value_finally); //设置M4的PWM
}

