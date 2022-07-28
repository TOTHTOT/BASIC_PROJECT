/*
 * graysensor.c
 *
 *  Created on: 2022年7月28日
 *      Author: TOTHTOT
 */
#include "graysensor.h"
void GraySensor_Init(void)
{
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN5);   //中间的灰度
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN3);   //左边的灰度
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P7, GPIO_PIN4);   //右边的灰度

}


// 获取两个灰度传感器的值,都为低电平表示灰度传感器红线在中间
short Car_Staright_Control(void)
{
    if(LIFT_GRAYSENSOR == 0 &&MID_GRAYSENSOR == 1 && RIGHT_GRAYSENSOR == 0)
    {
        return 0;
    }
    else if(LIFT_GRAYSENSOR == 1 &&MID_GRAYSENSOR == 0 && RIGHT_GRAYSENSOR == 0)
    {
        return -300;
    }
    else if(LIFT_GRAYSENSOR == 0 &&MID_GRAYSENSOR == 0 && RIGHT_GRAYSENSOR == 1)
    {
        return 300;
    }
}



