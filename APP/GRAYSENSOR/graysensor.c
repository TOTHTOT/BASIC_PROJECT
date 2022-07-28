/*
 * graysensor.c
 *
 *  Created on: 2022��7��28��
 *      Author: TOTHTOT
 */
#include "graysensor.h"
void GraySensor_Init(void)
{
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P2, GPIO_PIN5);   //�м�ĻҶ�
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P4, GPIO_PIN3);   //��ߵĻҶ�
    GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P7, GPIO_PIN4);   //�ұߵĻҶ�

}


// ��ȡ�����Ҷȴ�������ֵ,��Ϊ�͵�ƽ��ʾ�Ҷȴ������������м�
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



