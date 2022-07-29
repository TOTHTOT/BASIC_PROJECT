/*
 * graysensor.c
 *
 *  Created on: 2022��7��28��
 *      Author: TOTHTOT
 */
#include "graysensor.h"
#include "uart1.h"
short graysensor = 120;
uint8_t huidu_l_en = 1, huidu_r_en = 1;
void GraySensor_Init(void)
{
    GPIO_setAsInputPin(GPIO_PORT_P2, GPIO_PIN5);   //�м�ĻҶ�
    GPIO_setAsInputPin(GPIO_PORT_P4, GPIO_PIN3);   //��ߵĻҶ�
    GPIO_setAsInputPin(GPIO_PORT_P7, GPIO_PIN4);   //�ұߵĻҶ�
}


// ��ȡ�����Ҷȴ�������ֵ,��Ϊ�͵�ƽ��ʾ�Ҷȴ������������м�
short Car_Staright_Control(void)
{
    if(LIFT_GRAYSENSOR == 0  && RIGHT_GRAYSENSOR == 0)
    {
        printf("�м�\r\n");
        return 0;
    }
    else if(LIFT_GRAYSENSOR == 1 && RIGHT_GRAYSENSOR == 0)
    {
        printf("���\r\n");
        return 230;
    }
    else if(LIFT_GRAYSENSOR == 0  && RIGHT_GRAYSENSOR == 1)
    {
        printf("�ұ�\r\n");
        return -230;
    }
    return 0;
}



