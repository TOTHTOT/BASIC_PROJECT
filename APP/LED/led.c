/*
 * led.c
 *
 *  Created on: 2022��7��10��
 *      Author: TOTHTOT
 */

#include "led.h"
/* Led_Init
 * ����:Led��ʼ��
 * ����:LED1,led1��ʼ��ת;LED2,led2��ʼ��:ALL_LED,led1��led2һ���ʼ��
 * */
void Led_Init(uint8_t whitch_led)
{
    switch (whitch_led)
    {
    case LED1:
        GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
        break;
    case LED2:
        GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN7);
        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN7);
        break;
    case ALL_LED:
        GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
        GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
        GPIO_setAsOutputPin(GPIO_PORT_P4, GPIO_PIN7);
        GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN7);

    default:
        break;
    }
}

/* Led_Toggle
 * ����:LED��ת
 * ����:LED1,led1��ת;LED2,led2��ת:ALL_LED,led1��led2һ��ת
 * */
void Led_Toggle(uint8_t whitch_led)
{
    switch (whitch_led)
    {
    case LED1:
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        break;
    case LED2:
        GPIO_toggleOutputOnPin(GPIO_PORT_P4, GPIO_PIN7);
        break;
    case ALL_LED:
        GPIO_toggleOutputOnPin(GPIO_PORT_P4, GPIO_PIN7);
        GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
    default:
        break;

    }
}

/* Led_On
 * ����:LED��
 * ����:LED1,led1��;LED2,led2��:ALL_LED,led1��led2һ����
 * */
void Led_On(uint8_t whitch_led)
{
    switch (whitch_led)
      {
      case LED1:
          GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
          break;
      case LED2:
          GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN7);
          break;
      case ALL_LED:
          GPIO_setOutputHighOnPin(GPIO_PORT_P4, GPIO_PIN7);
          GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
      default:
          break;
      }
}

/* Led_Off
 * ����:LED��
 * ����:LED1,led1��;LED2,led2��:ALL_LED,led1��led2һ����
 * */
void Led_Off(uint8_t whitch_led)
{
    switch (whitch_led)
      {
      case LED1:
          GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
          break;
      case LED2:
          GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN7);
          break;
      case ALL_LED:
          GPIO_setOutputLowOnPin(GPIO_PORT_P4, GPIO_PIN7);
          GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
      default:
          break;
      }
}
