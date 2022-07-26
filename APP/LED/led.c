/*
 * led.c
 *
 *  Created on: 2022年7月10日
 *      Author: TOTHTOT
 */

#include "led.h"
/* Led_Init
 * 描述:Led初始化
 * 参数:LED1,led1初始化转;LED2,led2初始化:ALL_LED,led1和led2一起初始化
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
 * 描述:LED翻转
 * 参数:LED1,led1翻转;LED2,led2翻转:ALL_LED,led1和led2一起翻转
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
 * 描述:LED亮
 * 参数:LED1,led1亮;LED2,led2亮:ALL_LED,led1和led2一起亮
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
 * 描述:LED灭
 * 参数:LED1,led1灭;LED2,led2灭:ALL_LED,led1和led2一起灭
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
