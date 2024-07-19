#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"                  // Device header
#include "stdbool.h"

typedef enum 
{
    LED1 = 1,
    LED2,
    LED3
}LED_N;

typedef enum
{
    OFF = 0,
    ON = 1,
    TRABS = 2
}LED_Statue;

#define LED_Port GPIOB
#define LED_Pin_1 GPIO_Pin_9
#define LED_Pin_2 GPIO_Pin_8

void LED_GPIO_Init(void);
void LED_Set(LED_N LED_x,bool state);

#endif
