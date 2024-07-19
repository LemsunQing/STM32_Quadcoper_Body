#include "LED.h"

void LED_GPIO_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Pin = LED_Pin_1 | LED_Pin_2;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;

    GPIO_Init(LED_Port,&GPIO_InitStruct);

    GPIO_SetBits(LED_Port,LED_Pin_1 | LED_Pin_2);       //初始化，LED置高电平全灭
}

/// @brief LED控制亮灭,转换
/// @param NLED_x 
/// @param state 

void LED_Set(LED_N NLED_x,bool state)
{
    switch (NLED_x)
    {
        case LED1:
        state ?
        GPIO_ResetBits(LED_Port,LED_Pin_1):
        GPIO_SetBits(LED_Port,LED_Pin_1);
        break;
        case LED2:
        state ?
        GPIO_ResetBits(LED_Port,LED_Pin_2):
        GPIO_SetBits(LED_Port,LED_Pin_2);
        break;
    }

}
