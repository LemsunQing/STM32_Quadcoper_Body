#include "stm32f10x.h"                  // Device header
#include "SpeedControl.h"

void Wing_GPIO_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);

    GPIO_InitTypeDef GPIO_InitStruct;

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOA,&GPIO_InitStruct);

    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB,&GPIO_InitStruct);

    GPIO_ResetBits(GPIOA,Wing_Pin_LUA);
    GPIO_ResetBits(GPIOA,Wing_Pin_LUA);
    GPIO_ResetBits(GPIOB,Wing_Pin_LDB);
    GPIO_ResetBits(GPIOB,Wing_Pin_RDB);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimBaseStruct;
    TIM_TimBaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimBaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimBaseStruct.TIM_Period = 100 - 1;
    TIM_TimBaseStruct.TIM_Prescaler = 36 - 1;
    TIM_TimBaseStruct.TIM_RepetitionCounter = 0;

    TIM_TimeBaseInit(TIM1, &TIM_TimBaseStruct);

    TIM_OCInitTypeDef TIM_OCIintStruct;
    TIM_OCStructInit(&TIM_OCIintStruct);

    TIM_OCIintStruct.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCIintStruct.TIM_OCNPolarity = TIM_OCPolarity_High;
    TIM_OCIintStruct.TIM_OutputNState = TIM_OutputState_Enable;
    TIM_OCIintStruct.TIM_Pulse = 0;

    TIM_OC1Init(TIM1,&TIM_OCIintStruct);
    TIM_OC2Init(TIM1,&TIM_OCIintStruct);
    TIM_OC3Init(TIM1,&TIM_OCIintStruct);
    TIM_OC4Init(TIM1,&TIM_OCIintStruct);

    TIM_Cmd(TIM1,ENABLE);

}

