#include "stm32f10x.h"                  // Device header
#include "LED.h"
#include "Interf.h"
#include "Delay.h"
#include "MPU6050.h"
#include "SpeedControl.h"

int main(void)
{
	uint8_t ID_Infor;
	int16_t AX, AY, AZ, GX, GY, GZ;			//定义用于存放各个数据的变量

	LED_GPIO_Init();
	USART1_Init();
	MPU6050_Init();
	Wing_GPIO_Init();

	while (1)
	{
		LED_Set(1,1);
		TIM_SetCompare1(TIM1,20);
		TIM_SetCompare2(TIM1,20);
		TIM_SetCompare3(TIM1,20);
		TIM_SetCompare4(TIM1,20);

	}
}
