#include "Applicant.h"
#include "MPU6050.h"
#include "SI24R1.h"

int main()
{
	Init_All_Uint();
	MPU6050_Init();
	
	GPIO_Init_All();
	SI24R1_Init();	
	uint8_t data = 0;
	while(1)
	{
		data++;
		SI24R1_SendPayload(&data, 1);
		printf("Sent: %d\n", data);
		Delay_ms(1000); // 延迟1秒
	}
}
