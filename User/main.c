#include "Applicant.h"
#include "MPU6050.h"
#include "SI24R1.h"

int main()
{
	Init_All_Uint();
	MPU6050_Init();
	while(1)
	{
		LED_Set(1,1);
		printf("MPU6050%#X \r\n",MPU6050_GetID());
	}
}
