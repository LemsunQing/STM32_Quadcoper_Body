#ifndef __SPEEDCONTROL_H
#define __SPEEDCONTROL_H

#define Wing_Pin_LUA GPIO_Pin_6 // Define correct pins
#define Wing_Pin_RUA GPIO_Pin_7
#define Wing_Pin_LDA GPIO_Pin_0
#define Wing_Pin_RDA GPIO_Pin_1

void MOTOR_Init(void);
void Moto_Pwm(int16_t MOTO1_PWM,int16_t MOTO2_PWM,int16_t MOTO3_PWM,int16_t MOTO4_PWM);

#endif
