#ifndef __SPEEDCONTROL_H
#define __SPEEDCONTROL_H

#define Wing_Port (GPIOA | GPIOB)
#define Wing_Pin_LUA GPIO_Pin_6
#define Wing_Pin_RUA GPIO_Pin_7
#define Wing_Pin_LDB GPIO_Pin_0
#define Wing_Pin_RDB GPIO_Pin_1

void Wing_GPIO_Init(void);

#endif
