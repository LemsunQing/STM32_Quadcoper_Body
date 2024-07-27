#ifndef __SI24R1_H
#define __SI24R1_H

#include "stm32f10x.h"                  // Device header

// PB2 boot1 RF_IRQ
// PA8 RF_CE
// PB15 RF_MOSI
// PB14 RF_MISO
// PB13 RF_SCK
// PB12 RF_CSN

#define GPIO_Pin_15 SI_MOSI_Pin
#define GPIO_Pin_14 SI_MISO_Pin
#define GPIO_Pin_13 SI_SCK_Pin
#define GPIO_Pin_12 SI_CSN_Pin



void SI24R1_Init(void);

#endif
