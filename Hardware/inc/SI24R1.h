#ifndef __SI24R1_H
#define __SI24R1_H

#include "stm32f10x.h"                  // Device header
#include "Interf.h"
#include "Delay.h"

#define RF_CSN_PIN   GPIO_Pin_12
#define RF_SCK_PIN   GPIO_Pin_13
#define RF_MISO_PIN  GPIO_Pin_14
#define RF_MOSI_PIN  GPIO_Pin_15
#define RF_CE_PIN    GPIO_Pin_8
#define RF_IRQ_PIN   GPIO_Pin_2

#define RF_GPIO_PORT GPIOB
#define RF_CE_PORT   GPIOA

void GPIO_Init_All(void);
void SPI_GPIO_Init(void);
uint8_t SPI_Transfer(uint8_t data);
void SI24R1_Init(void);
uint8_t SI24R1_ReadReg(uint8_t reg);
void SI24R1_WriteReg(uint8_t reg, uint8_t value);
void SI24R1_SendPayload(uint8_t *data, uint8_t length);


void SI24R1_Init(void);

#endif
