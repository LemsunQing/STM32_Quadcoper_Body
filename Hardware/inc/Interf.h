#ifndef __INTERF_H
#define __INTERF_H

#include "stm32f10x.h"
#include "stdio.h"

void USART1_Init(void);
void USART1_SendChar(char c);
void USART1_SendString(char* str);
char USART1_ReceiveChar(void);
int fputc(int ch, FILE *f);
int fgetc(FILE *f);

#endif
