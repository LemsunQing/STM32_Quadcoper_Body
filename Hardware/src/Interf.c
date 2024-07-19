#include "Interf.h"


// 初始化USART1
void USART1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;

    // 使能USART1和GPIOA的时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_GPIOA, ENABLE);

    // 配置PA9为USART1_TX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置PA10为USART1_RX
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    // 配置USART1参数
    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStructure);

    // 使能USART1
    USART_Cmd(USART1, ENABLE);
}

// 通过USART1发送单个字符
void USART1_SendChar(char c)
{
    // 等待USART1的数据寄存器为空
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
    // 发送字符
    USART_SendData(USART1, c);
}

// 通过USART1发送字符串
void USART1_SendString(char* str)
{
    while (*str)
    {
        USART1_SendChar(*str++);
    }
}

// 通过USART1接收单个字符
char USART1_ReceiveChar(void)
{
    // 等待接收寄存器不为空
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
    // 读取接收到的字符
    return (char)USART_ReceiveData(USART1);
}
// fputc函数用于printf重定向
int fputc(int ch, FILE *f)
{
    USART1_SendChar((char)ch);
    return ch;
}

// fgetc函数用于scanf重定向
int fgetc(FILE *f)
{
    return USART1_ReceiveChar();
}


