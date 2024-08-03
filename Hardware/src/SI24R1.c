#include "SI24R1.h"

void GPIO_Init_All(void)
{
    // 使能GPIOA和GPIOB时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;

    // 配置RF_CSN, RF_SCK, RF_MOSI引脚为推挽输出
    GPIO_InitStructure.GPIO_Pin = RF_CSN_PIN | RF_SCK_PIN | RF_MOSI_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(RF_GPIO_PORT, &GPIO_InitStructure);

    // 配置RF_MISO引脚为浮空输入
    GPIO_InitStructure.GPIO_Pin = RF_MISO_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(RF_GPIO_PORT, &GPIO_InitStructure);

    // 配置RF_CE引脚为推挽输出
    GPIO_InitStructure.GPIO_Pin = RF_CE_PIN;
    GPIO_Init(RF_CE_PORT, &GPIO_InitStructure);

    // 配置RF_IRQ引脚为浮空输入
    GPIO_InitStructure.GPIO_Pin = RF_IRQ_PIN;
    GPIO_Init(RF_GPIO_PORT, &GPIO_InitStructure);

    // 设置初始状态
    GPIO_SetBits(RF_GPIO_PORT, RF_CSN_PIN | RF_SCK_PIN | RF_MOSI_PIN);
    GPIO_ResetBits(RF_CE_PORT, RF_CE_PIN);
}

void SPI_GPIO_Init(void)
{
    // 设置初始状态
    GPIO_SetBits(RF_GPIO_PORT, RF_CSN_PIN);
    GPIO_ResetBits(RF_GPIO_PORT, RF_SCK_PIN);
}

uint8_t SPI_Transfer(uint8_t data)
{
    uint8_t receivedData = 0;

    for (int i = 0; i < 8; i++)
    {
        if (data & 0x80)
            GPIO_SetBits(RF_GPIO_PORT, RF_MOSI_PIN);
        else
            GPIO_ResetBits(RF_GPIO_PORT, RF_MOSI_PIN);

        data <<= 1;

        GPIO_SetBits(RF_GPIO_PORT, RF_SCK_PIN);
        Delay_us(1);

        receivedData <<= 1;
        if (GPIO_ReadInputDataBit(RF_GPIO_PORT, RF_MISO_PIN))
            receivedData |= 0x01;

        GPIO_ResetBits(RF_GPIO_PORT, RF_SCK_PIN);
        Delay_us(1);
    }

    return receivedData;
}

void SI24R1_Init(void)
{
    GPIO_ResetBits(RF_CE_PORT, RF_CE_PIN); // 禁用CE
    GPIO_ResetBits(RF_GPIO_PORT, RF_CSN_PIN); // 选中设备
    SPI_Transfer(0x20 | 0x00); // 写入配置寄存器
    SPI_Transfer(0x0E); // 配置寄存器值：使能CRC，16位CRC，使能电源，发送模式
    GPIO_SetBits(RF_GPIO_PORT, RF_CSN_PIN); // 取消选中设备

    // 设置地址宽度为5字节
    SI24R1_WriteReg(0x03, 0x03); // SETUP_AW
    // 设置发送地址
    GPIO_ResetBits(RF_GPIO_PORT, RF_CSN_PIN);
    SPI_Transfer(0x20 | 0x10); // 写入TX_ADDR寄存器
    for (int i = 0; i < 5; i++)
        SPI_Transfer(0xE7); // 假设地址为0xE7E7E7E7E7
    GPIO_SetBits(RF_GPIO_PORT, RF_CSN_PIN);

    // 设置有效数据宽度
    SI24R1_WriteReg(0x11, 1); // RX_PW_P0

    GPIO_SetBits(RF_CE_PORT, RF_CE_PIN); // 启用CE
}

uint8_t SI24R1_ReadReg(uint8_t reg)
{
    uint8_t result;
    GPIO_ResetBits(RF_GPIO_PORT, RF_CSN_PIN); // 选中设备
    SPI_Transfer(reg); // 发送寄存器地址
    result = SPI_Transfer(0xFF); // 读取寄存器值
    GPIO_SetBits(RF_GPIO_PORT, RF_CSN_PIN); // 取消选中设备
    return result;
}

void SI24R1_WriteReg(uint8_t reg, uint8_t value)
{
    GPIO_ResetBits(RF_GPIO_PORT, RF_CSN_PIN); // 选中设备
    SPI_Transfer(0x20 | reg); // 发送寄存器地址
    SPI_Transfer(value); // 写入寄存器值
    GPIO_SetBits(RF_GPIO_PORT, RF_CSN_PIN); // 取消选中设备
}

void SI24R1_SendPayload(uint8_t *data, uint8_t length)
{
    GPIO_ResetBits(RF_GPIO_PORT, RF_CSN_PIN); // 选中设备
    SPI_Transfer(0xA0); // 发送Payload命令
    for (int i = 0; i < length; i++)
        SPI_Transfer(data[i]); // 发送数据
    GPIO_SetBits(RF_GPIO_PORT, RF_CSN_PIN); // 取消选中设备
}
