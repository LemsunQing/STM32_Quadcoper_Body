#ifndef __MPU6050_H
#define __MPU6050_H

#define SCL_H         GPIOB->BSRR = GPIO_Pin_6
#define SCL_L         GPIOB->BRR  = GPIO_Pin_6
#define SDA_H         GPIOB->BSRR = GPIO_Pin_7 
#define SDA_L         GPIOB->BRR  = GPIO_Pin_7
#define SDA_read      ((GPIOB->IDR & GPIO_Pin_7)!=0)?1:0

void IIC_GPIO_Init(void);        //初始化IIC的IO口				 
void IIC_Start(void);			 //发送IIC开始信号
void IIC_Stop(void);	  	  	 //发送IIC停止信号
void IIC_Ack(void);				 //IIC发送ACK信号
void IIC_NAck(void);			 //IIC不发送ACK信号
uint8_t IIC_WaitAck(void); 		 //IIC等待ACK信号

void IIC_SendByte(uint8_t data);  //IIC发送一个字节
uint8_t IIC_ReadByte(uint8_t ack);//IIC读取一个字节

uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf);
uint8_t IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t buf);
uint8_t IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);

//设置低通滤波
#define MPU6050_DLPF_BW_256         0x00
#define MPU6050_DLPF_BW_188         0x01
#define MPU6050_DLPF_BW_98          0x02
#define MPU6050_DLPF_BW_42          0x03
#define MPU6050_DLPF_BW_20          0x04
#define MPU6050_DLPF_BW_10          0x05
#define MPU6050_DLPF_BW_5           0x06


//供外部调用的API
void MPU6050_Init(void); //初始化;
void MPU6050_Check(void);
void MPU6050_Read(void);
void MPU6050_Offset(void);
void MPU6050_CalOff(void);
void MPU6050_CalOff_Acc(void);
void MPU6050_CalOff_Gyr(void);
void MPU6050_GyroRead(int16_t *gyroData);
void MPU6050_AccRead(int16_t *accData);
void MPU6050_TempRead(float *tempdata);
uint8_t MPU6050_testConnection(void);

#endif
