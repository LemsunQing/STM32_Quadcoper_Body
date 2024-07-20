#include "stm32f10x.h"                  // Device header
#include "MPU6050_Reg.h"
#include "MPU6050.h"
#include "Delay.h"
#include "Interf.h"

#define MPU6050Addr		0xD0		//MPU6050的I2C从机地址

/******************************************************************************
* 函  数：void IIC_GPIO_Init(void)
* 功　能：模拟IIC引脚初始化
* 参  数：无
* 返回值：无
* 备  注：PB6->SCL	PB7->SDA
*******************************************************************************/
void IIC_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;   
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);   
	
	GPIO_InitStruct.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7; 
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_Out_PP; 
	GPIO_InitStruct.GPIO_Speed=GPIO_Speed_50MHz;  
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_6|GPIO_Pin_7);
}

/******************************************************************************
* 函  数：void SDA_OUT(void)
* 功　能：配置模拟IIC SDA引脚为输出
* 参  数：无
* 返回值：无
* 备  注：无
*******************************************************************************/
void SDA_OUT(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/******************************************************************************
* 函  数：void SDA_IN(void)
* 功　能：配置模拟IIC SDA引脚为输入
* 参  数：无
* 返回值：无
* 备  注：无
*******************************************************************************/
void SDA_IN(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/******************************************************************************
* 函  数：void IIC_Start(void)
* 功　能：产生IIC起始信号
* 参  数：无
* 返回值：无
* 备  注：无
*******************************************************************************/	
void IIC_Start(void)
{
	SDA_OUT(); //SDA线输出 
	SDA_H;
	SCL_H;	
	Delay_us(4);
 	SDA_L;
	Delay_us(4);
	SCL_L;
}

/******************************************************************************
* 函  数：void IIC_Stop(void)
* 功　能：产生IIC停止信号
* 参  数：无
* 返回值：无
* 备  注：无
*******************************************************************************/	  
void IIC_Stop(void)
{
	SDA_OUT(); //SDA线输出
	SCL_L;
	SDA_L;
	Delay_us(4);
	SCL_H; 
	SDA_H;
	Delay_us(4);							   	
}

/******************************************************************************
* 函  数: uint8_t IIC_WaitAck(void)
* 功　能: 等待应答信号到来 （有效应答：从机第9个 SCL=0 时 SDA 被从机拉低,
*         并且 SCL = 1时 SDA依然为低）
* 参  数：无
* 返回值：1，接收应答失败	0，接收应答成功
* 备  注：从机给主机的应答
*******************************************************************************/
uint8_t IIC_WaitAck(void)
{
	uint8_t ucErrTime=0;
	SDA_IN(); //SDA设置为输入  （从机给一个低电平做为应答） 
	SDA_H;
	Delay_us(1);	   
	SCL_H;
	Delay_us(1);;	 
	while(SDA_read)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	SCL_L; //时钟输出低	   
	return 0;  
} 

/******************************************************************************
* 函  数: void IIC_Ack(void)
* 功　能: 产生ACK应答 （主机接收完一个字节数据后，主机产生的ACK通知从机一个
*         字节数据已正确接收）
* 参  数：无
* 返回值：无
* 备  注：主机给从机的应答
*******************************************************************************/
void IIC_Ack(void)
{
	SCL_L;
	SDA_OUT();
	SDA_L;
	Delay_us(1);
	SCL_H;
	Delay_us(2);
	SCL_L;
}

/******************************************************************************
* 函  数: void IIC_NAck(void)
* 功　能: 产生NACK应答 （主机接收完最后一个字节数据后，主机产生的NACK通知从机
*         发送结束，释放SDA,以便主机产生停止信号）
* 参  数：无
* 返回值：无
* 备  注：主机给从机的应答
*******************************************************************************/
void IIC_NAck(void)
{
	SCL_L;
	SDA_OUT();
	SDA_H;
	Delay_us(1);
	SCL_H;
	Delay_us(1);
	SCL_L;
}					 				     

/******************************************************************************
* 函  数：void IIC_SendByte(uint8_t data)
* 功  能：IIC发送一个字节
* 参  数：data 要写的数据
* 返回值：无
* 备  注：主机往从机发
*******************************************************************************/		  
void IIC_SendByte(uint8_t data)
{                        
    uint8_t t;   
	SDA_OUT(); 	    
    SCL_L; //拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {
		if(data&0x80)
			SDA_H;
		else
			SDA_L;
		data <<= 1;
		Delay_us(1);			
		SCL_H;
		Delay_us(1);
		SCL_L;	
		Delay_us(1);
    }	 
} 	 
   
/******************************************************************************
* 函  数：uint8_t IIC_ReadByte(uint8_t ack)
* 功  能：IIC读取一个字节
* 参  数：ack=1 时，主机数据还没接收完 ack=0 时主机数据已全部接收完成
* 返回值：无
* 备  注：从机往主机发
*******************************************************************************/	
uint8_t IIC_ReadByte(uint8_t ack)
{
	uint8_t i,receive=0;
	SDA_IN(); //SDA设置为输入模式 等待接收从机返回数据
	for(i=0;i<8;i++ )
	{
		SCL_L; 
		Delay_us(1);
		SCL_H;
		receive<<=1;
		if(SDA_read)receive++; //从机发送的电平
			Delay_us(1); 
	}					 
    if(ack)
        IIC_Ack(); //发送ACK 
    else
        IIC_NAck(); //发送nACK  
    return receive;
}

/******************************************************************************
* 函  数：uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t addr)
* 功　能：读取指定设备 指定寄存器的一个值
* 参  数：I2C_Addr  目标设备地址
		  reg	    寄存器地址
          *buf      读取数据要存储的地址    
* 返回值：返回 1失败 0成功
* 备  注：无
*******************************************************************************/ 
uint8_t IIC_ReadByteFromSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t *buf)
{
	IIC_Start();	
	IIC_SendByte(I2C_Addr);	 //发送从机地址
	if(IIC_WaitAck()) //如果从机未应答则数据发送失败
	{
		IIC_Stop();
		return 1;
	}
	IIC_SendByte(reg); //发送寄存器地址
	IIC_WaitAck();	  
	
	IIC_Start();
	IIC_SendByte(I2C_Addr+1); //进入接收模式			   
	IIC_WaitAck();
	*buf=IIC_ReadByte(0);	   
	IIC_Stop(); //产生一个停止条件
	return 0;
}

/*************************************************************************************
* 函  数：uint8_t IIC_WriteByteFromSlave(uint8_t I2C_Addr,uint8_t addr，uint8_t buf))
* 功　能：写入指定设备 指定寄存器的一个值
* 参  数：I2C_Addr  目标设备地址
		  reg	    寄存器地址
          buf       要写入的数据
* 返回值：1 失败 0成功
* 备  注：无
**************************************************************************************/ 
uint8_t IIC_WriteByteToSlave(uint8_t I2C_Addr,uint8_t reg,uint8_t data)
{
	IIC_Start();
	IIC_SendByte(I2C_Addr); //发送从机地址
	if(IIC_WaitAck())
	{
		IIC_Stop();
		return 1; //从机地址写入失败
	}
	IIC_SendByte(reg); //发送寄存器地址
	IIC_WaitAck();	  
	IIC_SendByte(data); 
	if(IIC_WaitAck())
	{
		IIC_Stop(); 
		return 1; //数据写入失败
	}
	IIC_Stop(); //产生一个停止条件
	return 0;
}

/***************************************************************************************
* 函  数：uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
* 功　能：读取指定设备 指定寄存器的 length个值
* 参  数：dev     目标设备地址
		  reg	  寄存器地址
          length  要读的字节数
		  *data   读出的数据将要存放的指针
* 返回值：1成功 0失败
* 备  注：无
***************************************************************************************/ 
uint8_t IIC_ReadMultByteFromSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
	uint8_t count = 0;
	uint8_t temp;
	IIC_Start();
	IIC_SendByte(dev); //发送从机地址
	if(IIC_WaitAck())
	{
		IIC_Stop(); 
		return 1; //从机地址写入失败
	}
	IIC_SendByte(reg); //发送寄存器地址
	IIC_WaitAck();	  
	IIC_Start();
	IIC_SendByte(dev+1); //进入接收模式	
	IIC_WaitAck();
	for(count=0;count<length;count++)
	{
		if(count!=(length-1))
			temp = IIC_ReadByte(1); //带ACK的读数据
		else  
			temp = IIC_ReadByte(0); //最后一个字节NACK

		data[count] = temp;
	}
    IIC_Stop(); //产生一个停止条件
	return 0;
}

/****************************************************************************************
* 函  数：uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
* 功　能：将多个字节写入指定设备 指定寄存器
* 参  数：dev     目标设备地址
		  reg	  寄存器地址
		  length  要写的字节数
		  *data   要写入的数据将要存放的指针
* 返回值：1成功 0失败
* 备  注：无
****************************************************************************************/ 
uint8_t IIC_WriteMultByteToSlave(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data)
{
 	uint8_t count = 0;
	IIC_Start();
	IIC_SendByte(dev); //发送从机地址
	if(IIC_WaitAck())
	{
		IIC_Stop();
		return 1; //从机地址写入失败
	}
	IIC_SendByte(reg); //发送寄存器地址
	IIC_WaitAck();	  
	for(count=0;count<length;count++)
	{
		IIC_SendByte(data[count]); 
		if(IIC_WaitAck()) //每一个字节都要等从机应答
		{
			IIC_Stop();
			return 1; //数据写入失败
		}
	}
	IIC_Stop(); //产生一个停止条件
	return 0;
}


//static uint8_t    MPU6050_buff[14];                  //加速度 陀螺仪 温度 原始数据
//INT16_XYZ	 GYRO_OFFSET_RAW,ACC_OFFSET_RAW;		 //零漂数据
//INT16_XYZ	 MPU6050_ACC_RAW,MPU6050_GYRO_RAW;	     //读取值原始数据
//uint8_t    	 SENSER_OFFSET_FLAG;                     //传感器校准标志位

/*****************************************************************************
* 函  数：uint8_t MPU6050_WriteByte(uint8_t addr,uint8_t reg,uint8_t data)
* 功  能：写一个字节数据到 MPU6050 寄存器
* 参  数：reg： 寄存器地址
*         data: 要写入的数据
* 返回值：0成功 1失败
* 备  注：MPU6050代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t MPU6050_WriteByte(uint8_t reg,uint8_t data)
{
	if(IIC_WriteByteToSlave(MPU6050Addr,reg,data))
		return 1;
	else
		return 0;
}

/*****************************************************************************
* 函  数：uint8_t MPU6050_ReadByte(uint8_t reg,uint8_t *buf)
* 功  能：从指定MPU6050寄存器读取一个字节数据
* 参  数：reg： 寄存器地址
*         buf:  读取数据存放的地址
* 返回值：1失败 0成功
* 备  注：MPU6050代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t MPU6050_ReadByte(uint8_t reg,uint8_t *buf)
{
	if(IIC_ReadByteFromSlave(MPU6050Addr,reg,buf))
		return 1;
	else
		return 0;
}

/*****************************************************************************
* 函  数：uint8_t MPU6050_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
* 功  能：从指定寄存器写入指定长度数据
* 参  数：reg：寄存器地址
*         len：写入数据长度 
*         buf: 写入数据存放的地址
* 返回值：0成功 1失败
* 备  注：MPU6050代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t MPU6050_WriteMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(IIC_WriteMultByteToSlave(MPU6050Addr,reg,len,buf))
		return 1;
	else
		return 0;
}

/*****************************************************************************
* 函  数：uint8_t MPU6050_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
* 功  能：从指定寄存器读取指定长度数据
* 参  数：reg：寄存器地址
*         len：读取数据长度 
*         buf: 读取数据存放的地址
* 返回值：0成功 0失败
* 备  注：MPU6050代码移植只需把I2C驱动修改成自己的即可
*****************************************************************************/
uint8_t MPU6050_ReadMultBytes(uint8_t reg,uint8_t len,uint8_t *buf)
{
	if(IIC_ReadMultByteFromSlave(MPU6050Addr,reg,len,buf))
		return 1;
	else
		return 0;
}

/*============================以上代码移植时需要修改=========================*/

/******************************************************************************
* 函  数：uint8_tMPU6050_getDeviceID(void)
* 功  能：读取  MPU6050 WHO_AM_I 标识将返回 0x68
* 参  数：无
* 返回值：返回读取数据
* 备  注：无
*******************************************************************************/
uint8_t MPU6050_getDeviceID(void)
{
	uint8_t buf;
	MPU6050_ReadByte(MPU6050_RA_WHO_AM_I, &buf);
	return buf;
}

/******************************************************************************
* 函  数：uint8_tMPU6050_testConnection(void)
* 功  能：检测MPU6050 是否已经连接
* 参  数：无
* 返回值：1已连接 0未链接
* 备  注：无
*******************************************************************************/
uint8_t MPU6050_testConnection(void) 
{
	if(MPU6050_getDeviceID() == 0x68)  
		return 1;
	else 
		return 0;
}

/******************************************************************************
* 函  数：void MPU6050_Check()
* 功  能：检测IIC总线上的MPU6050是否存在
* 参  数：无
* 返回值：无
* 备  注：无
*******************************************************************************/
void MPU6050_Check(void) 
{ 
	while(!MPU6050_testConnection())
	{
		printf("\rMPU6050 no connect...\r\n");
	}
}

/******************************************************************************
* 函  数：void MPU6050_AccRead(int16_t *accData)
* 功  能：读取加速度的原始数据
* 参  数：*accData 原始数据的指针
* 返回值：无
* 备  注：无
*******************************************************************************/
void MPU6050_AccRead(int16_t *accData)
{
    uint8_t buf[6];
   	MPU6050_ReadMultBytes(MPU6050_RA_ACCEL_XOUT_H,6,buf);
    accData[0] = (int16_t)((buf[0] << 8) | buf[1]);
    accData[1] = (int16_t)((buf[2] << 8) | buf[3]);
    accData[2] = (int16_t)((buf[4] << 8) | buf[5]);
}

/******************************************************************************
* 函  数：void MPU6050_GyroRead(int16_t *gyroData)
* 功  能：读取陀螺仪的原始数据
* 参  数：*gyroData 原始数据的指针
* 返回值：无
* 备  注：无
*******************************************************************************/
void MPU6050_GyroRead(int16_t *gyroData)
{
    uint8_t buf[6];
	MPU6050_ReadMultBytes(MPU6050_RA_GYRO_XOUT_H, 6, buf);
    gyroData[0] = (int16_t)((buf[0] << 8) | buf[1]) ;
    gyroData[1] = (int16_t)((buf[2] << 8) | buf[3]) ;
    gyroData[2] = (int16_t)((buf[4] << 8) | buf[5]) ;
}

/******************************************************************************
* 函  数：void MPU6050_TempRead(float *tempdata)
* 功  能：温度值读取
* 参  数：*tempdata 温度数据的指针
* 返回值：无
* 备  注：无
*******************************************************************************/
void MPU6050_TempRead(float *tempdata)
{
	uint8_t buf[2];
	short data;
	MPU6050_ReadMultBytes(MPU6050_RA_TEMP_OUT_H, 2, buf);
	data = (int16_t)((buf[0] << 8) | buf[1]) ;
	*tempdata = 36.53f + ((float)data/340.0f);
}

/******************************************************************************
* 函  数：void MPU6050_Init(void)
* 功  能：初始化MPU6050进入工作状态
* 参  数：无
* 返回值：无
* 备  注：DLPF 最好设为采样频率的一半！！！
*******************************************************************************/
void MPU6050_Init(void)
{
	MPU6050_Check(); //检查MPU6050是否连接

	MPU6050_WriteByte(MPU6050_RA_PWR_MGMT_1, 0x80); //复位MPU6050
	Delay_ms(100);
	MPU6050_WriteByte(MPU6050_RA_PWR_MGMT_1, 0x01); //唤醒MPU6050，并选择陀螺仪x轴PLL为时钟源
	MPU6050_WriteByte(MPU6050_RA_INT_ENABLE, 0x00); //禁止中断
	MPU6050_WriteByte(MPU6050_RA_GYRO_CONFIG, 0x18); //陀螺仪满量程+-2000度/秒 (最低分辨率 = 2^15/2000 = 16.4LSB/度/秒 
	MPU6050_WriteByte(MPU6050_RA_ACCEL_CONFIG, 0x08); //加速度满量程+-4g   (最低分辨率 = 2^15/4g = 8196LSB/g )
	MPU6050_WriteByte(MPU6050_RA_CONFIG, MPU6050_DLPF_BW_20);//设置陀螺的输出为1kHZ,DLPF=20Hz 
	MPU6050_WriteByte(MPU6050_RA_SMPLRT_DIV, 0x00);  //采样分频 (采样频率 = 陀螺仪输出频率 / (1+DIV)，采样频率1000hz）
	MPU6050_WriteByte(MPU6050_RA_INT_PIN_CFG, 0x02); //MPU 可直接访问MPU6050辅助I2C
	 
}

///******************************************************************************
//* 函  数：void MPU6050_CalOff(void)
//* 功  能：陀螺仪加速度校准
//* 参  数：无
//* 返回值：无
//* 备  注：无
//*******************************************************************************/
//void MPU6050_CalOff(void)
//{

//	 SENSER_FLAG_SET(ACC_OFFSET);//加速度校准
//	 SENSER_FLAG_SET(GYRO_OFFSET);//陀螺仪校准
//}

///******************************************************************************
//* 函  数：void MPU6050_CalOff_Acc(void)
//* 功  能：加速度计校准
//* 参  数：无
//* 返回值：无
//* 备  注：无
//*******************************************************************************/
//void MPU6050_CalOff_Acc(void)
//{
//	 SENSER_FLAG_SET(ACC_OFFSET);//加速度校准
//}

///******************************************************************************
//* 函  数：void MPU6050_CalOff_Gyr(void)
//* 功  能：陀螺仪校准
//* 参  数：无
//* 返回值：无
//* 备  注：无
//*******************************************************************************/
//void MPU6050_CalOff_Gyr(void)
//{
//	 SENSER_FLAG_SET(GYRO_OFFSET);//陀螺仪校准
//}

///******************************************************************************
//* 函  数：void MPU6050_Read(void)
//* 功  能：读取陀螺仪加速度计的原始数据
//* 参  数：无
//* 返回值：无
//* 备  注：无
//*******************************************************************************/
//void MPU6050_Read(void)
//{
//	MPU6050_ReadMultBytes(MPU6050_RA_ACCEL_XOUT_H, 14, MPU6050_buff);// 查询法读取MPU6050的原始数据
//}

///******************************************************************************
//* 函  数：uint8_t MPU6050_OffSet(INT16_XYZ value,INT16_XYZ *offset,uint16_t sensivity)
//* 功  能：MPU6050零偏校准
//* 参  数：value： 	 MPU6050原始数据
//*         offset：	 校准后的零偏值
//*         sensivity：加速度计的灵敏度
//* 返回值：1校准完成 0校准未完成
//* 备  注：无
//*******************************************************************************/
//uint8_t MPU6050_OffSet(INT16_XYZ value,INT16_XYZ *offset,uint16_t sensivity)
//{
//	static int32_t tempgx=0,tempgy=0,tempgz=0; 
//	static uint16_t cnt_a=0;//使用static修饰的局部变量，表明次变量具有静态存储周期，也就是说该函数执行完后不释放内存
//	if(cnt_a==0)
//	{
//		value.X=0;
//		value.Y=0;
//		value.Z=0;
//		tempgx = 0;
//		tempgy = 0;
//		tempgz = 0;
//		cnt_a = 1;
//		sensivity = 0;
//		offset->X = 0;
//		offset->Y = 0;
//		offset->Z = 0;
//	}
//	tempgx += value.X;
//	tempgy += value.Y; 
//	tempgz += value.Z-sensivity ;//加速度计校准 sensivity 等于 MPU6050初始化时设置的灵敏度值（8196LSB/g）;陀螺仪校准 sensivity = 0；
//	if(cnt_a==200)               //200个数值求平均
//	{
//		offset->X=tempgx/cnt_a;
//		offset->Y=tempgy/cnt_a;
//		offset->Z=tempgz/cnt_a;
//		cnt_a = 0;
//		return 1;
//	}
//	cnt_a++;
//	return 0;
//}	

///******************************************************************************
//* 函  数：void MPU6050_DataProcess(void)
//* 功  能：对MPU6050进行去零偏处理
//* 参  数：无
//* 返回值：无
//* 备  注：无
//*******************************************************************************/
//void MPU6050_Offset(void)
//{
//	//加速度去零偏AD值 
//	MPU6050_ACC_RAW.X =((((int16_t)MPU6050_buff[0]) << 8) | MPU6050_buff[1]) - ACC_OFFSET_RAW.X;
//	MPU6050_ACC_RAW.Y =((((int16_t)MPU6050_buff[2]) << 8) | MPU6050_buff[3]) - ACC_OFFSET_RAW.Y;
//	MPU6050_ACC_RAW.Z =((((int16_t)MPU6050_buff[4]) << 8) | MPU6050_buff[5]) - ACC_OFFSET_RAW.Z;
//	//陀螺仪去零偏AD值 
//	MPU6050_GYRO_RAW.X =((((int16_t)MPU6050_buff[8]) << 8) | MPU6050_buff[9]) - GYRO_OFFSET_RAW.X;
//	MPU6050_GYRO_RAW.Y =((((int16_t)MPU6050_buff[10]) << 8) | MPU6050_buff[11]) - GYRO_OFFSET_RAW.Y;
//	MPU6050_GYRO_RAW.Z =((((int16_t)MPU6050_buff[12]) << 8) | MPU6050_buff[13]) - GYRO_OFFSET_RAW.Z;
//	
//	if(GET_FLAG(GYRO_OFFSET)) //陀螺仪进行零偏校准
//	{
//		if(MPU6050_OffSet(MPU6050_GYRO_RAW,&GYRO_OFFSET_RAW,0))
//		{
//			
//			 SENSER_FLAG_RESET(GYRO_OFFSET);
//			 PID_WriteFlash(); //保存陀螺仪的零偏数据
//			 GYRO_Offset_LED();
//		     SENSER_FLAG_SET(ACC_OFFSET);//校准加速度
//			
////			 printf("GYRO_OFFSET_RAW Value :X=%d  Y=%d  Z=%d\n",GYRO_OFFSET_RAW.X,GYRO_OFFSET_RAW.Y,GYRO_OFFSET_RAW.Z);
////			 printf("\n");
//		}
//	}
//	if(GET_FLAG(ACC_OFFSET)) //加速度计进行零偏校准 
//	{
//		if(MPU6050_OffSet(MPU6050_ACC_RAW,&ACC_OFFSET_RAW,8196))
//		{
//			 SENSER_FLAG_RESET(ACC_OFFSET);
//			 PID_WriteFlash(); //保存加速度计的零偏数据
//			 ACC_Offset_LED();
////			 printf("ACC_OFFSET_RAW Value X=%d  Y=%d  Z=%d\n",ACC_OFFSET_RAW.X,ACC_OFFSET_RAW.Y,ACC_OFFSET_RAW.Z); 
////			 printf("\n");
//		}
//	}
//}
