#include "stm32f10x.h"                  // Device header
#include "MPU6050I2C.h"
#include "MPU6050.h"
#include "MPU6050_Reg.h"

#define MPU6050_ADDRESS 0xD0

void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data){//指定地址写
	MPU6050I2C_Start();
	MPU6050I2C_SendByte(MPU6050_ADDRESS);
	MPU6050I2C_ReceiveACK();//此处可以判断一下从机是否应答
	MPU6050I2C_SendByte(RegAddress);
	MPU6050I2C_ReceiveACK();//此处可以判断一下从机是否应答
	
	//此处可以加for循环
	MPU6050I2C_SendByte(Data);
	MPU6050I2C_ReceiveACK();//此处可以判断一下从机是否应答
	
	MPU6050I2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress){//指定地址读
	uint8_t Data;
	
	MPU6050I2C_Start();
	MPU6050I2C_SendByte(MPU6050_ADDRESS);
	MPU6050I2C_ReceiveACK();//此处可以判断一下从机是否应答
	MPU6050I2C_SendByte(RegAddress);
	MPU6050I2C_ReceiveACK();//此处可以判断一下从机是否应答
	
	MPU6050I2C_Start();//重复起始
	MPU6050I2C_SendByte(MPU6050_ADDRESS|0x01);
	MPU6050I2C_ReceiveACK();//此处可以判断一下从机是否应答
	
	//此处可以加for循环，循环末尾MPU6050I2C_SendACK(0);表示还要继续读数据，最后再MPU6050I2C_SendACK(1);
	Data=MPU6050I2C_ReceiveByte();
	MPU6050I2C_SendACK(1);
	
	MPU6050I2C_Stop();
	
	return Data;
}

void MPU6050_Init(){
	MPU6050I2C_Init();
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x01);//0000 0001
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0x00);//0000 0000
	
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV,0x09);//10分频
	MPU6050_WriteReg(MPU6050_CONFIG,0x06);//00 000 110
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG,0x18);//000 11 000 最大量程不自测
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG,0x00);//000 00 000 最小量程2g不自测
}

uint8_t MPU6050_GetID(){
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

void MPU6050_GetData(struct MPU6050_Data *Data){
	//直接<<8不会有问题，自动类型转换
	Data->AccX=MPU6050_ReadReg(MPU6050_ACCEL_XOUT_H)<<8 | MPU6050_ReadReg(MPU6050_ACCEL_XOUT_L);
	Data->AccY=MPU6050_ReadReg(MPU6050_ACCEL_YOUT_H)<<8 | MPU6050_ReadReg(MPU6050_ACCEL_YOUT_L);
	Data->AccZ=MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_H)<<8 | MPU6050_ReadReg(MPU6050_ACCEL_ZOUT_L);
	Data->GyroX=MPU6050_ReadReg(MPU6050_GYRO_XOUT_H)<<8 | MPU6050_ReadReg(MPU6050_GYRO_XOUT_L);
	Data->GyroY=MPU6050_ReadReg(MPU6050_GYRO_YOUT_H)<<8 | MPU6050_ReadReg(MPU6050_GYRO_YOUT_L);
	Data->GyroZ=MPU6050_ReadReg(MPU6050_GYRO_ZOUT_H)<<8 | MPU6050_ReadReg(MPU6050_GYRO_ZOUT_L);
}//可以尝试连续读多个字节
