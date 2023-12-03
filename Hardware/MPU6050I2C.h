#ifndef __MPU6050I2C_H__
#define __MPU6050I2C_H__

void MPU6050I2C_Init();
void MPU6050I2C_Start();
void MPU6050I2C_Stop();
void MPU6050I2C_SendByte(uint8_t Byte);
uint8_t MPU6050I2C_ReceiveByte();
void MPU6050I2C_SendACK(uint8_t ACKBit);
uint8_t MPU6050I2C_ReceiveACK();

#endif