#ifndef __MPU6050_H__
#define __MPU6050_H__

struct MPU6050_Data{
	int16_t AccX;
	int16_t AccY;
	int16_t AccZ;
	int16_t GyroX;
	int16_t GyroY;
	int16_t GyroZ;
};//或者用typedef struct，这样在定义时不用struct

void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
void MPU6050_Init();
uint8_t MPU6050_GetID();
void MPU6050_GetData(struct MPU6050_Data *Data);

#endif