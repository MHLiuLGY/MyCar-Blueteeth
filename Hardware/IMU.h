#ifndef __IMU_H__
#define __IMU_H__

#include "MPU6050.h"

void imuUpdate(struct MPU6050_Data Data,float *Roll,float *Pitch,float *Yaw,float dt);

#endif