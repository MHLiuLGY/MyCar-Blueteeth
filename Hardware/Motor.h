#ifndef __MOTOR_H__
#define __MOTOR_H__

void Motor_Init();
void Motor_FL_SetSpeed(int8_t Speed);
void Motor_FR_SetSpeed(int8_t Speed);
void Motor_BL_SetSpeed(int8_t Speed);
void Motor_BR_SetSpeed(int8_t Speed);
void Motor_Stop();

#endif