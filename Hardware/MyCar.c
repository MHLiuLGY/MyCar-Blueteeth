#include "stm32f10x.h"                  // Device header
#include "MyCar.h"
#include "Motor.h"

void MyCar_Init(){
	Motor_Init();
}

void MyCar_GoForward(int8_t Speed){
	Motor_FL_SetSpeed(Speed);
	Motor_FR_SetSpeed(Speed);
	Motor_BL_SetSpeed(Speed);
	Motor_BR_SetSpeed(Speed);
}

void MyCar_GoBackward(int8_t Speed){
	Motor_FL_SetSpeed(-Speed);
	Motor_FR_SetSpeed(-Speed);
	Motor_BL_SetSpeed(-Speed);
	Motor_BR_SetSpeed(-Speed);
}

void MyCar_TurnLeft(int8_t Speed){
	Motor_FL_SetSpeed(-Speed);
	Motor_FR_SetSpeed(Speed);
	Motor_BL_SetSpeed(-Speed);
	Motor_BR_SetSpeed(Speed);
}

void MyCar_TurnRight(int8_t Speed){
	Motor_FL_SetSpeed(Speed);
	Motor_FR_SetSpeed(-Speed);
	Motor_BL_SetSpeed(Speed);
	Motor_BR_SetSpeed(-Speed);
}

void MyCar_Stop(){
	Motor_Stop();
}
