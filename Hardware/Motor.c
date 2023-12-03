#include "stm32f10x.h"                  // Device header
#include "Motor.h"
#include "PWM.h"

void Motor_Init(){
	PWM_Init();
	
	/*RCC*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	/*GPIO*/
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_4|GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
}

void Motor_FL_SetSpeed(int8_t Speed){
	if(Speed>=0){//正转
		GPIO_SetBits(GPIOA,GPIO_Pin_1);
		GPIO_ResetBits(GPIOA,GPIO_Pin_0);
		PWM_SetCompare1(Speed);
	}else{//反转
		GPIO_SetBits(GPIOA,GPIO_Pin_0);
		GPIO_ResetBits(GPIOA,GPIO_Pin_1);
		PWM_SetCompare1(-Speed);
	}
}

void Motor_FR_SetSpeed(int8_t Speed){
	if(Speed>=0){//正转
		GPIO_SetBits(GPIOA,GPIO_Pin_4);
		GPIO_ResetBits(GPIOA,GPIO_Pin_5);
		PWM_SetCompare2(Speed);
	}else{//反转
		GPIO_SetBits(GPIOA,GPIO_Pin_5);
		GPIO_ResetBits(GPIOA,GPIO_Pin_4);
		PWM_SetCompare2(-Speed);
	}
}

void Motor_BL_SetSpeed(int8_t Speed){
	if(Speed>=0){//正转
		GPIO_SetBits(GPIOB,GPIO_Pin_11);
		GPIO_ResetBits(GPIOB,GPIO_Pin_10);
		PWM_SetCompare3(Speed);
	}else{//反转
		GPIO_SetBits(GPIOB,GPIO_Pin_10);
		GPIO_ResetBits(GPIOB,GPIO_Pin_11);
		PWM_SetCompare3(-Speed);
	}
}

void Motor_BR_SetSpeed(int8_t Speed){
	if(Speed>=0){//正转
		GPIO_SetBits(GPIOB,GPIO_Pin_12);
		GPIO_ResetBits(GPIOB,GPIO_Pin_13);
		PWM_SetCompare4(Speed);
	}else{//反转
		GPIO_SetBits(GPIOB,GPIO_Pin_13);
		GPIO_ResetBits(GPIOB,GPIO_Pin_12);
		PWM_SetCompare4(-Speed);
	}
}

void Motor_Stop(){
	GPIO_ResetBits(GPIOA,GPIO_Pin_0);
	GPIO_ResetBits(GPIOA,GPIO_Pin_1);
	GPIO_ResetBits(GPIOA,GPIO_Pin_4);
	GPIO_ResetBits(GPIOA,GPIO_Pin_5);
	
	GPIO_ResetBits(GPIOB,GPIO_Pin_10);
	GPIO_ResetBits(GPIOB,GPIO_Pin_11);
	GPIO_ResetBits(GPIOB,GPIO_Pin_12);
	GPIO_ResetBits(GPIOB,GPIO_Pin_13);
	
	PWM_SetCompare1(0);
	PWM_SetCompare2(0);
	PWM_SetCompare3(0);
	PWM_SetCompare4(0);
}
