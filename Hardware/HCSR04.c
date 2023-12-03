#include "stm32f10x.h"                  // Device header
#include "HCSR04.h"
#include "Delay.h"
#include "HCSR04Timer.h"
#include "KLM.h"

uint8_t HCSR04_Wait;

void HCSR04_W_Trig1(uint8_t BitValue){
	GPIO_WriteBit(GPIOB,GPIO_Pin_8,(BitAction)BitValue);
}

uint8_t HCSR04_R_Echo1(){
	uint8_t BitValue;
	BitValue=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_9);
	return BitValue;
}

void HCSR04_W_Trig2(uint8_t BitValue){
	GPIO_WriteBit(GPIOB,GPIO_Pin_6,(BitAction)BitValue);
}

uint8_t HCSR04_R_Echo2(){
	uint8_t BitValue;
	BitValue=GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_7);
	return BitValue;
}

void HCSR04_Init(){
	HCSR04Timer_Init();
	
	/*RCC*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);
	
	/*GPIO*/
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_8|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	HCSR04_W_Trig1(0);
	HCSR04_W_Trig2(0);
}

uint16_t HCSR04_GetDistance1(){
	uint16_t Distance,Time;
	
	HCSR04_W_Trig1(1);
	Delay_us(20);
	HCSR04_W_Trig1(0);
	
	HCSR04Timer_Cmd(ENABLE);
	
	while(HCSR04_R_Echo1()==0&&!HCSR04_Wait);//防止Echo初始状态恒为0导致程序卡死，用delay写wait函数也可以
	if(HCSR04_Wait==1){
		HCSR04_Wait=0;
		return 0;
	}
	
	TIM_SetCounter(TIM4,0);//使用封装函数void Timer_SetCounter(uint16_t Counter);不行？？
	
	while(HCSR04_R_Echo1()==1);
	
	if(Time<35000){//38ms表示超时信号
		Time=HCSR04Timer_GetCounter();
		Distance=(Time*170)/100;//单位:100cm
		//Distance=KLM(Distance);
	}
	else{
		Distance=65535;//超出范围
	}
	
	HCSR04Timer_Cmd(DISABLE);
	
	return Distance;
}

uint16_t HCSR04_GetDistance2(){
	uint16_t Distance,Time;
	
	HCSR04_W_Trig2(1);
	Delay_us(20);
	HCSR04_W_Trig2(0);
	
	HCSR04Timer_Cmd(ENABLE);
	
	while(HCSR04_R_Echo2()==0&&!HCSR04_Wait);//防止Echo初始状态恒为0导致程序卡死，用delay写wait函数也可以
	if(HCSR04_Wait==1){
		HCSR04_Wait=0;
		return 0;
	}
	
	TIM_SetCounter(TIM4,0);//使用封装函数void Timer_SetCounter(uint16_t Counter);不行？？
	
	while(HCSR04_R_Echo2()==1);
	
	if(Time<35000){//38ms表示超时信号
		Time=HCSR04Timer_GetCounter();
		Distance=(Time*170)/100;//单位:100cm
		//Distance=KLM(Distance);
	}
	else{
		Distance=65535;//超出范围
	}
	
	HCSR04Timer_Cmd(DISABLE);
	
	return Distance;
}

void TIM4_IRQHandler(){
	if(TIM_GetITStatus(TIM4,TIM_IT_Update)==SET){
		HCSR04_Wait=1;
		
		TIM_ClearITPendingBit(TIM4,TIM_IT_Update);
	}
}
