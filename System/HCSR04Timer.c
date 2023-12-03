#include "stm32f10x.h"                  // Device header
#include "HCSR04Timer.h"

void HCSR04Timer_Init(){
	/*RCC TIM4*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);
	
	/*时钟源*/
	TIM_InternalClockConfig(TIM4);
	
	/*时基单元*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//滤波采样频率
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStructure.TIM_Period=65536-1;//自动重装器寄存器，即ARR，0-65535
	TIM_TimeBaseInitStructure.TIM_Prescaler=72-1;//预分频器寄存器，即PSC，0-65535
	//CK_CNT_OV=CK_PSC/(PSC+1)/(ARR+1)    CK_PSC=72MHz    T=1/CK_CNT_OV
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//仅限高级计数器
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseInitStructure);//初始化中断标志位置位
	TIM_ClearFlag(TIM4,TIM_FLAG_Update);//在开启中断前，手动清除中断标志位
	
	/*中断输出控制*/
	TIM_ITConfig(TIM4,TIM_EventSource_Update,ENABLE);//每次更新可以申请中断
	
	/*NVIC*/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//分组在一个工程中只有一次
	
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);
	
	/*使能定时器*/
	TIM_Cmd(TIM4,DISABLE);
}

uint16_t HCSR04Timer_GetCounter(){
	return TIM_GetCounter(TIM4);
}

void HCSR04Timer_SetCounter(uint16_t Counter){
	TIM_SetCounter(TIM4,Counter);
}

void HCSR04Timer_Cmd(FunctionalState NewState){
	TIM_Cmd(TIM4,NewState);
}
