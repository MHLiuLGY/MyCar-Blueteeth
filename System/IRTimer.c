#include "stm32f10x.h"                  // Device header
#include "IRTimer.h"

void IRTimer_Init(){
	/*RCC TIM2*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);
	
	/*时钟源*/
	TIM_InternalClockConfig(TIM2);
	
	/*时基单元*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//滤波采样频率
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStructure.TIM_Period=65536-1;//自动重装器寄存器，即ARR，0-65535
	TIM_TimeBaseInitStructure.TIM_Prescaler=72-1;//预分频器寄存器，即PSC，0-65535
	//CK_CNT_OV=CK_PSC/(PSC+1)/(ARR+1)    CK_PSC=72MHz    T=1/CK_CNT_OV
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//仅限高级计数器
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//初始化中断标志位置位
	TIM_ClearFlag(TIM2,TIM_FLAG_Update);//在开启中断前，手动清除中断标志位
	
	/*使能定时器*/
	TIM_Cmd(TIM2,DISABLE);
}

uint16_t IRTimer_GetCounter(){
	return TIM_GetCounter(TIM2);
}
