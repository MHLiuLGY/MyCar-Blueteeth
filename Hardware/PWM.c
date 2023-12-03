#include "stm32f10x.h"                  // Device header
#include "PWM.h"

void PWM_Init(){
	/*RCC TIM3*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

	/*GPIO*/
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出，将引脚控制权交给片上外设
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_6|GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;//复用推挽输出，将引脚控制权交给片上外设
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	/*时钟源*/
	TIM_InternalClockConfig(TIM3);
	
	/*时基单元*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;//滤波采样频率
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up;//向上计数
	TIM_TimeBaseInitStructure.TIM_Period=100-1;//自动重装器寄存器，即ARR，0-65535
	TIM_TimeBaseInitStructure.TIM_Prescaler=72-1;//预分频器寄存器，即PSC，0-65535//针对电机，加大频率
	//CK_CNT_OV=CK_PSC/(PSC+1)/(ARR+1)    CK_PSC=72MHz    T=1/CK_CNT_OV
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter=0;//仅限高级计数器
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化中断标志位置位
	TIM_ClearFlag(TIM3,TIM_FLAG_Update);//在开启中断前，手动清除中断标志位
	
	/*输出比较单元*/
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCStructInit(&TIM_OCInitStructure);//赋初值
	
	TIM_OCInitStructure.TIM_OCMode=TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OCPolarity=TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OutputState=TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse=0;
	TIM_OC1Init(TIM3,&TIM_OCInitStructure);
	TIM_OC2Init(TIM3,&TIM_OCInitStructure);
	TIM_OC3Init(TIM3,&TIM_OCInitStructure);
	TIM_OC4Init(TIM3,&TIM_OCInitStructure);
	
	/*使能定时器*/
	TIM_Cmd(TIM3,ENABLE);
}

void PWM_SetCompare1(uint16_t Compare){
	TIM_SetCompare1(TIM3,Compare);
}

void PWM_SetCompare2(uint16_t Compare){
	TIM_SetCompare2(TIM3,Compare);
}

void PWM_SetCompare3(uint16_t Compare){
	TIM_SetCompare3(TIM3,Compare);
}

void PWM_SetCompare4(uint16_t Compare){
	TIM_SetCompare4(TIM3,Compare);
}
