#include "stm32f10x.h"                  // Device header
#include "IR.h"
#include "IRTimer.h"

uint16_t IR_Time;//存储计时（微秒）
uint8_t IR_State;//存储当前状态

uint8_t IR_Data[4];//存四个字节
uint8_t IR_pData;//当前存到第几位（共32位）

uint8_t IR_DataFlag;//置1表示收到数据，一帧结束
uint8_t IR_RepeatFlag;//置1表示重发数据，一帧结束

uint8_t IR_Address;
uint8_t IR_Command;

void IR_Init(){
	IRTimer_Init();
	
	/*RCC*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	/*GPIO*/
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	/*AFIO*/
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOA,GPIO_PinSource2);
	
	/*EXTI*/
	EXTI_InitTypeDef EXTI_InitStructure;
	
	EXTI_InitStructure.EXTI_Line=EXTI_Line2;
	EXTI_InitStructure.EXTI_LineCmd=ENABLE;
	EXTI_InitStructure.EXTI_Mode=EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger=EXTI_Trigger_Falling;
	EXTI_Init(&EXTI_InitStructure);
	
	/*NVIC*/
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//分组在一个工程中只有一次
	
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel=EXTI2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);
}

uint8_t IR_GetDataFlag(){
	if(IR_DataFlag){
		IR_DataFlag=0;
		return 1;
	}
	return 0;
}

uint8_t IR_GetRepeatFlag(){
	if(IR_RepeatFlag){
		IR_RepeatFlag=0;
		return 1;
	}
	return 0;
}

uint8_t IR_GetAddress(){
	return IR_Address;
}

uint8_t IR_GetCommand(){
	return IR_Command;
}

void EXTI2_IRQHandler(){
	if(EXTI_GetITStatus(EXTI_Line2)==SET){
		EXTI_ClearITPendingBit(EXTI_Line2);
		switch(IR_State){
			case 0:{//第1个下降沿，新的一帧，进入状态1
				TIM_SetCounter(TIM2,0);
				TIM_Cmd(TIM2,ENABLE);
				IR_State=1;
				break;
			}
			case 1:{//第2个下降沿，判断开始或重复。若判断为开始，进入状态2；若判断为重复，该帧结束，回到状态0
				IR_Time=IRTimer_GetCounter();
				TIM_SetCounter(TIM2,0);//不关闭计时器，且开始新的计时
				if(IR_Time>13500-500 && IR_Time<13500+500){
					IR_State=2;
				}else if(IR_Time>11250-500 && IR_Time<11250+500){
					IR_RepeatFlag=1;
					TIM_Cmd(TIM2,DISABLE);//也可以不停计时器，在状态0会清零重启
					IR_State=0;
				}else{
					IR_State=1;
				}
				break;
			}
			case 2:{//第3个至n个下降沿，判断高低电平。若判断最后一位存储完毕，该帧结束，回到状态0
				IR_Time=IRTimer_GetCounter();
				TIM_SetCounter(TIM2,0);
				if(IR_Time>1120-500 && IR_Time<1120+500){
					IR_Data[IR_pData/8]&=~(0x01<<(IR_pData%8));//左移补零故只能用0x01，清零需要0xFE故取反
					IR_pData++;
				}else if(IR_Time>2250-500 && IR_Time<2250+500){
					IR_Data[IR_pData/8]|=(0x01<<(IR_pData%8));//左移补零故只能用0x01，置1不需要取反
					IR_pData++;
				}else{
					IR_pData=0;
					IR_State=1;
				}
				if(IR_pData>=32){
					IR_pData=0;
					if((IR_Data[0]==(uint8_t)(~IR_Data[1]))&&(IR_Data[2]==(uint8_t)(~IR_Data[3]))){//强制转换为(uint8_t)，详见C中的按位取反的逻辑
						IR_Address=IR_Data[0];
						IR_Command=IR_Data[2];
						IR_DataFlag=1;
					}
					TIM_Cmd(TIM2,DISABLE);
					IR_State=0;
				}
				break;
			}
		}
	}
}
