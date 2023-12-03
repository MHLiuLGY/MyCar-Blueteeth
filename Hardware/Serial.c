#include "stm32f10x.h"                  // Device header
#include "Serial.h"
#include <stdarg.h>

uint8_t Serial_RxFlag;

char Serial_RxStrPacket[100];

void Serial_Init(){
	/*RCC*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	
	/*GPIO*/
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	/*USART*/
	USART_InitTypeDef USART_InitStructure;
	
	USART_InitStructure.USART_BaudRate=9600;
	USART_InitStructure.USART_HardwareFlowControl=USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode=USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStructure.USART_Parity=USART_Parity_No;
	USART_InitStructure.USART_StopBits=USART_StopBits_1;
	USART_InitStructure.USART_WordLength=USART_WordLength_8b;
	USART_Init(USART1,&USART_InitStructure);
	
	/*中断控制*/
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	
	/*NVIC*/
	//NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//分组在一个工程中只有一次
	
	NVIC_InitTypeDef NVIC_InitStructure;
	
	NVIC_InitStructure.NVIC_IRQChannel=USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=1;
	NVIC_Init(&NVIC_InitStructure);
	
	/*CMD*/
	USART_Cmd(USART1,ENABLE);
}

void Serial_SendByte(uint8_t Byte){//发送字节
	USART_SendData(USART1,Byte);
	while(USART_GetFlagStatus(USART1,USART_FLAG_TXE)==RESET);//也不需要手动清零，再次senddata会清零，同ADC
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)//发送数组
{
	uint16_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *String)//发送字符串
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++)
	{
		Serial_SendByte(String[i]);
	}
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)//求幂，调用于发送数字
{
	uint32_t Result = 1;
	while (Y --)
	{
		Result *= X;
	}
	return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)//发送数字
{
	uint8_t i;
	for (i = 0; i < Length; i ++)
	{
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

int fputc(int ch, FILE *f)//printf的底层，该方法将仅有的一个printf重定向到USART1
{
	Serial_SendByte(ch);
	return ch;
}

void Serial_Printf(char *format, ...)//平替printf
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_SendString(String);
}

void USART1_IRQHandler(){//接受数据包
	static uint8_t RxState=0;
	static uint8_t pRxPacket=0;
	if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET){
		uint8_t RxData=USART_ReceiveData(USART1);
		switch (RxState){
			case 0:{//状态0
				if(RxData=='@'&&Serial_RxFlag==0){//Str
					RxState=1;
					pRxPacket=0;
				}
				break;
			}
			case 1:{//状态1
				if(RxData=='\r'){//Str
					RxState=2;
				}
				else{
					Serial_RxStrPacket[pRxPacket]=RxData;//Str
					pRxPacket++;
				}
				break;
			}
			case 2:{//状态2
				if(RxData=='\n'){//Str
					RxState=0;
					Serial_RxFlag=1;
					Serial_RxStrPacket[pRxPacket]='\0';
				}
				break;
			}
		}
		
		USART_ClearITPendingBit(USART1,USART_IT_RXNE);//读取DR会自动清除
	}
}