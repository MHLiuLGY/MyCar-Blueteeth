#include "stm32f10x.h"                  // Device header
#include "HC05.h"
#include "Serial.h"
#include <string.h>

uint8_t HC05_RxFlag;

char HC05_RxStrPacket[100];

void HC05_Init()
{
    Serial_Init();

//    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

//    GPIO_InitTypeDef GPIO_InitStructure;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

//void HC05_EnterAT()
//{
//    GPIO_SetBits(GPIOA, GPIO_Pin_0);
//}

//void HC05_ExitAT()
//{
//    GPIO_ResetBits(GPIOA, GPIO_Pin_0);
//}

void HC05_SendByte(uint8_t Byte){//·¢ËÍ×Ö½Ú
	Serial_SendByte(Byte);
}

void HC05_SendString(char *Buf){
    Serial_Printf(Buf);
}

void HC05_SendNumber(uint32_t Number, uint8_t Length){
    Serial_SendNumber(Number, Length);
}

void HC05_GetStrPacket(){
    if (Serial_RxFlag == 1){
        strncpy(HC05_RxStrPacket, Serial_RxStrPacket, sizeof(HC05_RxStrPacket)-1);
        HC05_RxStrPacket[sizeof(HC05_RxStrPacket)-1] = '\0';
        Serial_RxFlag = 0;
        HC05_RxFlag = 1;
        
        HC05_SendString(HC05_RxStrPacket);
        HC05_SendString("\n");
    }
}
