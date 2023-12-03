#ifndef __HC05_H__
#define __HC05_H__

#include <stdio.h>

extern char HC05_RxStrPacket[];

extern uint8_t HC05_RxFlag;

void HC05_Init();
//void HC05_EnterAT();
//void HC05_ExitAT();
void HC05_SendByte(uint8_t Byte);
void HC05_SendString(char *Buf);
void HC05_SendNumber(uint32_t Number, uint8_t Length);
void HC05_GetStrPacket();

#endif