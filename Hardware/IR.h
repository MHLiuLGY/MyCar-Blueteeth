#ifndef __IR_H__
#define __IR_H__

void IR_Init();
uint8_t IR_GetDataFlag();
uint8_t IR_GetRepeatFlag();
uint8_t IR_GetAddress();
uint8_t IR_GetCommand();

#endif