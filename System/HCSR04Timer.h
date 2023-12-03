#ifndef __HCSR04TIMER_H__
#define __HCSR04TIMER_H__

void HCSR04Timer_Init();
uint16_t HCSR04Timer_GetCounter();
void HCSR04Timer_SetCounter(uint16_t Counter);
void HCSR04Timer_Cmd(FunctionalState NewState);

#endif