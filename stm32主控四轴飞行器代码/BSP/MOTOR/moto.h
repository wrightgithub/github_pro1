#ifndef _MOTO_H_
#define _MOTO_H_
#include "stm32f10x.h"

#define Moto_PwmMax 1000  //1300

void Moto_PwmRflash(uint16_t MOTO1_PWM,uint16_t MOTO2_PWM,uint16_t MOTO3_PWM,uint16_t MOTO4_PWM);
void Moto_Init(void);
void UltraMoto_PwmRflash(uint16_t MOTO1_PWM,uint16_t MOTO2_PWM,uint16_t MOTO3_PWM,uint16_t MOTO4_PWM);
void Set_Moto_Compensate(int* moto,int up,int down,int left,int right);
#endif
