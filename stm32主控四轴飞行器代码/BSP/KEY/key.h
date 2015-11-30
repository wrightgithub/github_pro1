#ifndef _KEY_H_
#define _KEY_H_

#include "stm32f10x.h"
#include "UserSys.h"

#define  KEY0_POS (1<<12)     //KEY0   PE12
#define  KEY1_POS (1<<13)     //KEY1   PE13
#define  KEY2_POS (1<<14)     //KEY2   PE14
#define  KEY3_POS (1<<15)     //KEY3   PE15

#define  KEY0_GET ((GPIOE->IDR&(KEY0_POS))?1:0)//读取按键 0
#define  KEY1_GET ((GPIOE->IDR&(KEY1_POS))?1:0)//读取按键 1
#define  KEY2_GET ((GPIOE->IDR&(KEY2_POS))?1:0)//读取按键 2
#define  KEY3_GET ((GPIOE->IDR&(KEY3_POS))?1:0)//读取按键 3

STRUCT(_SET_LAYER)
{
  u16 next_line;
	u16 line0;
	u16 line1;
	u16 line2;
	u16 line3;
	u16 line4;
	u16 line5;
	u16 line6;
	u16 line7;
};
extern _SET_LAYER Set_Layer;



void KEY_Init(void);
uint8_t Key_Scan(void);
void Menu_Layer_Select(u8* key);
void Key_Mode_Start(void);
void Fly_Start_Count(u8 *count);
void Layer6_Add_Par(const u16 *next);
void Layer6_Reduce_Par(const u16 *next);
#endif



