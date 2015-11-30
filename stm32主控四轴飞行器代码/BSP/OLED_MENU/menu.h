#ifndef __MENU_H_
#define	__MENU_H_
#include "UserSys.h"
#define LAYER_NUM 7
/****************************/
typedef enum{
	Layer0   =0,
	Layer1   =1,
	Layer2   =2,
	Layer3   =3,
	Layer4   =4,
	Layer5   =5,
	Layer6   =6
}_LAYER;
extern u8 Image_Par_buf[20];
extern _LAYER Now_Layer;
void OLED_MENU(_LAYER *Now_Layer);
void DisPlay_Layer0(void);
void DisPlay_Layer1(void);
void DisPlay_Layer2(void);
void Layer2_Reduce_Par(const u16 *next);
void Layer2_Add_Par(const u16 *next);
void DisPlay_Layer3(void);
void DisPlay_Layer4(void);
void DisPlay_Layer5(void);
void DisPlay_Layer6(void);
#endif



