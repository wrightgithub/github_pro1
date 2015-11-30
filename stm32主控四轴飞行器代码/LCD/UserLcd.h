#ifndef __USERLCD_H
#define __USERLCD_H 
#include "UserSys.h"
#include "sys.h" 
#include "lcd.h"
#include "5110.h"
#include "oled.h"
#include "remote.h"
void UserLcd_Init(void);
void UserLcdShow(void);
u8 *RemoteShow(void);//∫ÏÕ‚TFTœ‘ æ
#endif

