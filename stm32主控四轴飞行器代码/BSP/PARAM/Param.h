#ifndef _PARAM_H_
#define _PARAM_H_
/* Includes ------------------------------------------------------------------*/
#include "UserSys.h"

extern uint16_t VirtAddVarTab[15];

void EE_SAVE_ACC_OFFSET(void);

void paramLoad(void);
void Rc_NRFreceive(void);
void SavePid(void);
void ReadPid(void);
#endif /* __EEPROM_H */

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/

