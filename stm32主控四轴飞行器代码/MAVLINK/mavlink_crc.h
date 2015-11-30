#ifndef __MAVLINK_CRC_H
#define	__MAVLINK_CRC_H
#include "UserSys.h"

		
unsigned int do_crc(	unsigned int reg_init,	unsigned char *message, 	unsigned int len);

#endif

