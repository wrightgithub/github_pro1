#ifndef __DATA_H
#define	__DATA_H

#include "UserSys.h"

#define PAR_FRAME 0X51  //≤Œ ˝÷°
#define ORDER_FRAME 0X55   //÷∏¡Ó÷°
#define BYTE0(dwTemp)       (*(char *)(&dwTemp))
#define BYTE1(dwTemp)       (*((char *)(&dwTemp) + 1))
#define BYTE2(dwTemp)       (*((char *)(&dwTemp) + 2))
#define BYTE3(dwTemp)       (*((char *)(&dwTemp) + 3))	

void Data_Send_Status(const float rol,const float pit,const float yaw,const float Alt_CSB,const float Alt,const u8 ARMED);
void Data_Send_Senser(const int ax,const int ay,const int az,const int gx,const int gy,const int gz,const int mx,const int my,const int mz);
void PidDataReceive(void);
void Data_Send_PID2(float alt_p,float alt_i,float alt_d);
void MAVLINK_OPTICAL_FLOW_RECEIVE(const u8 *rxbuf);
void Data_Send_Check(u16 check,u8 *data_to_send);
void Receive_CameraData(u8 *buf);
void Report_Message(u8* _SendCount);
void Data_Send_Image_Par(u8 * USART_TX_BUF,u8 function,const int ax,const int ay);
void Track_Mode_Send(void);
#endif
