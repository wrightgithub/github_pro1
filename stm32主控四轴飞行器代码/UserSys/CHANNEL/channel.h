#ifndef __CHANNEL_H_
#define __CHANNEL_H_
#include "UserSys.h"
/*用来存放与遥控的数据交互*/
typedef struct int16_rcget
{
	      u8  NRF24L01_RXDATA[33];
	      u8  NRF24L01_TXDATA[33];
				int16_t ROLL;
				int16_t PITCH;
				int16_t THROTTLE;
				int16_t YAW;
	      int16_t Switch; //遥控器的一个输入量
	      int16_t AUX1;//辅助通道1
	
	      int16_t roll_offset;
	      int16_t pitch_offset;//零偏
	      int16_t yaw_offset;
}T_RC_DATA;
				
extern T_RC_DATA Rc_Data;
FLOW_fixedposition_mode Channel_Max(T_RC_DATA* Rc_Data,bool* _poshold);
void Channel_Middle(Ultrasound* Ultra,_SONAR_PID* sonar_shell_pid,_SONAR_PID* sonar_core_pid,T_RC_DATA* Rc_Data,bool* _poshold);
bool Channel_AUX1(_SONAR_PID* sonar_pid,T_RC_DATA* Rc_Data);
#endif
