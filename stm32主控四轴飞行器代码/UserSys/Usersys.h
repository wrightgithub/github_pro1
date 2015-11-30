#ifndef __USERSYS_H_
#define __USERSYS_H_


/**************模式选择*****************/

#define KS103SONAR            //KS103超声波
#define Altitude_Lowpass      //低通滤波
#define PIDHOLD               //pid定高
#define PID_POS   						//PID定位

#define PID_POS_DEBUG  			//PID定位调试
//#define ATTITUDE_PID_DEBUG  //姿态调试
//#define DOUBLE_PIDHOLD_DEBUG  //串级pid定高调试


/*****************************************/
#ifdef ATTITUDE_PID_DEBUG
  #define Reset_thro 1340
#else
  #define Reset_thro 1290
#endif
//#define FINAL_HIGH  100
#define Reset_high  0  //复位的高度
#define Sonar_Ctr_Frequency 12.5 //超声波定高周期80ms，12.5hz
#define Control_Frequency 200  //姿态控制周期5ms,200hz
#define Pos_Contro_fre  33.33  //定点控制频率
#define Pos_Contro_T_Count  12 //30ms 
#define V_T  0.03//30MS
#define IMAGE_BUF_NUM 601
#include "include.h" 	
#include "stm32f10x.h"
#include "led.h"
#include "app.h"
#include "oled.h"
#include "key.h"
#include "time.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "IMU.h"
#include "CONTROL.h"
#include "moto.h"
#include "USART.h"
#include "delay.h"
#include "Param.h"
#include "data.h"
#include "pos_ctr.h"
#include "sonar_ctr.h"
#include "track_line.h"
#include "menu.h"


/******外部声明********/
extern u8 camera_image_buf[IMAGE_BUF_NUM]; 
extern u8 key;
extern  float pit[6];
extern  float roll[6];
extern  float trackline_roll[8];
extern u16 ms0,ms1,ms2,ms3,ms4,ms50,ms6_oled,ms7,ms8;	//中断次数计数器
extern u8 USART3_RX_BUF[50];
extern u8 USART3_TX_BUF[64];
extern u8 USART2_TX_BUF[2]; 
extern u8 USART2_RX_BUF[64];
extern u8 USART1_RX_BUF[64];
extern u8 USART1_RX_STA ;
extern u8 USART2_RX_STA;
extern u8 USART3_RX_STA;
extern u8  sentDateFlag;
extern u8  prenum;
extern u8 SendCount;
extern u8 UltraSendFlag; //串口超声波发送读取数据指令
extern bool start_up;
extern bool sonar_valid ;
extern float pitch_offset;//-1;  //前飘给正，后飘给负
extern float roll_offset;//1.5;    //左飘给正，右飘给负

extern u16 START_TRACK_DELAY;  //等待多少时间开始循迹
extern u16 AUTO_LAND_DELAY  ;  //等待多少时间自动降落
/*******************************************************************/

/****************
延时结构体
*****************/
STRUCT(_DELAY)
{
	u16 ms_yaw;
	bool ms_yaw_ok;
	u16 ms_ultra;
	bool ms_ultra_ok;
	u16 ms_idling;
	bool ms_idling_ok;//怠速
	u16 ms_land;
	bool start_count_ms_land;
	u16 ms_start_track;
	bool start_count_ms_track;
	u16 ms_toward_line;
	bool start_ms_toward_line;
	u16 ms_ABC_change_yaw;
	bool start_ABC_change_yaw;
	u16 ms_point_B_stabilize;
	bool start_B_stabilize;
	enum{NONE=0,STABILIZE_OK=1} POINT_B_stabilize;
	u16 ms_search_circle_B;
	bool start_search_circle_B;
	u16 ms_B_TO_C;
	bool ms_B_TO_C_OK;
	u16 ms_allwait_start_track;
  bool allwait_start_track_ok;
};
EXTERN(_DELAY,struct_delay);




/***********函数声明****************/
unsigned long Middle( unsigned long *ArrDataBuffer);
float UltraMiddle( volatile float *ArrDataBuffer);
void FloatToByte(float floatNum,unsigned char* byteArry);
float ByteToFloat(unsigned char* byteArry);
int ByteToInt(unsigned char* byteArry);//字节转浮点数
float max(float a,float b);
int16_t MotorSmoothing(int16_t newvalue, int16_t oldvalue);
void First_Delay(_DELAY *delay);
void  Receive_SetHigh(float *high,bool valid_high,AltHold sethigh_mode,float goal);
void AutoLand_Mode(float *high);
void Set_Heading(float *now_angle);

#endif









