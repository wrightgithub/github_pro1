#ifndef _TRACK_LINE_H_
#define _TRACK_LINE_H_
#include "UserSys.h"

#define CIRCLE_CHECK_NUM 10
typedef struct 
{
	u8 receive_find;
  bool circle_find;
//	u8 circle[CIRCLE_CHECK_NUM];
	u8 check_count;
}_CIRCLE;

typedef struct
{
	bool once_in;
	float circle_angle_tolerance;
	float second_angle_tolerance;
	int final_alladd;
	int have_addangle;
	bool save_angle_tolerance;
	bool rec_yaw_second_once;
	enum {NOT_START=0,START_TOWARD=1,FINISH_TOWARD=2}toward_line_mode;
}_YAW;
typedef struct
{
	vs16 valid_offset;
	vs16 offset;
	vs16 last_offset;
	volatile float off_v;
	
  _CIRCLE Check;
	Track_Mode track_mode;
	float goal_angle;//最终加在pid里的角度
	
	u8 circle_count;
  _YAW Change_yaw;
	
	float  forward_angle; 
	u8 line_find;
}_TRACKING_OFFSET;
extern _TRACKING_OFFSET tracking_offset;

typedef struct
{
  bool start_B_to_A;
	float forward_angle;
	bool once_in;
	
}_MODE_ABA;
extern _MODE_ABA mode_ABA;

void Set_forward_angle(float angle);
void Mode_Track_Convert(void);
void Change_Yaw(u16 ms_delay);
void B_to_C_yaw(float yaw);
void Set_Track_delay(u16 delay_num);
void Set_AutoLand_delay(u16 delay_num);
void Start_Track_Delay(void);
void Mode_ABA_OF_B_to_A(void);
#endif


