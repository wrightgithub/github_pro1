#ifndef _POS_CTR_H_
#define _POS_CTR_H_
#include "include.h"
/***************
摄像头定位控制           
***************/
/****************************************
			坐标       ^X
			           |
			       ----|---->Y
			           |
			           |
*****************************************/


STRUCT(_CAM_POS)
{
	vs16 distance;
	vs16 valid_distance;
	vs16 last_valid_distance;
	volatile float v;
	volatile float last_v;
	bool dis_valid;
};
STRUCT(CAM_POSITION)
{
  _CAM_POS pos_ctr_x;
  _CAM_POS pos_ctr_y;
	volatile float ROLL;
	volatile float PITCH;
	FLOW_fixedposition_mode FixedPos_Mode;//定位模式 选择
	u8 binaryzation_threshold;
	u8 binary_rec_temp;
};
EXTERN(CAM_POSITION,cam_pos_ctr);


/*************
位置控制pid
**************/
STRUCT(_POSPID)
{
  float P;
	float I;
	float D;
	float integral;
	float I_OUT;
	float pid_out;
  float	last_error[2];
};
STRUCT(_TACHE)
{
_POSPID Shell;
_POSPID Core;
};
STRUCT(_PIDCTR)
{
  _TACHE  PosPid_X;
	_TACHE  PosPid_Y;
};

EXTERN(_PIDCTR, pos_pid_ctr);

 
void cascade_PosControl(Track_Mode mode ,_PIDCTR* Ctr,const vs16 *const x_dis,const vs16 *const y_dis,const volatile float*const x_v,const volatile float*const y_v);
void cascade_PosControl_V2(_PIDCTR* Ctr,const vs16 *const x_dis,const vs16 *const y_dis,const volatile float*const x_v,const volatile float*const y_v);
float Pos_Ctr_LowPass(float now_x,float last_x ,float a);
float Get_Pos_v(vs16 * now_pos,vs16 * last_pos,float dt);
void Init_PosPid(_POSPID *pid ,float P ,float I ,float D);
bool Pos_Is_Value(vs16 *distance,vs16 *last_distance,int top_value);
int Is_Exceed_MaxValue(const volatile float *value,float max,float min);
void SinglePid(_PIDCTR* Ctr,const vs16 *const x_dis,const vs16 *const y_dis);
void SinglePid_V2(_PIDCTR* Ctr,const volatile float*const x_dis,const volatile float*const y_dis,const volatile float*const x_v,const volatile float*const y_v);
void Angle_Compensate(float a_roll, float a_pit);
void Fixedpoint_Data_Handling(void);
void Tracking_Data_Handling(void);
u8 Is_find_circle(u8 par,bool* circle_find);

#endif

