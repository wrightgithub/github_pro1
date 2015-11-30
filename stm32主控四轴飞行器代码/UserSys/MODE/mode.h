#ifndef _MODE_H_
#define _MODE_H_
#include "pos_ctr.h"
#include "sonar_ctr.h"
#include "led.h"
#include "control.h"

#define PARAMETER 1
#define INTEGRAL 2
#define POS_INTEGRAL 3

//#define MODE(par,num) if((par)>=(num)){ (par)=0;
//#define END_MODE }
typedef enum{
	status_step0_none=0,
	status_step1=1,
	status_step2=2,
	status_step3=3,
	status_step4=4,
	status_step5=5,
	status_step6=6
}_MODE_STEP;

#define mode_max 5
#define mode_min 1
typedef enum{
	NO_WORK    =0,
	ALT_HOLD   =1,
	FIXED_POINT=2,
	TRACK_LINE =3,
	LINE_ABC   =4,
	TRACK_ABA  =5  
}_FLY_MODE;
extern _FLY_MODE fly_mode;
STRUCT(_MODE)
{
	bool unlock;//true:解锁&起飞 false:上锁
  bool land;//true:降落开  false:降落关
	
	bool althold;//true:开定高  false:关定高
	bool poshold;//true:开定点  false:关定点
	
	bool auto_liftoff;//自动起飞模式
 // bool auto_land;//自动降落模式
	bool ARMED ;//0：上锁，1：解锁
	
	bool reset_start;
	bool fly_start;
	bool Receive_Img;
  bool Rec_Img_ok;
	u8 fly_count;
};
EXTERN(_MODE,Plane_Mode);

STRUCT(_MODE_STATIC_PAR)
{
  u8 lock_num;
	u8 start_num;
	u8 althold_num;
	u8 poshold_num;
	bool set_heading_ok;
	bool have_reached;
	bool reach_start_thro;
	u8 goal_high_fre; 
};

EXTERN(_MODE_STATIC_PAR,Static_Par);

void Mode_Select(u8 *value,_MODE *mode);
u8 Mode_Fixed_Position(CAM_POSITION* cam_pos_ctr);
void Mode_AltHold(Ultrasound* Ultra,bool _auto_liftoff,_SONAR_PID* sonar_pid);
void Mode_Execute(_MODE *mode);
void Reset_Static_Par(bool *flag,u8 chance );
void Reset_All_Par(bool* flag);
void Plane_Idling(bool*falg,vs16 *Moto,u16 pwm);
void State_Machine(_FLY_MODE *fly_mode);
#endif

