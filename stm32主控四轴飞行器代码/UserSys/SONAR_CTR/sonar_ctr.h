#ifndef __SONAR_CTR_H_
#define __SONAR_CTR_H_

#include "include.h"
#include "delay.h"
#include "IMU.h"
#include "usart.h"

STRUCT(SetHighMode)//����ģʽ�ṹ��
{
	int  HighPidPwm;//����pidʽ����������
	AltHold  Mode;//����ģʽ
};

//union KS103_SONAR //������
//{
//  u16 High;
//  u8 KS103_RX[2];
//};

STRUCT(Ultrasound)//�������ṹ��
{
	u16 AltHoldThro;
  float NowHigh;   //m
  float KS103High;   //m
	//union KS103_SONAR KS103;
	SetHighMode SetHigh;
	float MixHigh;
	float z;   //m
	float last_z;
  float	ultra_v;//m/s
	float last_ultra_v;
	float liftoff_goal_high;
	u16 Add_Thro;
	float FINAL_HIGH;
	bool have_restrain;//���ƶ��߳���
	bool have_received;//�ﵽ�趨�߶�

};
EXTERN(Ultrasound,Ultra);

/***************������PID�ṹ��*******************************/
STRUCT(_PID_STATIC_INC)
{
  float integral;
	float last_i_out;
	float last_error;
};
STRUCT(_SONAR_PID) //��������PID�ṹ��
{
  int16_t P;
	int16_t I;
	int16_t D;
	float setgoal;//m
 _PID_STATIC_INC static_inc;
};

//EXTERN(_SONAR_PID,sonar_pid);
EXTERN(_SONAR_PID,sonar_shell_pid);
EXTERN(_SONAR_PID,sonar_core_pid);

float SONAR_CONTROL(Ultrasound* ultra,_SONAR_PID* sonar_pid);
float SONAR_CONTROL_V2(Ultrasound* ultra,_SONAR_PID* sonar_shell_pid,_SONAR_PID* sonar_core_pid);
void Init_SonarPid( _SONAR_PID *pid,int16_t p,int16_t i,int16_t d);
float sonar_filter(bool *_sonar_valid,Ultrasound* _ultra);
void Send_KS103_Order(u8 *buf,bool wait_flag,u8 *cycle);
bool Sonar_Valid_Check(bool  _sonar_valid,Ultrasound* ultra);
void Get_Sonar_Speed(bool _sonar_valid,Ultrasound* ultra,float T);
void Send_US100(void);
float LowPass(bool flag,float apex,float a,float nowvalue ,float lastvalue) ;
void Set_GoalHigh(float high);
void Change_Goal_High( float *goalhigh,float final_high,u8 fre,float land_k);
void Reduce_shell_I( int16_t*  shell_I ,int16_t final_shell_I);
void Is_Start_Thro(float final_high,float k,u16 add_thro,float start_high);
void AltHold_Data_Handling(void);
void AltHold_Process_Ctr(void);
#endif





