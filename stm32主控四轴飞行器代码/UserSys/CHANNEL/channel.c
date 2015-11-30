#include "channel.h"
T_RC_DATA Rc_Data;//1000~2000
/*****************************
ң����ͨ��ѡ��
*****************************/
//��λ��
FLOW_fixedposition_mode Channel_Max(T_RC_DATA* Rc_Data,bool* _poshold)
{
	FLOW_fixedposition_mode FixedPos_Mode;
	if(Rc_Data->Switch>1800)      //���ο��ص����
	{
		FixedPos_Mode=OPEN;  //��λģʽ��	
		*_poshold=true;				
	}
	else
	{
	  *_poshold=false;	
	  FixedPos_Mode=CLOSED;  //��λģʽ��
	} 
	
	return FixedPos_Mode;
}

//���ߵ�
void Channel_Middle(Ultrasound* Ultra,_SONAR_PID* sonar_shell_pid,_SONAR_PID* sonar_core_pid,T_RC_DATA* Rc_Data,bool* _poshold)
{
	if((Rc_Data->Switch>1450&&Rc_Data->Switch<1550)|| *_poshold==true)  //���ο��ص��м䵵
	{
		*_poshold=false;
		if(Ultra->SetHigh.Mode!=SetHigh_ing)
		{
		  Ultra->SetHigh.Mode=HOLDOPEN;    //����ģʽ��
		}
	}
	else
	{
		Ultra->SetHigh.Mode=HOLDCLOSED;   //����ģʽ��
		
		Reset(Ultra->SetHigh.GoalHigh);
		Reset(Ultra->SetHigh.HighPidPwm);//�Ȱ��ϴε�ֵ����
		Reset(sonar_shell_pid->integral );
		Reset(sonar_core_pid->integral );
		
	}
			
}

/***************************
���䵵
***************************/
bool Channel_AUX1(_SONAR_PID* sonar_pid,T_RC_DATA* Rc_Data)
{
	bool _auto_land;
	if(Rc_Data->AUX1<1100)
	{
		_auto_land=true;//�����Զ�����ģʽ		
		//sonar_pid->setgoal=-5;	
	}
	else if(Rc_Data->AUX1<1500&&Rc_Data->AUX1>1490)
	{
		_auto_land=false;
	}
	
	return _auto_land;
}






