#include "channel.h"
T_RC_DATA Rc_Data;//1000~2000
/*****************************
遥控器通道选择
*****************************/
//定位档
FLOW_fixedposition_mode Channel_Max(T_RC_DATA* Rc_Data,bool* _poshold)
{
	FLOW_fixedposition_mode FixedPos_Mode;
	if(Rc_Data->Switch>1800)      //三段开关的最大档
	{
		FixedPos_Mode=OPEN;  //定位模式开	
		*_poshold=true;				
	}
	else
	{
	  *_poshold=false;	
	  FixedPos_Mode=CLOSED;  //定位模式关
	} 
	
	return FixedPos_Mode;
}

//定高档
void Channel_Middle(Ultrasound* Ultra,_SONAR_PID* sonar_shell_pid,_SONAR_PID* sonar_core_pid,T_RC_DATA* Rc_Data,bool* _poshold)
{
	if((Rc_Data->Switch>1450&&Rc_Data->Switch<1550)|| *_poshold==true)  //三段开关的中间档
	{
		*_poshold=false;
		if(Ultra->SetHigh.Mode!=SetHigh_ing)
		{
		  Ultra->SetHigh.Mode=HOLDOPEN;    //定高模式开
		}
	}
	else
	{
		Ultra->SetHigh.Mode=HOLDCLOSED;   //定高模式关
		
		Reset(Ultra->SetHigh.GoalHigh);
		Reset(Ultra->SetHigh.HighPidPwm);//先把上次的值清零
		Reset(sonar_shell_pid->integral );
		Reset(sonar_core_pid->integral );
		
	}
			
}

/***************************
降落档
***************************/
bool Channel_AUX1(_SONAR_PID* sonar_pid,T_RC_DATA* Rc_Data)
{
	bool _auto_land;
	if(Rc_Data->AUX1<1100)
	{
		_auto_land=true;//开启自动降落模式		
		//sonar_pid->setgoal=-5;	
	}
	else if(Rc_Data->AUX1<1500&&Rc_Data->AUX1>1490)
	{
		_auto_land=false;
	}
	
	return _auto_land;
}






