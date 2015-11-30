#include "mode.h" 
#include "status.h"
_MODE Plane_Mode;
_MODE_STATIC_PAR Static_Par;

/********************
定点模式
*********************/
u8 Mode_Fixed_Position(CAM_POSITION* cam_pos_ctr)
{
	 u8 _start_pos=0; 
	if(cam_pos_ctr->FixedPos_Mode==OPEN)	//定点模式开
	{
		_start_pos=30;
		cam_pos_ctr->FixedPos_Mode=FIX_ING;//定位进行中
	}		
	else
	{	
		_start_pos=0;		
	}
	return _start_pos;
}

/********************************
定高模式
********************************/
void Mode_AltHold(Ultrasound* Ultra,bool _auto_liftoff,_SONAR_PID* sonar_pid)
{
	if(Ultra->SetHigh.Mode==HOLDOPEN)   	 /********定高模式*********/
	{		
			//LED3(ON);
//		if(_auto_liftoff==true)//开启自动起飞模式//1300
//		{
//			LED3(ON);
//			Ultra->AltHoldThro= 1300;//记下开启定高模式时的油门
//		}
		
		Ultra->SetHigh.Mode=SetHigh_ing;       //Ultra.SetHigh.Mode!=0 表示定高模式正在开启
	}		
}
/****************************
接收上位机数据后，进行模式选择
******************************/
void Mode_Select(u8 *value,_MODE *mode)
{
	switch(*value)
	{
	  case 0x01://解锁&起飞
		{
			Static_Par.lock_num++;
	
			if(Static_Par.lock_num==1)		  //按第一下
			{
		    mode->unlock=true;
			}
			else if(Static_Par.lock_num==2)	//按第二下
			{
				Static_Par.lock_num=0;
			  mode->unlock=false;
			}
		}break;
		case 0x02://降落
		{
		  Static_Par.start_num++;
	
			if(Static_Par.start_num==1)		  //按第一下
			{
		    mode->land=true;
			}
			else if(Static_Par.start_num==2)	//按第二下
			{
				Static_Par.start_num=0;
			  mode->land=false;
			}
		}break;
		case 0x04:  //定高
		{
		  Static_Par.althold_num++;
	
			if(Static_Par.althold_num==1)		  //按第一下
			{
		    mode->althold=true;
			}
			else if(Static_Par.althold_num==2)	//按第二下
			{
				Static_Par.althold_num=0;
			  mode->althold=false;
			}		  
		}break;
		case 0x05:  //定点
		{
		  Static_Par.poshold_num++;
	
			if(Static_Par.poshold_num==1)		  //按第一下
			{
		    mode->poshold=true;  //定点开
				mode->althold=true; //同时开定高
			}
			else if(Static_Par.poshold_num==2)	//按第二下
			{
				Static_Par.poshold_num=0;
			  mode->poshold=false; //定点关
				mode->althold=false; //同时关定高
			}		  
		}break;
		
		default:
			break;
	}
}
/***************************
执行模式
*****************************/
void Mode_Execute( _MODE *mode)
{
	//解锁起飞&上锁
	if(mode->unlock==true)
	{
    mode->ARMED=true;//解锁&起飞
		if(Static_Par.set_heading_ok==false)
		{
		  Set_Heading(&angle.yaw);
			Static_Par.set_heading_ok=true;
		}
	}
	else
	{
    mode->ARMED=false;//上锁
		Reset(Ultra.SetHigh.HighPidPwm);//先把上次的值清零
		
		memset(&sonar_shell_pid.static_inc,0,sizeof(sonar_shell_pid.static_inc));
		memset(&sonar_core_pid.static_inc, 0,sizeof(sonar_core_pid.static_inc));
		
		Set_GoalHigh(Reset_high);
	}
	
	//降落开&降落关
	if(mode->land==true)
	{
		
	}
	else
	{
		
	}
	
	//定高开&定高关****开定点的同时开定高
	if(mode->althold==true||mode->poshold==true)
	{
		LED3(ON);
		if(Ultra.SetHigh.Mode!=SetHigh_ing)
		{
			
	    Ultra.SetHigh.Mode=HOLDOPEN;    //定高模式开
		}
	}
  else
	{
		LED3(OFF);
	  Ultra.SetHigh.Mode=HOLDCLOSED;   //定高模式关
		
		Reset(Ultra.SetHigh.HighPidPwm);//先把上次的值清零
		memset(&sonar_shell_pid.static_inc,0,sizeof(sonar_shell_pid.static_inc));
		memset(&sonar_core_pid.static_inc, 0,sizeof(sonar_core_pid.static_inc));
		
	}
	
	//定点开&定点关
	if(mode->poshold==true)
	{
	  cam_pos_ctr.FixedPos_Mode =OPEN;  //定位模式开	 
		
	}
	else
	{
	  cam_pos_ctr.FixedPos_Mode =CLOSED;  //定位模式关
	}
 
}


void Reset_Static_Par(bool *flag,u8 chance )
{
	switch(chance)
	{
		case PARAMETER:
				if(*flag==true)
				{
					memset(&Static_Par,0,sizeof(Static_Par));
					memset(&Plane_Mode,0,sizeof(Plane_Mode));
					
					Reset(Ultra.SetHigh.HighPidPwm);//先把上次的值清零
		      memset(&sonar_shell_pid.static_inc,0,sizeof(sonar_shell_pid.static_inc));
		      memset(&sonar_core_pid.static_inc, 0,sizeof(sonar_core_pid.static_inc));
					Set_GoalHigh(Reset_high);
					
					*flag=false;
				}break;
		case INTEGRAL:
			  if(*flag==false)//false:上锁
				{				
					Reset(ctrl.pitch.shell.increment);
					Reset(ctrl.pitch.core.increment);
					Reset(ctrl.roll.shell.increment);
					Reset(ctrl.roll.core.increment);
					Reset(Static_Par.set_heading_ok);
					Reset(struct_delay.ms_idling);
					Reset(struct_delay.ms_idling_ok);
					Reset(struct_delay.start_ABC_change_yaw);
					struct_delay.start_search_circle_B=false;
					struct_delay.POINT_B_stabilize=NONE;
					struct_delay.allwait_start_track_ok=false;
					Reset(Static_Par.reach_start_thro);
					
					Ultra.AltHoldThro= Reset_thro;
					Reset(Static_Par.have_reached);
					Reset(Ultra.have_restrain);
					Reset(Ultra.have_received);
					

				}break;
				
		case POS_INTEGRAL://定点积分清零
			if(*flag==false)//false:上锁
			{
			  //积分清零
				Reset(pos_pid_ctr.PosPid_X.Core.integral);
				Reset(pos_pid_ctr.PosPid_X.Shell.integral);
				Reset(pos_pid_ctr.PosPid_Y.Core.integral);
				Reset(pos_pid_ctr.PosPid_Y.Shell.integral);
				
			  Reset(pos_pid_ctr.PosPid_X.Core.pid_out);
			  Reset(pos_pid_ctr.PosPid_Y.Core.pid_out);
				
				Reset(pos_pid_ctr.PosPid_X.Core.integral);
				Reset(pos_pid_ctr.PosPid_Y.Core.integral);
				Reset(pos_pid_ctr.PosPid_X.Shell.integral);
				Reset(pos_pid_ctr.PosPid_Y.Shell.integral);

				Reset(cam_pos_ctr.ROLL);
				Reset(cam_pos_ctr.PITCH);
				Reset(tracking_offset.circle_count);

				memset(&tracking_offset.Change_yaw,0,sizeof(tracking_offset.Change_yaw));
			}
		default:break;
			
  }
}


//void Reset_All_Par(bool* flag)
//{
//  if(*flag==false)
//	{
//		memset(&Static_Par,0,sizeof(Static_Par));
//		memset(&Plane_Mode,0,sizeof(Plane_Mode));
//		
//		Reset(Ultra.SetHigh.HighPidPwm);//先把上次的值清零
//		memset(&sonar_shell_pid.static_inc,0,sizeof(sonar_shell_pid.static_inc));
//		memset(&sonar_core_pid.static_inc, 0,sizeof(sonar_core_pid.static_inc));
//		Set_GoalHigh(Reset_high);
//		
//		Reset(pos_pid_ctr.PosPid_X.Core.integral);
//		Reset(pos_pid_ctr.PosPid_X.Shell.integral);
//		Reset(pos_pid_ctr.PosPid_Y.Core.integral);
//		Reset(pos_pid_ctr.PosPid_Y.Shell.integral);
//		Reset(pos_pid_ctr.PosPid_X.Core.pid_out);
//		Reset(pos_pid_ctr.PosPid_Y.Core.pid_out);
//	}
//}

/**********************
怠速，准备起飞函数
**********************/
void Plane_Idling(bool*falg,vs16 *Moto,u16 pwm)
{
	if(*falg==false)
	{
	  Moto[0]=pwm-1000;
		Moto[1]=pwm-1000;
		Moto[2]=pwm-1000;
		Moto[3]=pwm-1000;
	}
}

/**************************
状态机
***************************/
_FLY_MODE fly_mode=LINE_ABC;//初始的时候为定高模式
_MODE_STEP alt_hold_process;//定高
_MODE_STEP fixed_point_process;//定点
_MODE_STEP track_line_process;//循迹
_MODE_STEP line_ABC_process;//ABC
_MODE_STEP line_ABA_process;//ABA
void State_Machine(_FLY_MODE *fly_mode)
{
  switch(*fly_mode)
	{
	  case NO_WORK://不工作的时候
		{
		  break;
		}
		case ALT_HOLD://定高
		{
		  AltHold_Mode_Ctr(&alt_hold_process);
			break;
		}
		case FIXED_POINT://定点
		{
		  Fixed_Point_Mode_Ctr(&fixed_point_process);
			break;
		}
		case TRACK_LINE://循迹
		{
			Track_Line_Process(&track_line_process);
			break;
		}
		case LINE_ABC://校赛ABC
		{
		  Track_line_ABC(&line_ABC_process);
		  break;
		}
		case TRACK_ABA://13年国赛ABA
		{
		  Track_line_ABA(&line_ABA_process);
		}
		default :break;
	}

}













