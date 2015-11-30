#include "mode.h" 
#include "status.h"
_MODE Plane_Mode;
_MODE_STATIC_PAR Static_Par;

/********************
����ģʽ
*********************/
u8 Mode_Fixed_Position(CAM_POSITION* cam_pos_ctr)
{
	 u8 _start_pos=0; 
	if(cam_pos_ctr->FixedPos_Mode==OPEN)	//����ģʽ��
	{
		_start_pos=30;
		cam_pos_ctr->FixedPos_Mode=FIX_ING;//��λ������
	}		
	else
	{	
		_start_pos=0;		
	}
	return _start_pos;
}

/********************************
����ģʽ
********************************/
void Mode_AltHold(Ultrasound* Ultra,bool _auto_liftoff,_SONAR_PID* sonar_pid)
{
	if(Ultra->SetHigh.Mode==HOLDOPEN)   	 /********����ģʽ*********/
	{		
			//LED3(ON);
//		if(_auto_liftoff==true)//�����Զ����ģʽ//1300
//		{
//			LED3(ON);
//			Ultra->AltHoldThro= 1300;//���¿�������ģʽʱ������
//		}
		
		Ultra->SetHigh.Mode=SetHigh_ing;       //Ultra.SetHigh.Mode!=0 ��ʾ����ģʽ���ڿ���
	}		
}
/****************************
������λ�����ݺ󣬽���ģʽѡ��
******************************/
void Mode_Select(u8 *value,_MODE *mode)
{
	switch(*value)
	{
	  case 0x01://����&���
		{
			Static_Par.lock_num++;
	
			if(Static_Par.lock_num==1)		  //����һ��
			{
		    mode->unlock=true;
			}
			else if(Static_Par.lock_num==2)	//���ڶ���
			{
				Static_Par.lock_num=0;
			  mode->unlock=false;
			}
		}break;
		case 0x02://����
		{
		  Static_Par.start_num++;
	
			if(Static_Par.start_num==1)		  //����һ��
			{
		    mode->land=true;
			}
			else if(Static_Par.start_num==2)	//���ڶ���
			{
				Static_Par.start_num=0;
			  mode->land=false;
			}
		}break;
		case 0x04:  //����
		{
		  Static_Par.althold_num++;
	
			if(Static_Par.althold_num==1)		  //����һ��
			{
		    mode->althold=true;
			}
			else if(Static_Par.althold_num==2)	//���ڶ���
			{
				Static_Par.althold_num=0;
			  mode->althold=false;
			}		  
		}break;
		case 0x05:  //����
		{
		  Static_Par.poshold_num++;
	
			if(Static_Par.poshold_num==1)		  //����һ��
			{
		    mode->poshold=true;  //���㿪
				mode->althold=true; //ͬʱ������
			}
			else if(Static_Par.poshold_num==2)	//���ڶ���
			{
				Static_Par.poshold_num=0;
			  mode->poshold=false; //�����
				mode->althold=false; //ͬʱ�ض���
			}		  
		}break;
		
		default:
			break;
	}
}
/***************************
ִ��ģʽ
*****************************/
void Mode_Execute( _MODE *mode)
{
	//�������&����
	if(mode->unlock==true)
	{
    mode->ARMED=true;//����&���
		if(Static_Par.set_heading_ok==false)
		{
		  Set_Heading(&angle.yaw);
			Static_Par.set_heading_ok=true;
		}
	}
	else
	{
    mode->ARMED=false;//����
		Reset(Ultra.SetHigh.HighPidPwm);//�Ȱ��ϴε�ֵ����
		
		memset(&sonar_shell_pid.static_inc,0,sizeof(sonar_shell_pid.static_inc));
		memset(&sonar_core_pid.static_inc, 0,sizeof(sonar_core_pid.static_inc));
		
		Set_GoalHigh(Reset_high);
	}
	
	//���俪&�����
	if(mode->land==true)
	{
		
	}
	else
	{
		
	}
	
	//���߿�&���߹�****�������ͬʱ������
	if(mode->althold==true||mode->poshold==true)
	{
		LED3(ON);
		if(Ultra.SetHigh.Mode!=SetHigh_ing)
		{
			
	    Ultra.SetHigh.Mode=HOLDOPEN;    //����ģʽ��
		}
	}
  else
	{
		LED3(OFF);
	  Ultra.SetHigh.Mode=HOLDCLOSED;   //����ģʽ��
		
		Reset(Ultra.SetHigh.HighPidPwm);//�Ȱ��ϴε�ֵ����
		memset(&sonar_shell_pid.static_inc,0,sizeof(sonar_shell_pid.static_inc));
		memset(&sonar_core_pid.static_inc, 0,sizeof(sonar_core_pid.static_inc));
		
	}
	
	//���㿪&�����
	if(mode->poshold==true)
	{
	  cam_pos_ctr.FixedPos_Mode =OPEN;  //��λģʽ��	 
		
	}
	else
	{
	  cam_pos_ctr.FixedPos_Mode =CLOSED;  //��λģʽ��
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
					
					Reset(Ultra.SetHigh.HighPidPwm);//�Ȱ��ϴε�ֵ����
		      memset(&sonar_shell_pid.static_inc,0,sizeof(sonar_shell_pid.static_inc));
		      memset(&sonar_core_pid.static_inc, 0,sizeof(sonar_core_pid.static_inc));
					Set_GoalHigh(Reset_high);
					
					*flag=false;
				}break;
		case INTEGRAL:
			  if(*flag==false)//false:����
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
				
		case POS_INTEGRAL://�����������
			if(*flag==false)//false:����
			{
			  //��������
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
//		Reset(Ultra.SetHigh.HighPidPwm);//�Ȱ��ϴε�ֵ����
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
���٣�׼����ɺ���
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
״̬��
***************************/
_FLY_MODE fly_mode=LINE_ABC;//��ʼ��ʱ��Ϊ����ģʽ
_MODE_STEP alt_hold_process;//����
_MODE_STEP fixed_point_process;//����
_MODE_STEP track_line_process;//ѭ��
_MODE_STEP line_ABC_process;//ABC
_MODE_STEP line_ABA_process;//ABA
void State_Machine(_FLY_MODE *fly_mode)
{
  switch(*fly_mode)
	{
	  case NO_WORK://��������ʱ��
		{
		  break;
		}
		case ALT_HOLD://����
		{
		  AltHold_Mode_Ctr(&alt_hold_process);
			break;
		}
		case FIXED_POINT://����
		{
		  Fixed_Point_Mode_Ctr(&fixed_point_process);
			break;
		}
		case TRACK_LINE://ѭ��
		{
			Track_Line_Process(&track_line_process);
			break;
		}
		case LINE_ABC://У��ABC
		{
		  Track_line_ABC(&line_ABC_process);
		  break;
		}
		case TRACK_ABA://13�����ABA
		{
		  Track_line_ABA(&line_ABA_process);
		}
		default :break;
	}

}













