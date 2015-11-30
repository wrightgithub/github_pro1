#include "track_line.h"
#include "mode.h"
_TRACKING_OFFSET tracking_offset;
_MODE_ABA mode_ABA;
u16 START_TRACK_DELAY=500;  //�ȴ�����ʱ�俪ʼѭ��
u16 AUTO_LAND_DELAY  = 1600;  //�ȴ�����ʱ���Զ�����

void Start_Track_Delay(void)
{
	//��ʼѭ������ʱ

	//�ﵽ�趨�߶Ⱥ���ת��ɺ��ٹ�n s��ʼѭ��
	if(struct_delay.start_count_ms_track==true&&(tracking_offset.Change_yaw.toward_line_mode==FINISH_TOWARD))
	{
		//��ѭ����ʱ��ɲ��Ҹ߶ȳ���������
		if(  ((struct_delay.ms_start_track++)>=START_TRACK_DELAY)&&(Ultra.z<= (Ultra.FINAL_HIGH+10) ) )//2s��ʼѭ��
		{
			tracking_offset.track_mode=START_TRACK;//��ʼѭ��
			struct_delay.start_count_ms_track=false;
			struct_delay.ms_start_track=0;
//			if(tracking_offset.Change_yaw.once_in==true)
//			{
//			   if( (struct_delay.ms_B_TO_C++>1000)&&(struct_delay.ms_B_TO_C_OK==false))//2.5s
//				 {
//				   struct_delay.ms_B_TO_C_OK=true;
//				 }
//			}
			
		}
	}

}
/***************************
�趨ѭ��ǰ���ĽǶ�
****************************/
void Set_forward_angle(float angle)
{
  tracking_offset.forward_angle =angle;
}
/****************************************
ѭ����״̬ת��
****************************************/
void Mode_Track_Convert(void)
{
	//  <1>�����������ѭ��ģʽ�£����ҵ�Բ�˾ͽ������ѭ��ģʽ
	if(tracking_offset.track_mode==TRACK_ING)
	{
		if( (struct_delay.ms_search_circle_B++>=100)&&(struct_delay.start_search_circle_B==false))//2s
		{
		  struct_delay.start_search_circle_B=true;
			struct_delay.ms_search_circle_B=0;
		}
		
		if(struct_delay.start_search_circle_B==true)//���Կ�ʼѰ��B
		{
			if(tracking_offset.Check.circle_find==true)
			{
				tracking_offset.track_mode=END_TRACK;//�������ѭ��ģʽ
				tracking_offset.circle_count++;  //���ҵ���Բ����
				
				//  struct_delay.start_ABC_change_yaw =true;//��ʼABC��yaw��ת
					struct_delay.start_B_stabilize=true;//��ʼB�������
				
			}
		}
	}
			
	// <2>����ڿ�ʼѭ��������ѭ��ģʽ(�ǽ���ѭ��ģʽ)��
	if(tracking_offset.track_mode!=END_TRACK)
	{			
		//ǰ��
		tracking_offset.goal_angle=tracking_offset.forward_angle;//��ǰ����
		//if(	((tracking_offset.circle_count==1)&&(  struct_delay.ms_B_TO_C_OK==true)) ||(tracking_offset.circle_count==0) )
		//if(tracking_offset.line_find==1)
		if( tracking_offset.track_mode==TRACK_ING)
		{
			//λ��ת��
//			if(tracking_offset.circle_count==1)
//			{
//				static u16 last_offset=0;
//				if(Int_Fabs(tracking_offset.offset-last_offset)>40)
//				{
//					tracking_offset.offset=last_offset;
//				}
//				
//				last_offset=	tracking_offset.offset;
//			}
			
			cam_pos_ctr.pos_ctr_y.valid_distance =tracking_offset.offset;
			tracking_offset.off_v=constrain(tracking_offset.off_v,-20,20);
			cam_pos_ctr.pos_ctr_y.v              =tracking_offset.off_v;
			
		
		}
		
	}
	//���㲻ѭ��
	else if(tracking_offset.track_mode==END_TRACK)
	{
		tracking_offset.goal_angle=0;
	}
	//<3>������ڿ�ʼѭ��ģʽ���Ҳ���Բ�������ȫѭ��ģʽ
	 if(tracking_offset.track_mode==START_TRACK)
	{
	  if(tracking_offset.Check.circle_find==false)
		{
			//�Ҳ���Բ�������ȫѭ��ģʽ
			tracking_offset.track_mode=TRACK_ING;
		}
	}
}

/*******************************************
��ɸı�ƫ���Ƕ�
*******************************************/
void Change_Yaw(u16 ms_delay)
{
	static int _add_yaw=0;
	static bool rec_yaw_first_once=false;
	static float first_angle_tolerance=0;//�ϵ�˲���ƫ��
	u16 T=ms_delay/30;
	//��¼�¸��ϵ�״̬�µ�yaw
	if((Plane_Mode.ARMED==0)&&(rec_yaw_first_once==false))//����״̬
	{
	  first_angle_tolerance=angle.yaw;
		rec_yaw_first_once=true;
	}
	
	//��¼��ÿ�ν���һ˲���yaw 
	if((Plane_Mode.ARMED==1)&&(tracking_offset.Change_yaw.rec_yaw_second_once==false))
	{
	  tracking_offset.Change_yaw.second_angle_tolerance=angle.yaw;
		tracking_offset.Change_yaw.rec_yaw_second_once=true;
	}
	
	//��ʼת��ֱ��
	if(tracking_offset.Change_yaw.toward_line_mode==START_TOWARD)
	{
		if(tracking_offset.Change_yaw.save_angle_tolerance==false)
		{
			tracking_offset.Change_yaw.final_alladd=first_angle_tolerance-tracking_offset.Change_yaw.second_angle_tolerance ;//���յ�����
			if(tracking_offset.Change_yaw.final_alladd>0)//������ת
			{ 
				_add_yaw=1;
			}
			else if (tracking_offset.Change_yaw.final_alladd<0)
			{
			  _add_yaw=-1;
			}
			tracking_offset.Change_yaw.save_angle_tolerance=true;
		}
		
		//ÿ200ms�ۼ�һ��ƫ��
		if(struct_delay.ms_toward_line++>=T)
		{
			angle.heading+=_add_yaw;
			tracking_offset.Change_yaw.have_addangle++;
			struct_delay.ms_toward_line=0;
		}
		
		if(Int_Fabs(tracking_offset.Change_yaw.have_addangle)>=Int_Fabs(tracking_offset.Change_yaw.final_alladd))
		{
			tracking_offset.Change_yaw.toward_line_mode=FINISH_TOWARD; 
			//��תƫ����ɣ���λ����	
			_add_yaw=0;
		}	
		
	}
	
		
}


/***************************
B��C�ı�yaw��ֵ
***************************/
void B_to_C_yaw(float yaw)
{
	#define CHANGE_NUM 10
	static u16 have_change_yaw_count=0;
			
		if((struct_delay.start_B_stabilize==true)&&(struct_delay.POINT_B_stabilize==NONE))//��ʼB�㶨��
		{
			 if(struct_delay.ms_point_B_stabilize++>=1000)//s
			{
				 struct_delay.start_B_stabilize=false;//B�����̬���
				 struct_delay.POINT_B_stabilize=STABILIZE_OK;//B�����̬���
				
				 struct_delay.start_ABC_change_yaw=true;//��ʼABC��yaw��ת
				 struct_delay.ms_point_B_stabilize=0;//��ʱ����
			}
		}
	//��תֻ�ܽ���һ��
    if(tracking_offset.Change_yaw.once_in==false)
		{
			if(struct_delay.start_ABC_change_yaw==true)//��ʼABC��yaw��ת
			{
				 if((struct_delay.ms_ABC_change_yaw++)>=1)//125msתһ��
				 {
					 struct_delay.ms_ABC_change_yaw=0;
					 
					// angle.heading+=(yaw/(1.0*CHANGE_NUM));
					 angle.heading+=10;
					 DCT(0);
					 Ultra.AltHoldThro-=10;
					 //have_change_yaw_count++;
					 //if(have_change_yaw_count>=CHANGE_NUM)
					 {
							have_change_yaw_count=0;
						 
					    Set_forward_angle(mode_ABA.forward_angle);//������
//						  tracking_offset.offset=0; 
//						  tracking_offset.off_v=0;
//						  cam_pos_ctr.pos_ctr_y.valid_distance=0;
//						  cam_pos_ctr.pos_ctr_y.v=0;
						  struct_delay.start_search_circle_B=false;
						 
						  struct_delay.start_ABC_change_yaw=false;//��תABC_yaw�ر�
							struct_delay.start_count_ms_track=true;//׼����ʼѭ������ʱ
						  tracking_offset.Change_yaw.once_in=true;//��ֻ֤����һ��
					 }
				
				 }
				
			}
	}
}

/**********************************
ABA����ģʽ
***********************************/
void Mode_ABA_OF_B_to_A(void)
{
  	if((struct_delay.start_B_stabilize==true)&&(struct_delay.POINT_B_stabilize==NONE))//��ʼB�㶨��
		{
			 if(struct_delay.ms_point_B_stabilize++>=2000)//5s
			{
				 struct_delay.start_B_stabilize=false;//B�����̬���
				 struct_delay.POINT_B_stabilize=STABILIZE_OK;//B�����̬���
				 struct_delay.ms_point_B_stabilize=0;//��ʱ����
				
				 mode_ABA.start_B_to_A=true;//��ʼB��A��ģʽ
				 //Set_forward_angle(-2);//������
			}
		}
		if(mode_ABA.once_in==false )//��ֻ֤����һ��
		{
			if( mode_ABA.start_B_to_A==true)
			{
				Set_forward_angle(mode_ABA.forward_angle);//������

				struct_delay.start_count_ms_track=true;//׼����ʼѭ������ʱ
				
				mode_ABA.start_B_to_A=false;
				mode_ABA.once_in=true;
			}
	  }
}



void Set_Track_delay(u16 delay_num)
{
  START_TRACK_DELAY=delay_num;
}

void Set_AutoLand_delay(u16 delay_num)
{
  AUTO_LAND_DELAY=delay_num;
}

















