#include "status.h"
/****************************
����
****************************/
void AltHold_Mode_Ctr(_MODE_STEP *alt_hold_process)
{
   switch(*alt_hold_process)
	 {
		 //��ʼ������
		 case status_step0_none:
		 {
			 Set_Init_Par();
			 *alt_hold_process=status_step1;
		   break;
		 }
		 //��λPID
		 case status_step1:
		 {
			 Reset_All_Pid_Par();
			 *alt_hold_process=status_step2;
		 }
		 //����
		 case status_step2:
		 {
       AltHold_Process_Ctr();
			 *alt_hold_process=status_step1;//ѭ��������1
		 }
		 
		 default:break;
	   
	 }
}

/**************************
����
****************************/
void Fixed_Point_Mode_Ctr(_MODE_STEP *fixed_point_process)
{
  switch(*fixed_point_process)
	{
		//��ʼ������
	  case status_step0_none:
		{
			Set_Init_Par();
		  *fixed_point_process=status_step1;
			break;
		}
		//��λPID
		case status_step1:
		{
		  Reset_All_Pid_Par();
			*fixed_point_process=status_step2;
		}
		//����
		case status_step2:
		{
		   AltHold_Process_Ctr();
			*fixed_point_process=status_step3;
		}
		//������˲�
		case status_step3:
		{
		  if(ms8>=Pos_Contro_T_Count)//30ms
		  {
				Fixedpoint_Data_Handling();
			}
			*fixed_point_process=status_step4;

		}
		//����pid�㷨
		case status_step4:
		{
				if(ms8>=Pos_Contro_T_Count)//30ms
				{
					ms8=0;
					if((cam_pos_ctr.FixedPos_Mode==FIX_ING)&&(Plane_Mode.poshold==true)&&(struct_delay.ms_idling_ok==true)
					 &&( Static_Par.reach_start_thro==true)&&(Ultra.z>=30))//�߶ȴ���20cm��������
					{

						cascade_PosControl(tracking_offset.track_mode,&pos_pid_ctr,&cam_pos_ctr.pos_ctr_x.valid_distance,&cam_pos_ctr.pos_ctr_y.valid_distance,
														 &cam_pos_ctr.pos_ctr_x.v ,&cam_pos_ctr.pos_ctr_y.v);
					 
					}
				}
				*fixed_point_process=status_step1;//ѭ���ص���һ��	
				break;
		}
		default:break;
	}
}

/***********************************
ѭ��AB
***********************************/
void Track_Line_Process(_MODE_STEP *track_line_process)
{
  switch(*track_line_process) 
	{
		//��ʼ������
		case status_step0_none:
		{
			Set_Init_Par();
			*track_line_process=status_step1;//next
			break;			
		}
		//��λPID����
		case status_step1:
		{
		  Reset_All_Pid_Par();
			*track_line_process=status_step2;//next
		}
		//����
		case status_step2:
		{
		   AltHold_Process_Ctr();
			*track_line_process=status_step3;//next
		}
		//�����˲�
		case status_step3:
		{
				tracking_offset.Change_yaw.toward_line_mode=FINISH_TOWARD;
					//��ʼѭ������ʱ
					Start_Track_Delay();
				
					if(ms8>=Pos_Contro_T_Count)//30ms
					{
						
						//������˲�
						Fixedpoint_Data_Handling();
						//ѭ�������˲�
						Tracking_Data_Handling();
						//��תƫ��
						//Change_Yaw(60);//60ms
					
						//�ж��Ƿ��ҵ���Բ
						Is_find_circle(tracking_offset.Check.receive_find,&tracking_offset.Check.circle_find);   			
						//ѭ��ģʽת��
						Mode_Track_Convert();
				
			}
			*track_line_process=status_step4;//next
		}
		//PID
		case status_step4:
		{
		  if(ms8>=Pos_Contro_T_Count)//30ms
		  {
					ms8=0;		
					if((cam_pos_ctr.FixedPos_Mode==FIX_ING)&&(Plane_Mode.poshold==true)&&(struct_delay.ms_idling_ok==true)
					 &&( Static_Par.reach_start_thro==true)&&(Ultra.z>20))//�߶ȴ���20cm��������
					{
						
						if(struct_delay.allwait_start_track_ok==false)
						{
							if(struct_delay.ms_allwait_start_track++>200)//6s
							{
							  struct_delay.allwait_start_track_ok=true;
								struct_delay.ms_allwait_start_track=0;
								if(Ultra.z<(Ultra.FINAL_HIGH))
								{
								  	struct_delay.start_count_ms_track=true; //ѭ��ģʽ��ʱ
								}
							}
							
						}

						cascade_PosControl(tracking_offset.track_mode,&pos_pid_ctr,&cam_pos_ctr.pos_ctr_x.valid_distance,&cam_pos_ctr.pos_ctr_y.valid_distance,
														 &cam_pos_ctr.pos_ctr_x.v ,&cam_pos_ctr.pos_ctr_y.v);
					 
					}	
							
			}
			*track_line_process=status_step5;		//next
		}
		//�Զ�����
		case status_step5:
		{
			//�Զ���������������ٶ�ӵ㣬��֤�ٵڶ����㽵��
		  Auto_Land(1);
			*track_line_process=status_step1;		//ѭ���ص���һ��	
			break;
		}
		
		default:break;
	}		
}

/*********************************
ѭ��ABC
*********************************/
void Track_line_ABC(_MODE_STEP *line_ABC_process)
{
    switch(*line_ABC_process) 
	{
		//��ʼ������
		case status_step0_none:
		{
			Set_Init_Par();
			Set_AutoLand_delay(2000);
			*line_ABC_process=status_step1;//next
			break;			
		}
		//��λPID����
		case status_step1:
		{
		  Reset_All_Pid_Par();
			*line_ABC_process=status_step2;//next
		}
		//����
		case status_step2:
		{
		   AltHold_Process_Ctr();
			*line_ABC_process=status_step3;//next
		}
		//�����˲�
		case status_step3:
		{
				tracking_offset.Change_yaw.toward_line_mode=FINISH_TOWARD;
		  	//��ʼѭ������ʱ
        Start_Track_Delay();
			
				if(ms8>=12)//30ms
				{
					//������˲�
					Fixedpoint_Data_Handling();
					//ѭ�������˲�
					Tracking_Data_Handling();
					//��תƫ��
					//Change_Yaw(60);//60ms
					//�ж��Ƿ��ҵ���Բ
					Is_find_circle(tracking_offset.Check.receive_find,&tracking_offset.Check.circle_find);   			
					//ѭ��ģʽת��
					Mode_Track_Convert();
				
				}
			*line_ABC_process=status_step4;//next
		}
		//PID
		case status_step4:
		{
		  if(ms8>=12)//30ms
		  {
					ms8=0;		
					if((cam_pos_ctr.FixedPos_Mode==FIX_ING)&&(Plane_Mode.poshold==true)&&(struct_delay.ms_idling_ok==true)
					 &&( Static_Par.reach_start_thro==true)&&(Ultra.z>20))//�߶ȴ���20cm��������
					{

							if(struct_delay.allwait_start_track_ok==false)
							{
								if(struct_delay.ms_allwait_start_track++>100)//6s
								{
									struct_delay.allwait_start_track_ok=true;
									struct_delay.ms_allwait_start_track=0;
									if(Ultra.z<(Ultra.FINAL_HIGH))
									{
											struct_delay.start_count_ms_track=true; //ѭ��ģʽ��ʱ
									}
								}
								
							}
							
							cascade_PosControl(tracking_offset.track_mode,&pos_pid_ctr,&cam_pos_ctr.pos_ctr_x.valid_distance,&cam_pos_ctr.pos_ctr_y.valid_distance,
															 &cam_pos_ctr.pos_ctr_x.v ,&cam_pos_ctr.pos_ctr_y.v);
						 
					}	
							
			}
			*line_ABC_process=status_step5;	//next
		}
		//����B�����ת
		case status_step5:
		{
			if(cam_pos_ctr.FixedPos_Mode==FIX_ING)//2.5ms
			{
			  B_to_C_yaw(60);
			}
			*line_ABC_process=status_step6;//next
		}
		//�Զ�����
		case status_step6:
		{
		  Auto_Land(2);
			*line_ABC_process=status_step1;		//ѭ���ص���һ��	
			break;
		}
		default:break;
	}
}

/*********************************
ѭ������ABA
*********************************/
void Track_line_ABA(_MODE_STEP *line_ABA_process)
{
    switch(*line_ABA_process) 
	{
		//��ʼ������
		case status_step0_none:
		{
			Set_Init_Par();
			Set_AutoLand_delay(2000);
			*line_ABA_process=status_step1;//next
			break;			
		}
		//��λPID����
		case status_step1:
		{
		  Reset_All_Pid_Par();
			*line_ABA_process=status_step2;//next
		}
		//����
		case status_step2:
		{
		   AltHold_Process_Ctr();
			*line_ABA_process=status_step3;//next
		}
		//�����˲�
		case status_step3:
		{
			
		  	//��ʼѭ������ʱ
        Start_Track_Delay();
			
				if(ms8>=Pos_Contro_T_Count)//30ms
				{
					//������˲�
					Fixedpoint_Data_Handling();
					//ѭ�������˲�
					Tracking_Data_Handling();
					//�����תƫ��
					Change_Yaw(60);//60ms
					//�ж��Ƿ��ҵ���Բ
					Is_find_circle(tracking_offset.Check.receive_find,&tracking_offset.Check.circle_find);   			
					//ѭ��ģʽת��
					Mode_Track_Convert();
				
				}
			*line_ABA_process=status_step4;//next
		}
		//PID
		case status_step4:
		{
		  if(ms8>=Pos_Contro_T_Count)//30ms
		  {
					ms8=0;		
					if((cam_pos_ctr.FixedPos_Mode==FIX_ING)&&(Plane_Mode.poshold==true)&&(struct_delay.ms_idling_ok==true)
					 &&( Static_Par.reach_start_thro==true)&&(Ultra.z>20))//�߶ȴ���20cm��������
					{

						cascade_PosControl(tracking_offset.track_mode,&pos_pid_ctr,&cam_pos_ctr.pos_ctr_x.valid_distance,&cam_pos_ctr.pos_ctr_y.valid_distance,
														 &cam_pos_ctr.pos_ctr_x.v ,&cam_pos_ctr.pos_ctr_y.v);
					 
					}	
							
			}
			*line_ABA_process=status_step5;	//next
		}
		//����B����ȶ�һ��ʱ��Ȼ����˵�A��
		case status_step5:
		{
			//if(cam_pos_ctr.FixedPos_Mode==FIX_ING)//2.5ms
			{
			  Mode_ABA_OF_B_to_A();//B��A
			}
			*line_ABA_process=status_step6;//next
		}
		//�Զ�����
		case status_step6:
		{
		  Auto_Land(2);
			*line_ABA_process=status_step1;		//ѭ���ص���һ��	
			break;
		}
		default:break;
	}
}














