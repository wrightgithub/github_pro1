#include "Usersys.h"
#include "mode.h"
CAM_POSITION cam_pos_ctr;
_PIDCTR pos_pid_ctr;
void cascade_PosControl(Track_Mode mode ,_PIDCTR* Ctr,const vs16 *const x_dis,const vs16 *const y_dis,const volatile float*const x_v,const volatile float*const y_v)
{
	#define X 0
	#define Y 1
  float error[2];
	static float last_error[2]={0,0};
	/**************�⻷*****************/
	error[X]=0-*x_dis;//�����ǰ errorΪ��
	Ctr->PosPid_X.Shell.integral+=error[X]; //����Ϊ��
	Ctr->PosPid_X.Shell.I_OUT=Ctr->PosPid_X.Shell.I * Ctr->PosPid_X.Shell.integral/ Pos_Contro_fre;
	Ctr->PosPid_X.Shell.I_OUT= constrain(Ctr->PosPid_X.Shell.I_OUT,-5,5);
	         
	Ctr->PosPid_X.Shell.pid_out= Ctr->PosPid_X.Shell.P * error[X] 
                               +Ctr->PosPid_X.Shell.I_OUT
                               +Ctr->PosPid_X.Shell.D * (error[X]-last_error[X]);  //���Ϊ��   
	last_error[X]=error[X];
	
	error[Y]=0-*y_dis;//�������  errorΪ��
	Ctr->PosPid_Y.Shell.integral+=error[Y];//����Ϊ��
	
	Ctr->PosPid_Y.Shell.I_OUT=Ctr->PosPid_Y.Shell.I * Ctr->PosPid_Y.Shell.integral/ Pos_Contro_fre;
	Ctr->PosPid_Y.Shell.I_OUT= constrain(Ctr->PosPid_Y.Shell.I_OUT,-5,5);
	
	Ctr->PosPid_Y.Shell.pid_out=Ctr->PosPid_Y.Shell.P*error[Y]
                             +Ctr->PosPid_Y.Shell.I*Ctr->PosPid_Y.Shell.integral
                             +Ctr->PosPid_Y.Shell.D*(error[Y]-last_error[Y]);  //���Ϊ��   
	last_error[Y]=error[Y];
	
	
	/***********�ڻ�******************/
	float velocity_error[2];
	static float last_velocity_error[2]={0,0};
	velocity_error[X]=Ctr->PosPid_X.Shell.pid_out-*x_v;//��ǰvΪ��   ��һ�����Ӹ�
	
	 Ctr->PosPid_X.Core.integral+= velocity_error[X];//����Ϊ��
//	if(Int_Fabs(*x_dis)>20)
//	{
//	  Ctr->PosPid_X.Core.integral+= velocity_error[X];//����Ϊ��
//	}
//	else
//	{
//	  Ctr->PosPid_X.Core.integral=0;
//	}
	Ctr->PosPid_X.Core.I_OUT=Ctr->PosPid_X.Core.I*Ctr->PosPid_X.Core.integral/Pos_Contro_fre;
	Ctr->PosPid_X.Core.I_OUT=constrain(Ctr->PosPid_X.Core.I_OUT,-2,2);
	
  Ctr->PosPid_X.Core.pid_out=Ctr->PosPid_X.Core.P*velocity_error[X] 
	                          +Ctr->PosPid_X.Core.I_OUT
	                          +Ctr->PosPid_X.Core.D*(velocity_error[X]-last_velocity_error[X]);  //���Ϊ��
	last_velocity_error[X]=velocity_error[X];
	
	velocity_error[Y]=Ctr->PosPid_Y.Shell.pid_out-*y_v;////����vΪ��   ��һ�����Ӹ�
   Ctr->PosPid_Y.Core.integral+= velocity_error[Y];//����Ϊ��
//  if(Int_Fabs(*y_dis)>20)
//	{
//	  Ctr->PosPid_Y.Core.integral+= velocity_error[Y];//����Ϊ��
//	}
//	else
//	{
//	  Ctr->PosPid_Y.Core.integral=0;
//	}

	Ctr->PosPid_Y.Core.I_OUT=Ctr->PosPid_Y.Core.I*Ctr->PosPid_Y.Core.integral/Pos_Contro_fre;
	Ctr->PosPid_Y.Core.I_OUT=constrain(Ctr->PosPid_Y.Core.I_OUT,-2,2);
	
  Ctr->PosPid_Y.Core.pid_out=Ctr->PosPid_Y.Core.P*velocity_error[Y]
	                          +Ctr->PosPid_Y.Core.I_OUT
	                          +Ctr->PosPid_Y.Core.D*(velocity_error[Y]-last_velocity_error[Y]);//���Ϊ��
	last_velocity_error[Y]=velocity_error[Y];
	
  #undef X
	#undef Y
	//�����ڻ����Ϊ��    ///��ǰΪ�� 
	if(mode==END_TRACK)
	{
	  cam_pos_ctr.PITCH =  constrain( Ctr->PosPid_X.Core.pid_out,-5,5);//PITCHԽ��  Խ����ɣ�PITCHԽСԽ��ǰ��
	}
	else  
	{
	  cam_pos_ctr.PITCH=  tracking_offset.goal_angle;
	}
//	if(mode!=TRACK_ING)
	{
	 cam_pos_ctr.ROLL  =   constrain(Ctr->PosPid_Y.Core.pid_out,-5,5);//ROLLԽ��Խ���ҷ�  ROLLԽСԽ�����
	}
}




//void SinglePid(_PIDCTR* Ctr,const vs16 *const x_dis,const vs16 *const y_dis)
//{
//	#define X 0
//	#define Y 1
//  float error[2];
//	//static float last_error[2]={0,0};
//	/**************�⻷*****************/
//	error[X]=0-*x_dis;//�����ǰ errorΪ��
//	Ctr->PosPid_X.Shell.integral+=error[X]; //����Ϊ��
//	Ctr->PosPid_X.Shell.I_OUT=Ctr->PosPid_X.Shell.I * Ctr->PosPid_X.Shell.integral;
//	Ctr->PosPid_X.Shell.I_OUT=constrain(Ctr->PosPid_X.Shell.I_OUT, -5,5);
//	
//	Ctr->PosPid_X.Shell.pid_out=Ctr->PosPid_X.Shell.P * error[X]   
//                             +Ctr->PosPid_X.Shell.I * Ctr->PosPid_X.Shell.integral
//                             +Ctr->PosPid_X.Shell.D * (error[X]-Ctr->PosPid_X.Shell.last_error[X]);  //���Ϊ��   
//	Ctr->PosPid_X.Shell.last_error[X]=error[X];
//	
//	error[Y]=0-*y_dis;//�������  errorΪ��
//	Ctr->PosPid_Y.Shell.integral+=error[Y];//����Ϊ��
//	Ctr->PosPid_Y.Shell.I_OUT  =Ctr->PosPid_Y.Shell.I*Ctr->PosPid_Y.Shell.integral;
//	Ctr->PosPid_Y.Shell.I_OUT=constrain(Ctr->PosPid_Y.Shell.I_OUT, -5,5);
//	
//	Ctr->PosPid_Y.Shell.pid_out=Ctr->PosPid_Y.Shell.P*error[Y]
//                             +Ctr->PosPid_Y.Shell.I*Ctr->PosPid_Y.Shell.integral
//                             +Ctr->PosPid_Y.Shell.D*(error[Y]-Ctr->PosPid_Y.Shell.last_error[Y]);  //���Ϊ��   
//	Ctr->PosPid_Y.Shell.last_error[Y]=error[Y];
//	
//	#undef X
//	#undef Y
//	
//	cam_pos_ctr.PITCH =  constrain( Ctr->PosPid_X.Shell.pid_out,-5,5);//PITCHԽ��  Խ����ɣ�PITCHԽСԽ��ǰ��
//	cam_pos_ctr.ROLL  =   constrain(Ctr->PosPid_Y.Shell.pid_out,-5,5);//ROLLԽ��Խ���ҷ�  ROLLԽСԽ�����
//	
//}


////�ڶ��浥��PID
//void SinglePid_V2(_PIDCTR* Ctr,const volatile float*const x_dis,const volatile float*const y_dis,const volatile float*const x_v,const volatile float*const y_v)
//{
//	#define X 0
//	#define Y 1
//  float error[2];
//	float I_OUT[2];
//	/**************�⻷*****************/
//	error[X]=0-*x_dis;//�����ǰ errorΪ��
//	Ctr->PosPid_X.Shell.integral+=error[X]; //����Ϊ��
//	I_OUT[X]=constrain(Ctr->PosPid_X.Shell.I * Ctr->PosPid_X.Shell.integral,-10,10);

//	
//	Ctr->PosPid_X.Shell.pid_out=Ctr->PosPid_X.Shell.P * error[X]   
//                             +I_OUT[X]
//                             -Ctr->PosPid_X.Shell.D * ( *x_v);  //���Ϊ��   
//	
//	error[Y]=0-*y_dis;//�������  errorΪ��
//	Ctr->PosPid_Y.Shell.integral+=error[Y];//����Ϊ��
//  I_OUT[Y]=constrain(Ctr->PosPid_Y.Shell.I * Ctr->PosPid_Y.Shell.integral,-10,10);
//	
//	Ctr->PosPid_Y.Shell.pid_out=Ctr->PosPid_Y.Shell.P*error[Y]
//                             +I_OUT[Y]
//                             -Ctr->PosPid_Y.Shell.D*(*y_v);  //���Ϊ��   
//	
//	#undef X
//	#undef Y
//	
//	cam_pos_ctr.PITCH =  constrain( Ctr->PosPid_X.Shell.pid_out,-15,15);//PITCHԽ��  Խ����ɣ�PITCHԽСԽ��ǰ��
//	cam_pos_ctr.ROLL  =   constrain(Ctr->PosPid_Y.Shell.pid_out,-15,15);//ROLLԽ��Խ���ҷ�  ROLLԽСԽ�����
//	
//}

float Pos_Ctr_LowPass(volatile float now_x,volatile float last_x ,float a)
{
  return  last_x *(1-a)+now_x *a;
}


float Get_Pos_v(vs16 * now_pos,vs16 * last_pos,float dt)
{
  return (*now_pos-*last_pos)/dt;  //  pixel/s
}


void Init_PosPid(_POSPID *pid ,float P ,float I ,float D)
{
  pid->P=P;
	pid->I=I;
	pid->D=D;
}	



bool Pos_Is_Value(vs16 *distance,vs16 *last_distance,int top_value)
{
	//bool _result=false;
	
	if( FL_ABS(*distance-*last_distance)>top_value )
	{
		*distance=*last_distance;
	  //_result=false;
	}
//	else
//	{
//	 // _result=true;
//	}
//	
	//*last_distance=*distance;
	return true;
	//return _result;
}

int Is_Exceed_MaxValue(const volatile float *value,float max,float min)
{
	if(*value>=max)
	{
	  return max;
	}
	else if(*value<=min)
	{
	  return min;
	}
	else
	{
	  return *value;
	}


}

 float pit[6]={0};
 float roll[6]={0};
//�ǶȲ���
void Angle_Compensate(float a_roll, float a_pit)
{
//	static float pit[5]={0};
//	static float roll[5]={0};
  for(u8 i = 5; i > 0; i--)
	{
		pit[i]  = pit[i-1];
		roll[i] = roll[i-1];	
	}
	pit[0]  = a_pit;
	roll[0] = a_roll;
	
	//����̬�ں�
	
	cam_pos_ctr.pos_ctr_x.valid_distance =cam_pos_ctr.pos_ctr_x.distance-(0.9446*pit[4]/*( 0.4 * pit[2] + 0.6 * pit[3])*/);//0.9446
	cam_pos_ctr.pos_ctr_y.valid_distance =cam_pos_ctr.pos_ctr_y.distance-(-0.2273*roll[4]/*( 0.75 * roll[2] + 0.25 * roll[3])*/);//0.8273
}
	
//ѭ��ʱ�ĽǶȲ���
float trackline_roll[8]={0};
void TrackLine_Angle_Compensate(float a_roll)
{
  for(u8 i = 7; i > 0; i--)
	{
		trackline_roll[i] = trackline_roll[i-1];	
	}
	trackline_roll[0] = a_roll;
	
	//����̬�ں�
		tracking_offset.valid_offset =tracking_offset.offset -(-0.8273*trackline_roll[5]/*( 0.75 * roll[2] + 0.25 * roll[3])*/);
}



void cascade_PosControl_V2(_PIDCTR* Ctr,const vs16 *const x_dis,const vs16 *const y_dis,const volatile float*const x_v,const volatile float*const y_v)
{
	#define X 0
	#define Y 1
  float error[2];
	static float last_error[2]={0,0};
	static u8 shell_count=0;
	shell_count++;
	/**************�⻷*****************/
	if(shell_count>=1)
	{
		shell_count=0;
		
		error[X]=0-*x_v;//�����ǰ errorΪ��
		Ctr->PosPid_X.Shell.integral+=error[X]; //����Ϊ��
		Ctr->PosPid_X.Shell.I_OUT=Ctr->PosPid_X.Shell.I * Ctr->PosPid_X.Shell.integral/ Pos_Contro_fre;
		Ctr->PosPid_X.Shell.I_OUT= constrain(Ctr->PosPid_X.Shell.I_OUT,-2,2);
						 
		Ctr->PosPid_X.Shell.pid_out= Ctr->PosPid_X.Shell.P * error[X] 
																 +Ctr->PosPid_X.Shell.I_OUT
																 +Ctr->PosPid_X.Shell.D * (error[X]-last_error[X]);  //���Ϊ��   
		last_error[X]=error[X];
		
		error[Y]=0-*y_v;//�������  errorΪ��
		Ctr->PosPid_Y.Shell.integral+=error[Y];//����Ϊ��
		
		Ctr->PosPid_Y.Shell.I_OUT=Ctr->PosPid_Y.Shell.I * Ctr->PosPid_Y.Shell.integral/ Pos_Contro_fre;
		Ctr->PosPid_Y.Shell.I_OUT= constrain(Ctr->PosPid_Y.Shell.I_OUT,-2,2);
		
		Ctr->PosPid_Y.Shell.pid_out=Ctr->PosPid_Y.Shell.P*error[Y]
															 +Ctr->PosPid_Y.Shell.I*Ctr->PosPid_Y.Shell.integral
															 +Ctr->PosPid_Y.Shell.D*(error[Y]-last_error[Y]);  //���Ϊ��   
		last_error[Y]=error[Y];
		
	}
	/***********�ڻ�******************/
	float velocity_error[2];
	static float last_velocity_error[2]={0,0};
	velocity_error[X]=Ctr->PosPid_X.Shell.pid_out-*x_dis;//��ǰvΪ��   ��һ�����Ӹ�
	
	if(Int_Fabs(*x_dis)>20)
	{
	  Ctr->PosPid_X.Core.integral+= velocity_error[X];//����Ϊ��
	}
	else
	{
	  Ctr->PosPid_X.Core.integral=0;
	}
	Ctr->PosPid_X.Core.I_OUT=Ctr->PosPid_X.Core.I*Ctr->PosPid_X.Core.integral/Pos_Contro_fre;
	Ctr->PosPid_X.Core.I_OUT=constrain(Ctr->PosPid_X.Core.I_OUT,-2,2);
	
  Ctr->PosPid_X.Core.pid_out=Ctr->PosPid_X.Core.P*velocity_error[X] 
	                          +Ctr->PosPid_X.Core.I_OUT
	                          +Ctr->PosPid_X.Core.D*(velocity_error[X]-last_velocity_error[X]);  //���Ϊ��
	last_velocity_error[X]=velocity_error[X];
	
	velocity_error[Y]=Ctr->PosPid_Y.Shell.pid_out-*y_dis;////����vΪ��   ��һ�����Ӹ�
  if(Int_Fabs(*y_dis)>15)
	{
	  Ctr->PosPid_Y.Core.integral+= velocity_error[Y];//����Ϊ��
	}
	else
	{
	  Ctr->PosPid_Y.Core.integral=0;
	}

	Ctr->PosPid_Y.Core.I_OUT=Ctr->PosPid_Y.Core.I*Ctr->PosPid_Y.Core.integral/Pos_Contro_fre;
	Ctr->PosPid_Y.Core.I_OUT=constrain(Ctr->PosPid_Y.Core.I_OUT,-2,2);
	
  Ctr->PosPid_Y.Core.pid_out=Ctr->PosPid_Y.Core.P*velocity_error[Y]
	                          +Ctr->PosPid_Y.Core.I_OUT
	                          +Ctr->PosPid_Y.Core.D*(velocity_error[Y]-last_velocity_error[Y]);//���Ϊ��
	last_velocity_error[Y]=velocity_error[Y];
	
  #undef X
	#undef Y
	//�����ڻ����Ϊ��    ///��ǰΪ�� 
	cam_pos_ctr.PITCH =   Ctr->PosPid_X.Core.pid_out;//PITCHԽ��  Խ����ɣ�PITCHԽСԽ��ǰ��
	cam_pos_ctr.ROLL  =   Ctr->PosPid_Y.Core.pid_out;//ROLLԽ��Խ���ҷ�  ROLLԽСԽ�����
}




/*********���ݴ���************************/
//�������ݴ���
void Fixedpoint_Data_Handling()
{
		//pixel
	cam_pos_ctr.pos_ctr_x.dis_valid=true;
	cam_pos_ctr.pos_ctr_y.dis_valid=true;
	
		// �ǶȲ���
	 Angle_Compensate(angle.roll, angle.pitch);

	//�ж�Xλ���Ƿ���Ч
	 cam_pos_ctr.pos_ctr_x.dis_valid= Pos_Is_Value(&cam_pos_ctr.pos_ctr_x.distance,&cam_pos_ctr.pos_ctr_x.last_valid_distance,40);

	//�ж�Yλ���Ƿ���Ч
	 cam_pos_ctr.pos_ctr_y.dis_valid= Pos_Is_Value(&cam_pos_ctr.pos_ctr_y.distance,&cam_pos_ctr.pos_ctr_y.last_valid_distance,40);
	 
	//�����ٶ� ps: pixel/s
	
	cam_pos_ctr.pos_ctr_x.v=KalmanFilter_dis_x( Get_Pos_v(&cam_pos_ctr.pos_ctr_x.valid_distance,&cam_pos_ctr.pos_ctr_x.last_valid_distance ,V_T)
																							,0.1667);//0.1667
	cam_pos_ctr.pos_ctr_y.v=KalmanFilter_dis_y( Get_Pos_v(&cam_pos_ctr.pos_ctr_y.valid_distance,&cam_pos_ctr.pos_ctr_y.last_valid_distance ,V_T)
																							,0.1667);//0.1667

	//���汾��ֵ
	cam_pos_ctr.pos_ctr_x.last_v=cam_pos_ctr.pos_ctr_x.v;
	cam_pos_ctr.pos_ctr_y.last_v=cam_pos_ctr.pos_ctr_y.v;
	
	cam_pos_ctr.pos_ctr_x.last_valid_distance=cam_pos_ctr.pos_ctr_x.valid_distance;
	cam_pos_ctr.pos_ctr_y.last_valid_distance=cam_pos_ctr.pos_ctr_y.valid_distance;
	
}

//ѭ�������˲�
void Tracking_Data_Handling()
{
	// �ǶȲ���
	 TrackLine_Angle_Compensate(angle.roll);
	
	tracking_offset.off_v =Get_Pos_v(&tracking_offset.valid_offset,&tracking_offset.last_offset,0.03);
	tracking_offset.off_v =KalmanFilter_Tracking_Offset(tracking_offset.off_v,0.1,8);
	
	tracking_offset.last_offset=tracking_offset.valid_offset;
}

/**********ѭ�����ݴ���*******************/
u8 Is_find_circle(u8 par,bool* circle_find)
{
	static bool start_check=false;
	static u8 zero_par=0;
	if(par==1)//��⵽һ�� ������Բ��־ ���������������10�����Ƿ���1
	{
		start_check=true;
	}
	
	//��ʼ���
	if(start_check==true)
	{
		if(par==0)//���CIRCLE_CHECK_NUM������һ����0 ��϶�û�ҵ�Բ
		{
			zero_par++;
			if(zero_par>=2)//��Ϊ�ɵ�*****
			{
			  zero_par=0;
				*circle_find=false;
				tracking_offset.Check.check_count=0;
				start_check=false;
				return 0;
			}
		}
		tracking_offset.Check.check_count++;
		
		
    //�������CIRCLE_CHECK_NUM��  ��϶��ҵ�Բ
		if(tracking_offset.Check.check_count>=CIRCLE_CHECK_NUM)
		{
			* circle_find=true;
		  tracking_offset.Check.check_count=0;
			start_check=false;
			return 1;
		}
		
	}
	
	return 0;
}



//void Fixed_Point_Process_Ctr(void)
//{
//	 /********����pid�����㷨********/
//		if(ms8>=Pos_Contro_T_Count)//30ms
//		{
//			ms8=0;
//			//����
//			//������˲�
//      Fixedpoint_Data_Handling();
//	
//			//����pid�㷨
//			if((cam_pos_ctr.FixedPos_Mode==FIX_ING)&&(Plane_Mode.poshold==true)&&(struct_delay.ms_idling_ok==true)
//				 &&( Static_Par.reach_start_thro==true)&&(Ultra.z>35))//�߶ȴ���30cm��������
//			{

//		    	cascade_PosControl(tracking_offset.track_mode,&pos_pid_ctr,&cam_pos_ctr.pos_ctr_x.valid_distance,&cam_pos_ctr.pos_ctr_y.valid_distance,
//				                   &cam_pos_ctr.pos_ctr_x.v ,&cam_pos_ctr.pos_ctr_y.v);
//         
//			}
//		}

//}

















