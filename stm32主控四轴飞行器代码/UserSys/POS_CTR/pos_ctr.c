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
	/**************外环*****************/
	error[X]=0-*x_dis;//如果往前 error为负
	Ctr->PosPid_X.Shell.integral+=error[X]; //积分为负
	Ctr->PosPid_X.Shell.I_OUT=Ctr->PosPid_X.Shell.I * Ctr->PosPid_X.Shell.integral/ Pos_Contro_fre;
	Ctr->PosPid_X.Shell.I_OUT= constrain(Ctr->PosPid_X.Shell.I_OUT,-5,5);
	         
	Ctr->PosPid_X.Shell.pid_out= Ctr->PosPid_X.Shell.P * error[X] 
                               +Ctr->PosPid_X.Shell.I_OUT
                               +Ctr->PosPid_X.Shell.D * (error[X]-last_error[X]);  //输出为负   
	last_error[X]=error[X];
	
	error[Y]=0-*y_dis;//如果往右  error为负
	Ctr->PosPid_Y.Shell.integral+=error[Y];//积分为负
	
	Ctr->PosPid_Y.Shell.I_OUT=Ctr->PosPid_Y.Shell.I * Ctr->PosPid_Y.Shell.integral/ Pos_Contro_fre;
	Ctr->PosPid_Y.Shell.I_OUT= constrain(Ctr->PosPid_Y.Shell.I_OUT,-5,5);
	
	Ctr->PosPid_Y.Shell.pid_out=Ctr->PosPid_Y.Shell.P*error[Y]
                             +Ctr->PosPid_Y.Shell.I*Ctr->PosPid_Y.Shell.integral
                             +Ctr->PosPid_Y.Shell.D*(error[Y]-last_error[Y]);  //输出为负   
	last_error[Y]=error[Y];
	
	
	/***********内环******************/
	float velocity_error[2];
	static float last_velocity_error[2]={0,0};
	velocity_error[X]=Ctr->PosPid_X.Shell.pid_out-*x_v;//往前v为正   减一减更加负
	
	 Ctr->PosPid_X.Core.integral+= velocity_error[X];//积分为负
//	if(Int_Fabs(*x_dis)>20)
//	{
//	  Ctr->PosPid_X.Core.integral+= velocity_error[X];//积分为负
//	}
//	else
//	{
//	  Ctr->PosPid_X.Core.integral=0;
//	}
	Ctr->PosPid_X.Core.I_OUT=Ctr->PosPid_X.Core.I*Ctr->PosPid_X.Core.integral/Pos_Contro_fre;
	Ctr->PosPid_X.Core.I_OUT=constrain(Ctr->PosPid_X.Core.I_OUT,-2,2);
	
  Ctr->PosPid_X.Core.pid_out=Ctr->PosPid_X.Core.P*velocity_error[X] 
	                          +Ctr->PosPid_X.Core.I_OUT
	                          +Ctr->PosPid_X.Core.D*(velocity_error[X]-last_velocity_error[X]);  //输出为负
	last_velocity_error[X]=velocity_error[X];
	
	velocity_error[Y]=Ctr->PosPid_Y.Shell.pid_out-*y_v;////往右v为正   减一减更加负
   Ctr->PosPid_Y.Core.integral+= velocity_error[Y];//积分为负
//  if(Int_Fabs(*y_dis)>20)
//	{
//	  Ctr->PosPid_Y.Core.integral+= velocity_error[Y];//积分为负
//	}
//	else
//	{
//	  Ctr->PosPid_Y.Core.integral=0;
//	}

	Ctr->PosPid_Y.Core.I_OUT=Ctr->PosPid_Y.Core.I*Ctr->PosPid_Y.Core.integral/Pos_Contro_fre;
	Ctr->PosPid_Y.Core.I_OUT=constrain(Ctr->PosPid_Y.Core.I_OUT,-2,2);
	
  Ctr->PosPid_Y.Core.pid_out=Ctr->PosPid_Y.Core.P*velocity_error[Y]
	                          +Ctr->PosPid_Y.Core.I_OUT
	                          +Ctr->PosPid_Y.Core.D*(velocity_error[Y]-last_velocity_error[Y]);//输出为负
	last_velocity_error[Y]=velocity_error[Y];
	
  #undef X
	#undef Y
	//由于内环输出为负    ///往前为负 
	if(mode==END_TRACK)
	{
	  cam_pos_ctr.PITCH =  constrain( Ctr->PosPid_X.Core.pid_out,-5,5);//PITCH越大  越往后飞，PITCH越小越往前飞
	}
	else  
	{
	  cam_pos_ctr.PITCH=  tracking_offset.goal_angle;
	}
//	if(mode!=TRACK_ING)
	{
	 cam_pos_ctr.ROLL  =   constrain(Ctr->PosPid_Y.Core.pid_out,-5,5);//ROLL越大越往右飞  ROLL越小越往左飞
	}
}




//void SinglePid(_PIDCTR* Ctr,const vs16 *const x_dis,const vs16 *const y_dis)
//{
//	#define X 0
//	#define Y 1
//  float error[2];
//	//static float last_error[2]={0,0};
//	/**************外环*****************/
//	error[X]=0-*x_dis;//如果往前 error为负
//	Ctr->PosPid_X.Shell.integral+=error[X]; //积分为负
//	Ctr->PosPid_X.Shell.I_OUT=Ctr->PosPid_X.Shell.I * Ctr->PosPid_X.Shell.integral;
//	Ctr->PosPid_X.Shell.I_OUT=constrain(Ctr->PosPid_X.Shell.I_OUT, -5,5);
//	
//	Ctr->PosPid_X.Shell.pid_out=Ctr->PosPid_X.Shell.P * error[X]   
//                             +Ctr->PosPid_X.Shell.I * Ctr->PosPid_X.Shell.integral
//                             +Ctr->PosPid_X.Shell.D * (error[X]-Ctr->PosPid_X.Shell.last_error[X]);  //输出为负   
//	Ctr->PosPid_X.Shell.last_error[X]=error[X];
//	
//	error[Y]=0-*y_dis;//如果往右  error为负
//	Ctr->PosPid_Y.Shell.integral+=error[Y];//积分为负
//	Ctr->PosPid_Y.Shell.I_OUT  =Ctr->PosPid_Y.Shell.I*Ctr->PosPid_Y.Shell.integral;
//	Ctr->PosPid_Y.Shell.I_OUT=constrain(Ctr->PosPid_Y.Shell.I_OUT, -5,5);
//	
//	Ctr->PosPid_Y.Shell.pid_out=Ctr->PosPid_Y.Shell.P*error[Y]
//                             +Ctr->PosPid_Y.Shell.I*Ctr->PosPid_Y.Shell.integral
//                             +Ctr->PosPid_Y.Shell.D*(error[Y]-Ctr->PosPid_Y.Shell.last_error[Y]);  //输出为负   
//	Ctr->PosPid_Y.Shell.last_error[Y]=error[Y];
//	
//	#undef X
//	#undef Y
//	
//	cam_pos_ctr.PITCH =  constrain( Ctr->PosPid_X.Shell.pid_out,-5,5);//PITCH越大  越往后飞，PITCH越小越往前飞
//	cam_pos_ctr.ROLL  =   constrain(Ctr->PosPid_Y.Shell.pid_out,-5,5);//ROLL越大越往右飞  ROLL越小越往左飞
//	
//}


////第二版单级PID
//void SinglePid_V2(_PIDCTR* Ctr,const volatile float*const x_dis,const volatile float*const y_dis,const volatile float*const x_v,const volatile float*const y_v)
//{
//	#define X 0
//	#define Y 1
//  float error[2];
//	float I_OUT[2];
//	/**************外环*****************/
//	error[X]=0-*x_dis;//如果往前 error为负
//	Ctr->PosPid_X.Shell.integral+=error[X]; //积分为负
//	I_OUT[X]=constrain(Ctr->PosPid_X.Shell.I * Ctr->PosPid_X.Shell.integral,-10,10);

//	
//	Ctr->PosPid_X.Shell.pid_out=Ctr->PosPid_X.Shell.P * error[X]   
//                             +I_OUT[X]
//                             -Ctr->PosPid_X.Shell.D * ( *x_v);  //输出为负   
//	
//	error[Y]=0-*y_dis;//如果往右  error为负
//	Ctr->PosPid_Y.Shell.integral+=error[Y];//积分为负
//  I_OUT[Y]=constrain(Ctr->PosPid_Y.Shell.I * Ctr->PosPid_Y.Shell.integral,-10,10);
//	
//	Ctr->PosPid_Y.Shell.pid_out=Ctr->PosPid_Y.Shell.P*error[Y]
//                             +I_OUT[Y]
//                             -Ctr->PosPid_Y.Shell.D*(*y_v);  //输出为负   
//	
//	#undef X
//	#undef Y
//	
//	cam_pos_ctr.PITCH =  constrain( Ctr->PosPid_X.Shell.pid_out,-15,15);//PITCH越大  越往后飞，PITCH越小越往前飞
//	cam_pos_ctr.ROLL  =   constrain(Ctr->PosPid_Y.Shell.pid_out,-15,15);//ROLL越大越往右飞  ROLL越小越往左飞
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
//角度补偿
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
	
	//与姿态融合
	
	cam_pos_ctr.pos_ctr_x.valid_distance =cam_pos_ctr.pos_ctr_x.distance-(0.9446*pit[4]/*( 0.4 * pit[2] + 0.6 * pit[3])*/);//0.9446
	cam_pos_ctr.pos_ctr_y.valid_distance =cam_pos_ctr.pos_ctr_y.distance-(-0.2273*roll[4]/*( 0.75 * roll[2] + 0.25 * roll[3])*/);//0.8273
}
	
//循迹时的角度补偿
float trackline_roll[8]={0};
void TrackLine_Angle_Compensate(float a_roll)
{
  for(u8 i = 7; i > 0; i--)
	{
		trackline_roll[i] = trackline_roll[i-1];	
	}
	trackline_roll[0] = a_roll;
	
	//与姿态融合
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
	/**************外环*****************/
	if(shell_count>=1)
	{
		shell_count=0;
		
		error[X]=0-*x_v;//如果往前 error为负
		Ctr->PosPid_X.Shell.integral+=error[X]; //积分为负
		Ctr->PosPid_X.Shell.I_OUT=Ctr->PosPid_X.Shell.I * Ctr->PosPid_X.Shell.integral/ Pos_Contro_fre;
		Ctr->PosPid_X.Shell.I_OUT= constrain(Ctr->PosPid_X.Shell.I_OUT,-2,2);
						 
		Ctr->PosPid_X.Shell.pid_out= Ctr->PosPid_X.Shell.P * error[X] 
																 +Ctr->PosPid_X.Shell.I_OUT
																 +Ctr->PosPid_X.Shell.D * (error[X]-last_error[X]);  //输出为负   
		last_error[X]=error[X];
		
		error[Y]=0-*y_v;//如果往右  error为负
		Ctr->PosPid_Y.Shell.integral+=error[Y];//积分为负
		
		Ctr->PosPid_Y.Shell.I_OUT=Ctr->PosPid_Y.Shell.I * Ctr->PosPid_Y.Shell.integral/ Pos_Contro_fre;
		Ctr->PosPid_Y.Shell.I_OUT= constrain(Ctr->PosPid_Y.Shell.I_OUT,-2,2);
		
		Ctr->PosPid_Y.Shell.pid_out=Ctr->PosPid_Y.Shell.P*error[Y]
															 +Ctr->PosPid_Y.Shell.I*Ctr->PosPid_Y.Shell.integral
															 +Ctr->PosPid_Y.Shell.D*(error[Y]-last_error[Y]);  //输出为负   
		last_error[Y]=error[Y];
		
	}
	/***********内环******************/
	float velocity_error[2];
	static float last_velocity_error[2]={0,0};
	velocity_error[X]=Ctr->PosPid_X.Shell.pid_out-*x_dis;//往前v为正   减一减更加负
	
	if(Int_Fabs(*x_dis)>20)
	{
	  Ctr->PosPid_X.Core.integral+= velocity_error[X];//积分为负
	}
	else
	{
	  Ctr->PosPid_X.Core.integral=0;
	}
	Ctr->PosPid_X.Core.I_OUT=Ctr->PosPid_X.Core.I*Ctr->PosPid_X.Core.integral/Pos_Contro_fre;
	Ctr->PosPid_X.Core.I_OUT=constrain(Ctr->PosPid_X.Core.I_OUT,-2,2);
	
  Ctr->PosPid_X.Core.pid_out=Ctr->PosPid_X.Core.P*velocity_error[X] 
	                          +Ctr->PosPid_X.Core.I_OUT
	                          +Ctr->PosPid_X.Core.D*(velocity_error[X]-last_velocity_error[X]);  //输出为负
	last_velocity_error[X]=velocity_error[X];
	
	velocity_error[Y]=Ctr->PosPid_Y.Shell.pid_out-*y_dis;////往右v为正   减一减更加负
  if(Int_Fabs(*y_dis)>15)
	{
	  Ctr->PosPid_Y.Core.integral+= velocity_error[Y];//积分为负
	}
	else
	{
	  Ctr->PosPid_Y.Core.integral=0;
	}

	Ctr->PosPid_Y.Core.I_OUT=Ctr->PosPid_Y.Core.I*Ctr->PosPid_Y.Core.integral/Pos_Contro_fre;
	Ctr->PosPid_Y.Core.I_OUT=constrain(Ctr->PosPid_Y.Core.I_OUT,-2,2);
	
  Ctr->PosPid_Y.Core.pid_out=Ctr->PosPid_Y.Core.P*velocity_error[Y]
	                          +Ctr->PosPid_Y.Core.I_OUT
	                          +Ctr->PosPid_Y.Core.D*(velocity_error[Y]-last_velocity_error[Y]);//输出为负
	last_velocity_error[Y]=velocity_error[Y];
	
  #undef X
	#undef Y
	//由于内环输出为负    ///往前为负 
	cam_pos_ctr.PITCH =   Ctr->PosPid_X.Core.pid_out;//PITCH越大  越往后飞，PITCH越小越往前飞
	cam_pos_ctr.ROLL  =   Ctr->PosPid_Y.Core.pid_out;//ROLL越大越往右飞  ROLL越小越往左飞
}




/*********数据处理************************/
//定点数据处理
void Fixedpoint_Data_Handling()
{
		//pixel
	cam_pos_ctr.pos_ctr_x.dis_valid=true;
	cam_pos_ctr.pos_ctr_y.dis_valid=true;
	
		// 角度补偿
	 Angle_Compensate(angle.roll, angle.pitch);

	//判断X位移是否有效
	 cam_pos_ctr.pos_ctr_x.dis_valid= Pos_Is_Value(&cam_pos_ctr.pos_ctr_x.distance,&cam_pos_ctr.pos_ctr_x.last_valid_distance,40);

	//判断Y位移是否有效
	 cam_pos_ctr.pos_ctr_y.dis_valid= Pos_Is_Value(&cam_pos_ctr.pos_ctr_y.distance,&cam_pos_ctr.pos_ctr_y.last_valid_distance,40);
	 
	//计算速度 ps: pixel/s
	
	cam_pos_ctr.pos_ctr_x.v=KalmanFilter_dis_x( Get_Pos_v(&cam_pos_ctr.pos_ctr_x.valid_distance,&cam_pos_ctr.pos_ctr_x.last_valid_distance ,V_T)
																							,0.1667);//0.1667
	cam_pos_ctr.pos_ctr_y.v=KalmanFilter_dis_y( Get_Pos_v(&cam_pos_ctr.pos_ctr_y.valid_distance,&cam_pos_ctr.pos_ctr_y.last_valid_distance ,V_T)
																							,0.1667);//0.1667

	//保存本次值
	cam_pos_ctr.pos_ctr_x.last_v=cam_pos_ctr.pos_ctr_x.v;
	cam_pos_ctr.pos_ctr_y.last_v=cam_pos_ctr.pos_ctr_y.v;
	
	cam_pos_ctr.pos_ctr_x.last_valid_distance=cam_pos_ctr.pos_ctr_x.valid_distance;
	cam_pos_ctr.pos_ctr_y.last_valid_distance=cam_pos_ctr.pos_ctr_y.valid_distance;
	
}

//循迹数据滤波
void Tracking_Data_Handling()
{
	// 角度补偿
	 TrackLine_Angle_Compensate(angle.roll);
	
	tracking_offset.off_v =Get_Pos_v(&tracking_offset.valid_offset,&tracking_offset.last_offset,0.03);
	tracking_offset.off_v =KalmanFilter_Tracking_Offset(tracking_offset.off_v,0.1,8);
	
	tracking_offset.last_offset=tracking_offset.valid_offset;
}

/**********循迹数据处理*******************/
u8 Is_find_circle(u8 par,bool* circle_find)
{
	static bool start_check=false;
	static u8 zero_par=0;
	if(par==1)//检测到一次 发现了圆标志 则继续接收连续的10次里是否都是1
	{
		start_check=true;
	}
	
	//开始检查
	if(start_check==true)
	{
		if(par==0)//如果CIRCLE_CHECK_NUM次里有一次是0 则断定没找到圆
		{
			zero_par++;
			if(zero_par>=2)//设为可调*****
			{
			  zero_par=0;
				*circle_find=false;
				tracking_offset.Check.check_count=0;
				start_check=false;
				return 0;
			}
		}
		tracking_offset.Check.check_count++;
		
		
    //如果找了CIRCLE_CHECK_NUM次  则断定找到圆
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
//	 /********定点pid控制算法********/
//		if(ms8>=Pos_Contro_T_Count)//30ms
//		{
//			ms8=0;
//			//定点
//			//定点的滤波
//      Fixedpoint_Data_Handling();
//	
//			//定点pid算法
//			if((cam_pos_ctr.FixedPos_Mode==FIX_ING)&&(Plane_Mode.poshold==true)&&(struct_delay.ms_idling_ok==true)
//				 &&( Static_Par.reach_start_thro==true)&&(Ultra.z>35))//高度大于30cm开启定点
//			{

//		    	cascade_PosControl(tracking_offset.track_mode,&pos_pid_ctr,&cam_pos_ctr.pos_ctr_x.valid_distance,&cam_pos_ctr.pos_ctr_y.valid_distance,
//				                   &cam_pos_ctr.pos_ctr_x.v ,&cam_pos_ctr.pos_ctr_y.v);
//         
//			}
//		}

//}

















