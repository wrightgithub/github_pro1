#include "sonar_ctr.h"
#include "math.h"
#include "mode.h"
 float target_speed=0;
 int sonar_shell_iout=0,sonar_core_iout=0;
 int sonar_shell_out=0;
//_SONAR_PID sonar_pid;

_SONAR_PID sonar_shell_pid;
_SONAR_PID sonar_core_pid;

Ultrasound Ultra;

/****************************
 超声波初始化
 ***************************/
void Init_SonarPid( _SONAR_PID *pid,int16_t p,int16_t i,int16_t d)
{
	pid->P=p;
	pid->I=i;
	pid->D=d;
}
/****************************

*****************************/
void Set_GoalHigh(float high)
{
 sonar_shell_pid.setgoal=high;
}


/*************************
超声波定高核心PID算法V2版
速度做内环
*************************/
float SONAR_CONTROL_V2(Ultrasound* ultra,_SONAR_PID* sonar_shell_pid,_SONAR_PID* sonar_core_pid)
{
	float shell_error=0,core_error=0;
  float pid_shell_out=0,pid_core_out=0;
	float P_OUT=0,D_OUT=0,I_OUT=0;
	//外环
	shell_error=sonar_shell_pid->setgoal - ultra->z; //外环误差   //假如设定100  往下掉则error增大
	shell_error=shell_error;
	sonar_shell_pid->static_inc.integral+=shell_error;  //外环积分累加
	//sonar_shell_pid->static_inc.integral=constrain(sonar_shell_pid->static_inc.integral,-8000,8000);
	
  P_OUT =  (float)(sonar_shell_pid->P/100.00)*( shell_error);   
	I_OUT =( ((float)sonar_shell_pid->I/10000.00) * (sonar_shell_pid->static_inc.integral) );
	//I_OUT =constrain(	I_OUT,-50,50);
	sonar_shell_iout=I_OUT;//监测积分
	
  D_OUT = (float)(sonar_shell_pid->D /100.00)*(shell_error-sonar_shell_pid->static_inc.last_error);    ///往下掉速度为负，往上飞速度为正
	pid_shell_out=(P_OUT+I_OUT);//外环输出
	sonar_shell_out=pid_shell_out;
	
	//内环
  target_speed= pid_shell_out; //每个高度差对应一个期望速度
	
	core_error= target_speed-ultra->ultra_v;//内环误差 
	sonar_core_pid->static_inc.integral+=(0-ultra->ultra_v);//内环积分特殊处理
	
 	P_OUT= (float)(sonar_core_pid->P/100.00) * core_error;
	I_OUT=( (float)(sonar_core_pid->I/10000.00)*sonar_core_pid->static_inc.integral );
	//I_OUT =constrain(	I_OUT,-20,20);
	sonar_core_iout=I_OUT;//监测积分
	
	D_OUT= (float)(sonar_core_pid->D/100.00)*(core_error-sonar_core_pid->static_inc.last_error);
	sonar_core_pid->static_inc.last_error=core_error;
	
	pid_core_out= P_OUT+I_OUT+D_OUT;  //内环输出
	
	pid_core_out=constrain(pid_core_out,-250,250);//输出油门量限幅
	
	return pid_core_out;

}
/***********************************************
  作用： 低通滤波
  flag：有效标志
  apex：误差最大值
  a：本次值得系数
*************************************************/
float LowPass(bool flag,float apex,float a,float nowvalue ,float lastvalue) 
{
 	float FilterValue=0;
	FilterValue= (1.00-a)* lastvalue+ a*nowvalue ;
  float height_diff = nowvalue - FilterValue;
	if ((flag == true)&&(height_diff < -apex || height_diff >apex))//超出阈值说明新的值可信度很低
	{						
		return FilterValue;//如果超出阈值范围则以低通滤波后的值为准
	}
	else
	{
		return  nowvalue;
	}
}

float max(float a,float b)
{
 if(a>=b)
	 return b;
 else
	 return a;
}

/**********************************
超生波滤波
***********************************/

float sonar_filter(bool *_sonar_valid,Ultrasound* _ultra)
{
	#define THRESHOLD 0.4//原始数据毛刺阈值
	float high;
	u8 i=0;
	float sum=0;
	static float  last_ground_distance = 0;
	static float last_filtered_high = 0;
	static float sonar_high[3];
	static u8 high_count=0;
if (*_sonar_valid == true)
	{
		//非正常跳变
		if (FL_ABS(_ultra->KS103High - last_ground_distance)>THRESHOLD)//起始边界
		{
			last_ground_distance = _ultra->KS103High;
			return 0;

		}
		//正常
		else
		{
			//	*_sonar_valid = true;

				if(high_count<3)
				{
						sonar_high[high_count]= _ultra->KS103High;
						high_count++;
				}
				else if(high_count>=3)
				{
					  high_count=3;
					  sonar_high[2]= _ultra->KS103High;//之后从5插入
					  for(i=0;i<3;i++)
					  {
					    sum+= sonar_high[i];
					  }
					   _ultra->NowHigh =sum/3;
					   //移动数据
					   for(i=0;i<2;i++)
					   {
					     sonar_high[i]=sonar_high[i+1];
					   }
				}
			
			  last_ground_distance = _ultra->KS103High;
		}

	}

	//进行完上面的判断后
	if (*_sonar_valid == true)
	{
		//if(Ultra.have_received==true)
		{
		  high = LowPass(true, 0.3, 0.05, _ultra->NowHigh, last_filtered_high);
		  last_filtered_high = high;
		}
	}

	return high;
}

/************************
  声纳有效与否的判断
***************************/
bool Sonar_Valid_Check( bool  _sonar_valid,Ultrasound* ultra)
{
			if(ultra->KS103High<0.01||ultra->KS103High>3)//
				_sonar_valid=false;
			else
				_sonar_valid=true;
			
			return _sonar_valid;
}


/*******************************************
  发送ks103指令
************************************************/

void Send_KS103_Order(u8 *buf,bool wait_flag,u8 *cycle)
{
  if(wait_flag==true)//1000*5ms=5s
		{
		  /**************串口2读取超声波数据******************/
		// #ifdef KS103SONAR
     if(*cycle>=20)//数据每50ms发一次
		 {
				buf[0]=0xe8;
				usart2_Send_Data(buf,1);//串口1发送18个字节 
				delay_us(50);//20~100us
				buf[0]=0x02;
				usart2_Send_Data(buf,1);//串口1发送18个字节 
				delay_us(50);
				buf[0]=0xb4;//0~5米带温度补偿
				usart2_Send_Data(buf,1);//串口1发送18个字节 
				delay_us(50);
			  *cycle=0;
		 }
		// #endif
		 
		}
}


/***********************************
获取超声波速度
***********************************/
void Get_Sonar_Speed(bool _sonar_valid,Ultrasound* ultra,float T)
{
	if(_sonar_valid == true)
	{	
		ultra->ultra_v=(ultra->z-ultra->last_z)/T;	// m/s	
		ultra->last_z=ultra->z;	
	}
	ultra->ultra_v=KalmanFilter_KS103_v(ultra->ultra_v,0.2,6);  //滤到相位相差别太大  
}

/********************
发射US100
********************/
void Send_US100()
{
  GPIOE->BSRR|= 1<<3;				//超声波发射			
  delay_us(10);									
  GPIOE->BRR|= 1<<3;
}


/*********************
增加目标高度
*********************/
void Change_Goal_High( float *goalhigh,float final_high,u8 fre,float land_k)
{
	Static_Par.goal_high_fre++;
	if(Static_Par.goal_high_fre>=fre)
	{
		Static_Par.goal_high_fre=0;
		if(*goalhigh>=final_high)
		{
			Static_Par.have_reached=true;
			*goalhigh=final_high;
		}
		if(Static_Par.have_reached==false)
		{
			(*goalhigh)+=(final_high-*goalhigh)/30;
		}
		//自动降落是改变目标高度
		if(Plane_Mode.land ==true)
		{
		//	(*goalhigh)-=(final_high-*goalhigh)/10;
			*(goalhigh)-=10;
			if((*goalhigh)<=-land_k*final_high)
			{
				(*goalhigh)=-land_k*final_high;
			}
		}
  }

}


/****************
逐渐减小积分
******************/
void Reduce_shell_I( int16_t*  shell_I ,int16_t final_shell_I)
{
	if(*shell_I > final_shell_I)
	{
    (*shell_I)--;
	}
	else
	{
	 *shell_I=final_shell_I; 
	}
	
}

/***************************
检测油门是否累加到了起飞油门
****************************/
void Is_Start_Thro(float final_high,float k,u16 add_thro,float start_high)
{
  if((Static_Par.reach_start_thro==false)&&(Plane_Mode.ARMED==true)&&(Plane_Mode.althold==true)
		 &&(struct_delay.ms_idling_ok==true)&&(!Plane_Mode.land))//解锁
	{
	  if(Ultra.z<=start_high)//检测到油门不够刚好起飞
		{
		   Ultra.AltHoldThro+=3;
			Static_Par.reach_start_thro=false;
		}
		//达到起飞要求
	 if((Ultra.z>start_high)/*||FL_ABS(angle.pitch>1)||FL_ABS(angle.roll>1)*/) //达到起飞油门
		{
			
			if(Static_Par.reach_start_thro==false)
			{
			  Ultra.AltHoldThro+=add_thro;//k*final_high;
				Static_Par.reach_start_thro=true;
			}
			
			//防止过冲
			if( Ultra.AltHoldThro>=1460)
			{
			   Ultra.AltHoldThro=1460;
			}
//			 
//			if((Ultra.AltHoldThro<1300)&&(Static_Par.reach_start_thro==true))//防止出现意外，只靠+add_thro起来一点
//			{
//			  Ultra.AltHoldThro=1380;
//			}
		}
	}
	
}

/****************定高数据处理**************/
void AltHold_Data_Handling()
{
	float high=0;
	//获取滤波后的超声波高度	//ps :注意滤波后的相位不要太滞后		
	//判断声纳原始数据是否有效
	 sonar_valid=Sonar_Valid_Check(sonar_valid,&Ultra);  
	 high=sonar_filter(&sonar_valid,&Ultra)*100;//cm 
	 if(high==0)
	 {
	    sonar_valid=false;
	 }
	 else if(sonar_valid==true)
	 {
	   Ultra.z=high;
	 }


}



void AltHold_Process_Ctr()
{
	   //发送KS103指令//每80ms发送一次
    Send_KS103_Order(USART2_TX_BUF,struct_delay.ms_ultra_ok,&UltraSendFlag);
	
		if(ms1>=10)//25ms 
		{
				ms1 =0;
				//定高数据处理  //高度滤波
				AltHold_Data_Handling(); 		
				//获取超声波速度
				Get_Sonar_Speed(sonar_valid,&Ultra,0.025);// cm/s			
				//如果正在进行定高模式 ，并且高度是有效的，则开启Pid,  需要等怠速完成
				if( (Static_Par.reach_start_thro==true)&&(Ultra.SetHigh.Mode==SetHigh_ing)&&
					 (sonar_valid==true)&&(Plane_Mode.unlock==true)&&(struct_delay.ms_idling_ok==true))
				{		
						//累加期望高度
						if( (Ultra.SetHigh.Mode==SetHigh_ing)&&(Plane_Mode.unlock==true))
						{		
							Change_Goal_High( &sonar_shell_pid.setgoal,Ultra.FINAL_HIGH,1,0.6);
						}
						
						//超声波定高核心PID算法
						if(Ultra.z>=5)
						{
						  Ultra.SetHigh.HighPidPwm = SONAR_CONTROL_V2(&Ultra,&sonar_shell_pid,&sonar_core_pid);	//可参考姿态修改
						}
				}			
				
				//超调提前减
				if((Ultra.z-Ultra.FINAL_HIGH)>=10 &&Ultra.have_restrain==false)
				{
						Ultra.have_restrain=true;
				   Ultra.SetHigh.HighPidPwm-=0.4*(Ultra.z-Ultra.FINAL_HIGH);
				}				
		}
		
		//检测是否到达了起飞油门
		if(ms7>=5)//12.5ms
		{
			ms7=0;
			#ifndef ATTITUDE_PID_DEBUG //如果没定义调姿态
			Is_Start_Thro(Ultra.FINAL_HIGH,0.4,Ultra.Add_Thro,8);
			#endif
		}
}















