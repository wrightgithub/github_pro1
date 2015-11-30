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
 ��������ʼ��
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
���������ߺ���PID�㷨V2��
�ٶ����ڻ�
*************************/
float SONAR_CONTROL_V2(Ultrasound* ultra,_SONAR_PID* sonar_shell_pid,_SONAR_PID* sonar_core_pid)
{
	float shell_error=0,core_error=0;
  float pid_shell_out=0,pid_core_out=0;
	float P_OUT=0,D_OUT=0,I_OUT=0;
	//�⻷
	shell_error=sonar_shell_pid->setgoal - ultra->z; //�⻷���   //�����趨100  ���µ���error����
	shell_error=shell_error;
	sonar_shell_pid->static_inc.integral+=shell_error;  //�⻷�����ۼ�
	//sonar_shell_pid->static_inc.integral=constrain(sonar_shell_pid->static_inc.integral,-8000,8000);
	
  P_OUT =  (float)(sonar_shell_pid->P/100.00)*( shell_error);   
	I_OUT =( ((float)sonar_shell_pid->I/10000.00) * (sonar_shell_pid->static_inc.integral) );
	//I_OUT =constrain(	I_OUT,-50,50);
	sonar_shell_iout=I_OUT;//������
	
  D_OUT = (float)(sonar_shell_pid->D /100.00)*(shell_error-sonar_shell_pid->static_inc.last_error);    ///���µ��ٶ�Ϊ�������Ϸ��ٶ�Ϊ��
	pid_shell_out=(P_OUT+I_OUT);//�⻷���
	sonar_shell_out=pid_shell_out;
	
	//�ڻ�
  target_speed= pid_shell_out; //ÿ���߶Ȳ��Ӧһ�������ٶ�
	
	core_error= target_speed-ultra->ultra_v;//�ڻ���� 
	sonar_core_pid->static_inc.integral+=(0-ultra->ultra_v);//�ڻ��������⴦��
	
 	P_OUT= (float)(sonar_core_pid->P/100.00) * core_error;
	I_OUT=( (float)(sonar_core_pid->I/10000.00)*sonar_core_pid->static_inc.integral );
	//I_OUT =constrain(	I_OUT,-20,20);
	sonar_core_iout=I_OUT;//������
	
	D_OUT= (float)(sonar_core_pid->D/100.00)*(core_error-sonar_core_pid->static_inc.last_error);
	sonar_core_pid->static_inc.last_error=core_error;
	
	pid_core_out= P_OUT+I_OUT+D_OUT;  //�ڻ����
	
	pid_core_out=constrain(pid_core_out,-250,250);//����������޷�
	
	return pid_core_out;

}
/***********************************************
  ���ã� ��ͨ�˲�
  flag����Ч��־
  apex��������ֵ
  a������ֵ��ϵ��
*************************************************/
float LowPass(bool flag,float apex,float a,float nowvalue ,float lastvalue) 
{
 	float FilterValue=0;
	FilterValue= (1.00-a)* lastvalue+ a*nowvalue ;
  float height_diff = nowvalue - FilterValue;
	if ((flag == true)&&(height_diff < -apex || height_diff >apex))//������ֵ˵���µ�ֵ���ŶȺܵ�
	{						
		return FilterValue;//���������ֵ��Χ���Ե�ͨ�˲����ֵΪ׼
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
�������˲�
***********************************/

float sonar_filter(bool *_sonar_valid,Ultrasound* _ultra)
{
	#define THRESHOLD 0.4//ԭʼ����ë����ֵ
	float high;
	u8 i=0;
	float sum=0;
	static float  last_ground_distance = 0;
	static float last_filtered_high = 0;
	static float sonar_high[3];
	static u8 high_count=0;
if (*_sonar_valid == true)
	{
		//����������
		if (FL_ABS(_ultra->KS103High - last_ground_distance)>THRESHOLD)//��ʼ�߽�
		{
			last_ground_distance = _ultra->KS103High;
			return 0;

		}
		//����
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
					  sonar_high[2]= _ultra->KS103High;//֮���5����
					  for(i=0;i<3;i++)
					  {
					    sum+= sonar_high[i];
					  }
					   _ultra->NowHigh =sum/3;
					   //�ƶ�����
					   for(i=0;i<2;i++)
					   {
					     sonar_high[i]=sonar_high[i+1];
					   }
				}
			
			  last_ground_distance = _ultra->KS103High;
		}

	}

	//������������жϺ�
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
  ������Ч�����ж�
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
  ����ks103ָ��
************************************************/

void Send_KS103_Order(u8 *buf,bool wait_flag,u8 *cycle)
{
  if(wait_flag==true)//1000*5ms=5s
		{
		  /**************����2��ȡ����������******************/
		// #ifdef KS103SONAR
     if(*cycle>=20)//����ÿ50ms��һ��
		 {
				buf[0]=0xe8;
				usart2_Send_Data(buf,1);//����1����18���ֽ� 
				delay_us(50);//20~100us
				buf[0]=0x02;
				usart2_Send_Data(buf,1);//����1����18���ֽ� 
				delay_us(50);
				buf[0]=0xb4;//0~5�״��¶Ȳ���
				usart2_Send_Data(buf,1);//����1����18���ֽ� 
				delay_us(50);
			  *cycle=0;
		 }
		// #endif
		 
		}
}


/***********************************
��ȡ�������ٶ�
***********************************/
void Get_Sonar_Speed(bool _sonar_valid,Ultrasound* ultra,float T)
{
	if(_sonar_valid == true)
	{	
		ultra->ultra_v=(ultra->z-ultra->last_z)/T;	// m/s	
		ultra->last_z=ultra->z;	
	}
	ultra->ultra_v=KalmanFilter_KS103_v(ultra->ultra_v,0.2,6);  //�˵���λ����̫��  
}

/********************
����US100
********************/
void Send_US100()
{
  GPIOE->BSRR|= 1<<3;				//����������			
  delay_us(10);									
  GPIOE->BRR|= 1<<3;
}


/*********************
����Ŀ��߶�
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
		//�Զ������Ǹı�Ŀ��߶�
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
�𽥼�С����
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
��������Ƿ��ۼӵ����������
****************************/
void Is_Start_Thro(float final_high,float k,u16 add_thro,float start_high)
{
  if((Static_Par.reach_start_thro==false)&&(Plane_Mode.ARMED==true)&&(Plane_Mode.althold==true)
		 &&(struct_delay.ms_idling_ok==true)&&(!Plane_Mode.land))//����
	{
	  if(Ultra.z<=start_high)//��⵽���Ų����պ����
		{
		   Ultra.AltHoldThro+=3;
			Static_Par.reach_start_thro=false;
		}
		//�ﵽ���Ҫ��
	 if((Ultra.z>start_high)/*||FL_ABS(angle.pitch>1)||FL_ABS(angle.roll>1)*/) //�ﵽ�������
		{
			
			if(Static_Par.reach_start_thro==false)
			{
			  Ultra.AltHoldThro+=add_thro;//k*final_high;
				Static_Par.reach_start_thro=true;
			}
			
			//��ֹ����
			if( Ultra.AltHoldThro>=1460)
			{
			   Ultra.AltHoldThro=1460;
			}
//			 
//			if((Ultra.AltHoldThro<1300)&&(Static_Par.reach_start_thro==true))//��ֹ�������⣬ֻ��+add_thro����һ��
//			{
//			  Ultra.AltHoldThro=1380;
//			}
		}
	}
	
}

/****************�������ݴ���**************/
void AltHold_Data_Handling()
{
	float high=0;
	//��ȡ�˲���ĳ������߶�	//ps :ע���˲������λ��Ҫ̫�ͺ�		
	//�ж�����ԭʼ�����Ƿ���Ч
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
	   //����KS103ָ��//ÿ80ms����һ��
    Send_KS103_Order(USART2_TX_BUF,struct_delay.ms_ultra_ok,&UltraSendFlag);
	
		if(ms1>=10)//25ms 
		{
				ms1 =0;
				//�������ݴ���  //�߶��˲�
				AltHold_Data_Handling(); 		
				//��ȡ�������ٶ�
				Get_Sonar_Speed(sonar_valid,&Ultra,0.025);// cm/s			
				//������ڽ��ж���ģʽ �����Ҹ߶�����Ч�ģ�����Pid,  ��Ҫ�ȵ������
				if( (Static_Par.reach_start_thro==true)&&(Ultra.SetHigh.Mode==SetHigh_ing)&&
					 (sonar_valid==true)&&(Plane_Mode.unlock==true)&&(struct_delay.ms_idling_ok==true))
				{		
						//�ۼ������߶�
						if( (Ultra.SetHigh.Mode==SetHigh_ing)&&(Plane_Mode.unlock==true))
						{		
							Change_Goal_High( &sonar_shell_pid.setgoal,Ultra.FINAL_HIGH,1,0.6);
						}
						
						//���������ߺ���PID�㷨
						if(Ultra.z>=5)
						{
						  Ultra.SetHigh.HighPidPwm = SONAR_CONTROL_V2(&Ultra,&sonar_shell_pid,&sonar_core_pid);	//�ɲο���̬�޸�
						}
				}			
				
				//������ǰ��
				if((Ultra.z-Ultra.FINAL_HIGH)>=10 &&Ultra.have_restrain==false)
				{
						Ultra.have_restrain=true;
				   Ultra.SetHigh.HighPidPwm-=0.4*(Ultra.z-Ultra.FINAL_HIGH);
				}				
		}
		
		//����Ƿ񵽴����������
		if(ms7>=5)//12.5ms
		{
			ms7=0;
			#ifndef ATTITUDE_PID_DEBUG //���û�������̬
			Is_Start_Thro(Ultra.FINAL_HIGH,0.4,Ultra.Add_Thro,8);
			#endif
		}
}















