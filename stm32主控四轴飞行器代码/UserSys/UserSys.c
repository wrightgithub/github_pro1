#include "UserSys.h"
#include "mode.h"
_DELAY struct_delay;

/***************中位值平均滤波法***************/
//如果是小数根据精度要求*10的精度次后，出来的值在/10的精度次
unsigned long Middle( unsigned long *ArrDataBuffer)
{
	#define num 20
	 u16 j=0,k=0,l=0;
		unsigned long temp=0;
		unsigned long sum=0,Value=0;
	
	for(j=0;j<num-1;j++)//采样值由小到大排列
	{
		for(k=0;k<num-j-1;k++)
		{
			if(ArrDataBuffer[k]>ArrDataBuffer[k+1])
			{
				temp=ArrDataBuffer[k];
				ArrDataBuffer[k]=ArrDataBuffer[k+1];
				ArrDataBuffer[k+1]=temp;
			}
		}
	}
	for(l=1;l<num-1;l++)
	{
		sum+=ArrDataBuffer[l];
	}
	Value=sum/(num-2);
	
	#undef num
	return(Value);
}

/**************************
中位值平均滤波
***************************/

float UltraMiddle(volatile float *ArrDataBuffer)
{
		#define num 20
	 u8 j=0,k=0,l=0;
		float temp=0;
		float sum=0,Value=0;
	
	for(j=0;j<num-1;j++)//采样值由小到大排列
	{
		for(k=0;k<num-j-1;k++)
		{
			if(ArrDataBuffer[k]>ArrDataBuffer[k+1])
			{
				temp=ArrDataBuffer[k];
				ArrDataBuffer[k]=ArrDataBuffer[k+1];
				ArrDataBuffer[k+1]=temp;
			}
		}
	}
	for(l=1;l<num-1;l++)
	{
		sum+=ArrDataBuffer[l];
	}
	Value=sum/(num-2);
	
	#undef num
	return(Value);
}
/************浮点数与字节之间转换关系**********
     unsigned char* byteArry  是数组
**********************************************/
void FloatToByte(float floatNum,unsigned char* byteArry) //浮点数转字节   
{  
	char* pchar=(char*)&floatNum;
	for(int i=0;i<sizeof(float);i++) 
	{  
		*byteArry=*pchar; 
		pchar++;
		byteArry++; 
	} 
}

/*************************
字节转浮点数
************************/
float ByteToFloat(unsigned char* byteArry)//字节转浮点数
 { 
	return*((float*)byteArry);
 }

int ByteToInt(unsigned char* byteArry)//字节转浮点数
 { 
	return  *((int*)byteArry);
 }



/************************************************************************/
/*  Filter for motor value smoothing                                    */
/************************************************************************/
int16_t MotorSmoothing(int16_t newvalue, int16_t oldvalue)
{
        int16_t motor;
        if(newvalue > oldvalue) motor = (1 * (int16_t)oldvalue + newvalue) / 2;  //mean of old and new
        else                                          motor = newvalue - (oldvalue - newvalue) * 1; // 2 * new - old
        return(motor);
}



/******************************
延时等待函数
*******************************/
void Init_struct_delay(_DELAY *delay)
{
	delay->ms_yaw_ok   =false;
  delay->ms_ultra_ok =false;
	delay->ms_ultra =0;
	delay->ms_yaw   =0;
}

void First_Delay(_DELAY *delay)
{
//   if(delay->ms_yaw>=400&&delay->ms_yaw_ok==false)
// 	{
// 		delay->ms_yaw_ok=true;
// 		angle.heading=angle.yaw;
// 	}
	if(delay->ms_ultra>=1000)//1000
	{
		delay->ms_ultra_ok=true;
	}

}

/***************************************
  达到设定高度后的处理
****************************************/
void Receive_SetHigh(float *high,bool valid_high,AltHold sethigh_mode,float goal)
{
	//只进来一次
  if( Ultra.have_received==false && *high>=goal&&(sethigh_mode==SetHigh_ing)&&(valid_high==true))
	{
		Ultra.have_received=true;
		
		Plane_Mode.auto_liftoff=false;//达到设定高度后关闭自动起飞模式
		struct_delay.start_count_ms_land=true;  //开始降落模式的延时
		struct_delay.start_count_ms_track=true; //循迹模式延时
		tracking_offset.Change_yaw.toward_line_mode=START_TOWARD;//开始转换yaw
		LED4(ON);
	}

}





/****************************
自动降落模式判断
*****************************/
void AutoLand_Mode(float *high)
{
	
	//if(*high<=1)//这个点说明超声波出了问题
	{
		if((1<=*high)&&(*high <=25)&&(Plane_Mode.land==true))
		{
			
			//上锁
			Plane_Mode.ARMED=false;
			Plane_Mode.unlock  =false;
			
			//复位模式
			Plane_Mode.althold =false;
			Plane_Mode.poshold =false;
			Plane_Mode.reset_start=true;
			
			Plane_Mode.land=false;
			Plane_Mode.auto_liftoff=true;
			
		}

  }
}


/***************************
设定偏航角
****************************/
void Set_Heading(float *now_angle)
{
  angle.heading=*now_angle;
}

/**************************
初始化参数
***************************/
void Set_Init_Par()
{
	  Plane_Mode.auto_liftoff =true;//自动起飞模式
	  tracking_offset.forward_angle=3.0;//前进的角度
		mode_ABA.forward_angle=-3.2;
  	Ultra.Add_Thro=60;//起飞突加油门
		Ultra.FINAL_HIGH=85;//定高高度
		Ultra.AltHoldThro= Reset_thro;//记下开启定高模式时的油门
		if(Plane_Mode.auto_liftoff==true)//开启自动起飞模式
		{
			LED3(ON);
			Set_GoalHigh(Reset_high);
		  Init_SonarPid( &sonar_shell_pid,30,50,400);//25 60  ，70 25 无超调   40 60 500  60 60 200
		  Init_SonarPid( &sonar_core_pid,60,60,150);

		}
//		Init_PosPid(&pos_pid_ctr.PosPid_X.Shell ,0.15,0.002 ,0.2);
//		Init_PosPid(&pos_pid_ctr.PosPid_X.Core ,0.2,0.06 , 0.4);//0.35
//		
//		Init_PosPid(&pos_pid_ctr.PosPid_Y.Shell ,0.15,0.002 ,0.2);
//		Init_PosPid(&pos_pid_ctr.PosPid_Y.Core ,0.2,0.06 , 0.4);
		
		Init_PosPid(&pos_pid_ctr.PosPid_X.Shell ,0.15,0.002 ,0);
		Init_PosPid(&pos_pid_ctr.PosPid_X.Core ,0.2,0.05 , 0.4);//0.35
		
		Init_PosPid(&pos_pid_ctr.PosPid_Y.Shell ,0.15,0.002 ,0);
		Init_PosPid(&pos_pid_ctr.PosPid_Y.Core ,0.2,0.05 , 0.4);
		
		
		
//		
//	Init_PosPid(&pos_pid_ctr.PosPid_X.Shell, 0.25, 0.002, 0.10);
//	Init_PosPid(&pos_pid_ctr.PosPid_X.Core, 0.26, 0.06, 0.4);//0.35

//	Init_PosPid(&pos_pid_ctr.PosPid_Y.Shell, 0.25, 0.002, 0.10);
//	Init_PosPid(&pos_pid_ctr.PosPid_Y.Core, 0.26, 0.06, 0.4);
}

/************************
复位所有参数
*************************/
void Reset_All_Pid_Par()
{
	//复位参数
	Reset_Static_Par(&Plane_Mode.reset_start,PARAMETER);
	//复位姿态积分
	Reset_Static_Par(&Plane_Mode.ARMED ,INTEGRAL);
	Reset_Static_Par(&Plane_Mode.ARMED ,POS_INTEGRAL);
}

/****************************
自动降落
*****************************/
void Auto_Land(u8 circle_num )
{
		//自动降落延时
	  //第一次找到圆
		if(tracking_offset.circle_count==circle_num)
		{
				if(struct_delay.start_count_ms_land==true)
				{
					 if((struct_delay.ms_land++)>=400)//1s降落
					 {
						 Plane_Mode.land=true;//降落
						 struct_delay.start_count_ms_land=false;//关闭计数
						 struct_delay.ms_land=0;
					 }
					
				}
				if(Ultra.z<=20)
				{
					Ultra.AltHoldThro-=15;
				}
		}

}












