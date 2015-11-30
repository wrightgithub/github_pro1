#include "UserSys.h"
#include "mode.h"
_DELAY struct_delay;

/***************��λֵƽ���˲���***************/
//�����С�����ݾ���Ҫ��*10�ľ��ȴκ󣬳�����ֵ��/10�ľ��ȴ�
unsigned long Middle( unsigned long *ArrDataBuffer)
{
	#define num 20
	 u16 j=0,k=0,l=0;
		unsigned long temp=0;
		unsigned long sum=0,Value=0;
	
	for(j=0;j<num-1;j++)//����ֵ��С��������
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
��λֵƽ���˲�
***************************/

float UltraMiddle(volatile float *ArrDataBuffer)
{
		#define num 20
	 u8 j=0,k=0,l=0;
		float temp=0;
		float sum=0,Value=0;
	
	for(j=0;j<num-1;j++)//����ֵ��С��������
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
/************���������ֽ�֮��ת����ϵ**********
     unsigned char* byteArry  ������
**********************************************/
void FloatToByte(float floatNum,unsigned char* byteArry) //������ת�ֽ�   
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
�ֽ�ת������
************************/
float ByteToFloat(unsigned char* byteArry)//�ֽ�ת������
 { 
	return*((float*)byteArry);
 }

int ByteToInt(unsigned char* byteArry)//�ֽ�ת������
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
��ʱ�ȴ�����
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
  �ﵽ�趨�߶Ⱥ�Ĵ���
****************************************/
void Receive_SetHigh(float *high,bool valid_high,AltHold sethigh_mode,float goal)
{
	//ֻ����һ��
  if( Ultra.have_received==false && *high>=goal&&(sethigh_mode==SetHigh_ing)&&(valid_high==true))
	{
		Ultra.have_received=true;
		
		Plane_Mode.auto_liftoff=false;//�ﵽ�趨�߶Ⱥ�ر��Զ����ģʽ
		struct_delay.start_count_ms_land=true;  //��ʼ����ģʽ����ʱ
		struct_delay.start_count_ms_track=true; //ѭ��ģʽ��ʱ
		tracking_offset.Change_yaw.toward_line_mode=START_TOWARD;//��ʼת��yaw
		LED4(ON);
	}

}





/****************************
�Զ�����ģʽ�ж�
*****************************/
void AutoLand_Mode(float *high)
{
	
	//if(*high<=1)//�����˵����������������
	{
		if((1<=*high)&&(*high <=25)&&(Plane_Mode.land==true))
		{
			
			//����
			Plane_Mode.ARMED=false;
			Plane_Mode.unlock  =false;
			
			//��λģʽ
			Plane_Mode.althold =false;
			Plane_Mode.poshold =false;
			Plane_Mode.reset_start=true;
			
			Plane_Mode.land=false;
			Plane_Mode.auto_liftoff=true;
			
		}

  }
}


/***************************
�趨ƫ����
****************************/
void Set_Heading(float *now_angle)
{
  angle.heading=*now_angle;
}

/**************************
��ʼ������
***************************/
void Set_Init_Par()
{
	  Plane_Mode.auto_liftoff =true;//�Զ����ģʽ
	  tracking_offset.forward_angle=3.0;//ǰ���ĽǶ�
		mode_ABA.forward_angle=-3.2;
  	Ultra.Add_Thro=60;//���ͻ������
		Ultra.FINAL_HIGH=85;//���߸߶�
		Ultra.AltHoldThro= Reset_thro;//���¿�������ģʽʱ������
		if(Plane_Mode.auto_liftoff==true)//�����Զ����ģʽ
		{
			LED3(ON);
			Set_GoalHigh(Reset_high);
		  Init_SonarPid( &sonar_shell_pid,30,50,400);//25 60  ��70 25 �޳���   40 60 500  60 60 200
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
��λ���в���
*************************/
void Reset_All_Pid_Par()
{
	//��λ����
	Reset_Static_Par(&Plane_Mode.reset_start,PARAMETER);
	//��λ��̬����
	Reset_Static_Par(&Plane_Mode.ARMED ,INTEGRAL);
	Reset_Static_Par(&Plane_Mode.ARMED ,POS_INTEGRAL);
}

/****************************
�Զ�����
*****************************/
void Auto_Land(u8 circle_num )
{
		//�Զ�������ʱ
	  //��һ���ҵ�Բ
		if(tracking_offset.circle_count==circle_num)
		{
				if(struct_delay.start_count_ms_land==true)
				{
					 if((struct_delay.ms_land++)>=400)//1s����
					 {
						 Plane_Mode.land=true;//����
						 struct_delay.start_count_ms_land=false;//�رռ���
						 struct_delay.ms_land=0;
					 }
					
				}
				if(Ultra.z<=20)
				{
					Ultra.AltHoldThro-=15;
				}
		}

}












