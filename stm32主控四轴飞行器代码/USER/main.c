#include "Usersys.h"
#include "mode.h"
 u8 UltraSendFlag=1;//���������Ͷ�����ָ���־ 1������
 u8 start_pos=0;  //��������λ���Ͽ����㿪��
 u8 key=0;
extern bool start_tick;
int main(void)
{   
    IAC_Init();     //�ӿ��Լ�����ĳ�ʼ��                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	  mpu_dmp_init();   //dmp��ʼ��DEFAULT_MPU_HZ 100hz
	  delay_ms(100);
	  /********���г�ʼ����ɺ��ٿ���ʱ��****/
	  EnTIM3();       //����ʱ��
	  memset(&Plane_Mode,0,sizeof(Plane_Mode));
	  memset(&sonar_shell_pid,0,sizeof(sonar_shell_pid));
	  memset(&sonar_core_pid,0,sizeof(sonar_core_pid));
	  memset(&struct_delay,0,sizeof(struct_delay));
	  memset(&Static_Par,0,sizeof(Static_Par));
	  memset(&tracking_offset,0,sizeof(tracking_offset));
		memset(&mode_ABA,0,sizeof(mode_ABA));
	  /************************************************************/
	  OLED_Fill(0x00); //����
    cam_pos_ctr.binaryzation_threshold=110;	
    while(1)
	  {   	
			//���ڲ���1s���յ����ٸ�����ͷ����
		  start_tick=true;
			//��ʼ��ʱ
      First_Delay(&struct_delay);
			
			//��λ������ģʽ
		  Mode_Execute(&Plane_Mode);
			
			
			//��ͬģʽ�µ�һЩ����
			start_pos=Mode_Fixed_Position(&cam_pos_ctr);
		  Mode_AltHold(&Ultra,Plane_Mode.auto_liftoff,&sonar_shell_pid);
				
			
			//�ﵽ�趨�߶Ⱥ�Ĵ�����һ���ȶ���PID����
			Receive_SetHigh(&Ultra.z,sonar_valid,Ultra.SetHigh.Mode,Ultra.FINAL_HIGH); 
			
//			//��λ����
//			Reset_Static_Par(&Plane_Mode.reset_start,PARAMETER);
//			//��λ��̬����
//			Reset_Static_Par(&Plane_Mode.ARMED ,INTEGRAL);
//			Reset_Static_Par(&Plane_Mode.ARMED ,POS_INTEGRAL);
//			
      /*******����λ�������ݽ���*****/
			//����λ����ȡPID����
	    PidDataReceive();   
			
		  /****��ʱ��ʱ��********/	
			
			//����λ��������
			if(ms3>=8)//20ms 
			{
				ms3=0; 
				Report_Message(&SendCount);
			}
					
			//����ָʾ��	
			if(ms4>=4)//10ms
			{
				ms4=0;		
				if(Plane_Mode.ARMED)	
					LED1(ON);
				else
					LED1(OFF);
			}

			
//			Track_Mode_Send();
//			usart1_Send_Data(Image_Par_buf,9);	
			
						//��������ʾoled
			if(!Plane_Mode.ARMED)
			{	
				if(Plane_Mode.fly_start==true)
				{
					if(ms6_oled>=400)
					{
				    Fly_Start_Count(&Plane_Mode.fly_count);
						ms6_oled=0;
					}
					if(Plane_Mode.fly_count==3)
					{
					   Key_Mode_Start();
					}
				}
				else
				{
				  key=Key_Scan();
          Menu_Layer_Select(&key);		
				  OLED_MENU(&Now_Layer);
				}					
			}
				

		
	  }
}











