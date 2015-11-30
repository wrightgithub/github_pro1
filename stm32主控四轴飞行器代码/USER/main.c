#include "Usersys.h"
#include "mode.h"
 u8 UltraSendFlag=1;//超声波发送读数据指令标志 1：发送
 u8 start_pos=0;  //用于在上位机上看定点开关
 u8 key=0;
extern bool start_tick;
int main(void)
{   
    IAC_Init();     //接口以及外设的初始化                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     
	  mpu_dmp_init();   //dmp初始化DEFAULT_MPU_HZ 100hz
	  delay_ms(100);
	  /********所有初始化完成后再开定时器****/
	  EnTIM3();       //开定时器
	  memset(&Plane_Mode,0,sizeof(Plane_Mode));
	  memset(&sonar_shell_pid,0,sizeof(sonar_shell_pid));
	  memset(&sonar_core_pid,0,sizeof(sonar_core_pid));
	  memset(&struct_delay,0,sizeof(struct_delay));
	  memset(&Static_Par,0,sizeof(Static_Par));
	  memset(&tracking_offset,0,sizeof(tracking_offset));
		memset(&mode_ABA,0,sizeof(mode_ABA));
	  /************************************************************/
	  OLED_Fill(0x00); //清屏
    cam_pos_ctr.binaryzation_threshold=110;	
    while(1)
	  {   	
			//用于测试1s接收到多少个摄像头数据
		  start_tick=true;
			//初始延时
      First_Delay(&struct_delay);
			
			//上位机控制模式
		  Mode_Execute(&Plane_Mode);
			
			
			//不同模式下的一些操作
			start_pos=Mode_Fixed_Position(&cam_pos_ctr);
		  Mode_AltHold(&Ultra,Plane_Mode.auto_liftoff,&sonar_shell_pid);
				
			
			//达到设定高度后的处理，换一组稳定的PID定高
			Receive_SetHigh(&Ultra.z,sonar_valid,Ultra.SetHigh.Mode,Ultra.FINAL_HIGH); 
			
//			//复位参数
//			Reset_Static_Par(&Plane_Mode.reset_start,PARAMETER);
//			//复位姿态积分
//			Reset_Static_Par(&Plane_Mode.ARMED ,INTEGRAL);
//			Reset_Static_Par(&Plane_Mode.ARMED ,POS_INTEGRAL);
//			
      /*******与上位机的数据交互*****/
			//从上位机获取PID数据
	    PidDataReceive();   
			
		  /****定时器时序********/	
			
			//向上位机传数据
			if(ms3>=8)//20ms 
			{
				ms3=0; 
				Report_Message(&SendCount);
			}
					
			//解锁指示灯	
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
			
						//解锁后不显示oled
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











