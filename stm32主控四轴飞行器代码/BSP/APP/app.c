#include "UserSys.h"
#include "app.h"
#include "mode.h"
void IAC_Init(void)
{
	 delay_init(72);  //延时函数初始化
	 Nvic_Init();     //中断初始化 
	 LED_Init();      //led初始化
	 KEY_Init();    //按键初始化
	 OLED_Init();     //oled初始化
	 //Draw_LibLogo();  //logo	
    paramLoad();    //pid参数初始化
	 State_Display();//OLED数据显示
   Moto_Init();		  //电机初始化//200hz
	
	 usart1_init(72,115200); //串口1初始化  摄像头
	 usart2_Init(36,9600);    //ks103数据 9600 
	 usart3_config(460800); //串口3初始化   上位机
	 TIM3_Init(2500); //定时器初始化 2.5MS
}
void State_Display(void)
{
    OLED_Fill(0x00); //清屏	
	  OLED_P8x16Str(30,3,"WelCome!");
//	OLED_P6x8Str(0,6,"THR:");
//	OLED_P6x8Str(63,5,"ROLL:");
//	OLED_P6x8Str(63,6,"PITCH:");
//  OLED_P6x8Str(0,7,"r");
//  OLED_P6x8Str(42,7,"p");	
//	OLED_P6x8Str(84,7,"y");	
}


void  OledDispaly(void)
{
  OLED_4num(4,5,tracking_offset.goal_angle);
  OLED_4num(4,6,tracking_offset.Check.circle_find);
	Dis_Float(5,93,key,1);
	Dis_Float(6,93,cam_pos_ctr.PITCH,1);
	Dis_Float(7,6,angle.roll,1);
	Dis_Float(7,48,angle.pitch,1);
	Dis_Float(7,90,angle.yaw,1);
	
	#ifdef DOUBLE_PIDHOLD_DEBUG
	if(Ultra.SetHigh.Mode==SetHigh_ing)	
	  OLED_P6x8Str(0,2,"Althold");
	else if(Ultra.SetHigh.Mode==0) 
		OLED_P6x8Str(0,2,"AltStop");
	#endif
	
	#ifdef PID_POS_DEBUG
	if(tracking_offset.track_mode==END_TRACK)
		OLED_P6x8Str(0,2,"endtra");
	else if(tracking_offset.track_mode==START_TRACK)
		OLED_P6x8Str(0,2,"starttra");
	else if(tracking_offset.track_mode==TRACK_ING)
	  OLED_P6x8Str(0,2,"traing");
	#endif
	if(cam_pos_ctr.FixedPos_Mode==FIX_ING) 
		OLED_P6x8Str(0,3,"Pos: ON");
	else  if(cam_pos_ctr.FixedPos_Mode==CLOSED)           
		OLED_P6x8Str(0,3,"Pos:OFF");
    
	
	 if(Plane_Mode.land==true) 
		OLED_P6x8Str(0,4,"LAND: ON");
	else  if(Plane_Mode.land==false)           
		OLED_P6x8Str(0,4,"LAND:OFF");


	#ifdef   DOUBLE_PIDHOLD_DEBUG //
		 Dis_Float(2,70,sonar_shell_pid.setgoal,2);//
	   Dis_Float(3,70,Ultra.z,2);//  Ultra.z  cm 高度  //超声波高度
	   Dis_Float(4,70,Ultra.Add_Thro,1);		
	
		 Dis_Float(0,6,sonar_shell_pid.P/100.00,2);
	   Dis_Float(0,42,sonar_shell_pid.I/10000.0,4);
	   Dis_Float(0,84,sonar_shell_pid.D/100.00,2);
		 
		 Dis_Float(1,6,sonar_core_pid.P/100.00,2);
		 Dis_Float(1,42,sonar_core_pid.I/10000.00,4);
	   Dis_Float(1,84,sonar_core_pid.D/100.00,2);
	#endif
  #ifdef PID_POS_DEBUG 
		Dis_Float(0,6, pos_pid_ctr.PosPid_Y.Shell.P ,3);
	  Dis_Float(0,42,pos_pid_ctr.PosPid_Y.Shell.I ,3);
	  Dis_Float(0,84,pos_pid_ctr.PosPid_Y.Shell.D ,2);
	
		Dis_Float(1,6, pos_pid_ctr.PosPid_Y.Core.P ,2);
	  Dis_Float(1,42,pos_pid_ctr.PosPid_Y.Core.I ,3);
	  Dis_Float(1,84,pos_pid_ctr.PosPid_Y.Core.D ,3);
	
	  Dis_Float(2,50,Ultra.z,2);//  Ultra.z  cm 高度  //超声波高度
	  Dis_Float(3,50, cam_pos_ctr.pos_ctr_x.valid_distance,1);
	  Dis_Float(4,50, cam_pos_ctr.pos_ctr_y.valid_distance,1);
	  Dis_Float(3,90, tracking_offset.Change_yaw.final_alladd ,1);
	  Dis_Float(4,90, cam_pos_ctr.pos_ctr_y.v ,1);
	#endif
	#ifdef ATTITUDE_PID_DEBUG
	 	Dis_Float(0,6,ctrl.roll.shell.kp ,3);
	  Dis_Float(0,42,ctrl.roll.shell.ki,4);
	  Dis_Float(0,84,ctrl.roll.shell.kd,2);
	
		Dis_Float(1,6,  ctrl.roll.core.kp,2);
	  Dis_Float(1,42, ctrl.roll.core.ki,4);
	  Dis_Float(1,84, ctrl.roll.core.kd,3);
		
	  Dis_Float(3,100, pitch_offset,2);
		Dis_Float(4,100, roll_offset,2);
	#endif


}
