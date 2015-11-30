#include "menu.h"
#include "mode.h"
u8 Image_Par_buf[20];
_LAYER Now_Layer=Layer0;
void OLED_MENU(_LAYER *Now_Layer)
{
  switch(*Now_Layer)
	{
	  case Layer0://第0层显示基本的传感器数据
		{
			DisPlay_Layer0();
		  break;
		}
		case Layer1://第1层选择选择要进行的比赛模式模式
		{
		  DisPlay_Layer1();
		  break;
		}
		case Layer2://第2层显示可设置的比赛需要的参数，如高度,阈值
		{
			DisPlay_Layer2();
		  break;
		}
		case Layer3://显示定高PID
		{			
			DisPlay_Layer3();
		  break;
		}
		case Layer4://显示定点PID
		{
			DisPlay_Layer4();
		  break;
		}
		case Layer5://姿态PID
		{
			DisPlay_Layer5();
		  break;
		}
		case Layer6:
		{
			DisPlay_Layer6();
		  break;
		}
		default:break;
	}
}



void DisPlay_Layer0(void)
{
  OLED_P6x8Str(0,0,"ROLL");
	OLED_P6x8Str(0,1,"PITCH");
	OLED_P6x8Str(0,2,"YAW");
	OLED_P6x8Str(0,3,"HIGH");
  OLED_P6x8Str(0,4,"POS.X");
	OLED_P6x8Str(0,5,"POS.Y");
	OLED_P6x8Str(0,6,"X.V");
	OLED_P6x8Str(0,7,"Y.V");
	OLED_P6x8Str(80,0,"y:");
	OLED_P6x8Str(80,1,"yv");
	Dis_Float(0,40,angle.roll,1); 
	Dis_Float(1,40,angle.pitch,1);
	Dis_Float(2,40,angle.yaw,1);   OLED_4num(80,2,tracking_offset.Check.circle_find);
	Dis_Float(3,40,Ultra.z,2);     OLED_4num(80,3,tracking_offset.circle_count);
	Dis_Float(4,40, cam_pos_ctr.pos_ctr_x.valid_distance,1);   	Dis_Float(4,80,tracking_offset.goal_angle,2);
	Dis_Float(5,40, cam_pos_ctr.pos_ctr_y.valid_distance,1);
	Dis_Float(6,40, cam_pos_ctr.pos_ctr_x.v ,1);                 OLED_4num(80,6, tracking_offset.line_find);
	Dis_Float(7,40, cam_pos_ctr.pos_ctr_y.v ,1);
	Dis_Float(0,96, tracking_offset.valid_offset ,1);
	Dis_Float(1,96, tracking_offset.off_v ,1);
	
}

void DisPlay_Layer1(void)
{
	/*************状态显示**********************/
	//定高状态
	if(Ultra.SetHigh.Mode==SetHigh_ing)	
	  OLED_P6x8Str(0,2,"Althold");
	else if(Ultra.SetHigh.Mode==0) 
		OLED_P6x8Str(0,2,"AltStop");
	//循迹状态
	if(tracking_offset.track_mode==END_TRACK)
		OLED_P6x8Str(0,3,"endtra     ");
	else if(tracking_offset.track_mode==START_TRACK)
		OLED_P6x8Str(0,3,"starttra   ");
	else if(tracking_offset.track_mode==TRACK_ING)
	  OLED_P6x8Str(0,3,"traing    ");
	//定点状态
	if(cam_pos_ctr.FixedPos_Mode==FIX_ING) 
		OLED_P6x8Str(0,4,"Pos: ON");
	else  if(cam_pos_ctr.FixedPos_Mode==CLOSED)           
		OLED_P6x8Str(0,4,"Pos:OFF");
    //降落状态
	 if(Plane_Mode.land==true) 
		OLED_P6x8Str(0,5,"LAND: ON");
	else  if(Plane_Mode.land==false)           
		OLED_P6x8Str(0,5,"LAND:OFF");
	
	OLED_4num(80,2,tracking_offset.Check.circle_find);
	Dis_Float(4,80,tracking_offset.goal_angle,2);
	
}

void DisPlay_Layer2(void)
{
		//可设置的比赛模式
  OLED_P6x8Str(0,0,"Mode:");
	switch(fly_mode)
	{
	  case 1:
		{
			OLED_P6x8Str(30,0,"ALTHOLD");
		  break;
		}
		case 2:
		{
			OLED_P6x8Str(30,0,"FIXED.POINT");
		  break;
		}
		case 3:
		{
			OLED_P6x8Str(30,0,"TRACK.LINE");
		  break;
		}
		case 4:
		{
			OLED_P6x8Str(30,0,"LINE.ABC");
		  break;
		}
		case 5:
		{
			OLED_P6x8Str(30,0,"TRACK.ABA");
		  break;  
		}
		default:break;
	}
	
  OLED_P6x8Str(0,2,"goalhigh");
	Dis_Float(2,54,Ultra.FINAL_HIGH,1);
	OLED_P6x8Str(0,3,"forward.angle");
	Dis_Float(3,84,tracking_offset.forward_angle ,1);
	OLED_P6x8Str(0,4,"RecImg");
	if(Plane_Mode.Receive_Img==true)
	{
	  OLED_P6x8Str(64,4,"YES");
	}
	else
	{
	  OLED_P6x8Str(64,4,"NO ");
	}
	
	OLED_P6x8Str(0,5,"Add_Thro");
	Dis_Float(5,54,Ultra.Add_Thro,1);
	
	OLED_P6x8Str(16,6,"sure to start?");
	OLED_P6x8Str(0,7,"Start:");
	if(Plane_Mode.fly_start==true)
	{
			OLED_P6x8Str(56,7,"YES");
	}
	else
	{
	    OLED_P6x8Str(56,7,"NO ");
	}
	  
}

void DisPlay_Layer3(void)
{	
	OLED_P6x8Str(30,0,"ALTHOLD.PID");
	OLED_P6x8Str(0,2,"S.P");
	OLED_P6x8Str(0,3,"S.I");
	OLED_P6x8Str(0,4,"S.D");
	
	OLED_P6x8Str(0,5,"C.P");
	OLED_P6x8Str(0,6,"C.I");
	OLED_P6x8Str(0,7,"C.D");
	
	Dis_Float(2,24,sonar_shell_pid.P/100.00,2);
	Dis_Float(3,24,sonar_shell_pid.I/10000.0,4);
	Dis_Float(4,24,sonar_shell_pid.D/100.00,2);
 
	Dis_Float(5,24,sonar_core_pid.P/100.00,2);
	Dis_Float(6,24,sonar_core_pid.I/10000.00,4);
	Dis_Float(7,24,sonar_core_pid.D/100.00,2);
}

void DisPlay_Layer4(void)
{
  OLED_P6x8Str(30,0,"POSHOLD.PID");
	OLED_P6x8Str(0,2,"S.P");
	OLED_P6x8Str(0,3,"S.I");
	OLED_P6x8Str(0,4,"S.D");
	
	OLED_P6x8Str(0,5,"C.P");
	OLED_P6x8Str(0,6,"C.I");
	OLED_P6x8Str(0,7,"C.D");
	
	Dis_Float(2,24, pos_pid_ctr.PosPid_Y.Shell.P ,3);
	Dis_Float(3,24,pos_pid_ctr.PosPid_Y.Shell.I ,3);
	Dis_Float(4,24,pos_pid_ctr.PosPid_Y.Shell.D ,2);

	Dis_Float(5,24, pos_pid_ctr.PosPid_Y.Core.P ,2);
	Dis_Float(6,24,pos_pid_ctr.PosPid_Y.Core.I ,3);
	Dis_Float(7,24,pos_pid_ctr.PosPid_Y.Core.D ,3);
}

void DisPlay_Layer5(void)
{
  OLED_P6x8Str(16,0,"AUTTITUDE.PID");
	Dis_Float(2,6,ctrl.roll.shell.kp ,3);
	Dis_Float(2,42,ctrl.roll.shell.ki,4);
	Dis_Float(2,84,ctrl.roll.shell.kd,2);

	Dis_Float(3,6,  ctrl.roll.core.kp,2);
	Dis_Float(3,42, ctrl.roll.core.ki,4);
	Dis_Float(3,84, ctrl.roll.core.kd,3);
	OLED_P6x8Str(16,5,"YAW.PID");
	Dis_Float(6,6,ctrl.yaw.shell.kp ,3);
	Dis_Float(6,42,ctrl.yaw.shell.ki,4);
	Dis_Float(6,84,ctrl.yaw.shell.kd,2);
	Dis_Float(7,6,  ctrl.yaw.core.kp,2);
	Dis_Float(7,42, ctrl.yaw.core.ki,4);
	Dis_Float(7,84, ctrl.yaw.core.kd,3);
	
  
}

extern u16 pix_count;
extern u16 rx_start;
extern bool start_receive_img;
void DisPlay_Layer6(void)
{
	if(	Plane_Mode.Rec_Img_ok==true)
	{
		
		cam_pos_ctr.binary_rec_temp=camera_image_buf[0];
		Draw_Pic(camera_image_buf+1);
		
		Plane_Mode.Rec_Img_ok=false;
		start_receive_img=false;
		rx_start=0;
		pix_count=0;
	}
	
  OLED_P6x8Str(86,0,"binary");
	Dis_Float(1,86,cam_pos_ctr.binaryzation_threshold,1);
	Dis_Float(2,86,cam_pos_ctr.binary_rec_temp ,1);
	OLED_P6x8Str(0,7,"RecImg:");
	if(Plane_Mode.Receive_Img==true)
	{
			OLED_P6x8Str(64,7,"YES");
	}
	else
	{
	    OLED_P6x8Str(64,7,"NO ");
	}
}


void Layer2_Add_Par(const u16 *next)
{
   u16	line=*next-1;
   switch(line)
	 {
	   case 0://第一行改变比赛模式
		 {
			  Set_Layer.line0++;
			 if( Set_Layer.line0>=(mode_max+1) )
			 {
				  Set_Layer.line0=mode_min;
			 }
			 fly_mode=(_FLY_MODE) Set_Layer.line0;
			 OLED_P6x8Str(30,0,"            ");
		   break;
		 }
		 case 1://
		 {
		   break;
		 }
		 case 2://第二行改变超声波定高高度
		 {
			 Ultra.FINAL_HIGH++;
		   break;
		 }
		 case 3://第三行改变循迹前进的角度
		 {
			 tracking_offset.forward_angle+=0.1;
		   break;
		 }
		 case 4://
		 {
			 Set_Layer.line4++;
			 if(Set_Layer.line4==1)
			 {
			   Plane_Mode.Receive_Img=true;
         Data_Send_Image_Par(Image_Par_buf,ORDER_FRAME, 0xA4,0);//开
			   usart1_Send_Data(Image_Par_buf,9);
			 }
			 if(Set_Layer.line4>=2)
			 {
				 Set_Layer.line4=0;
			   Plane_Mode.Receive_Img=false;
         Data_Send_Image_Par(Image_Par_buf,ORDER_FRAME, 0x0f,0);//关
			   usart1_Send_Data(Image_Par_buf,9);	
			 }
		   break;
		 }
		 case  5://
		 {
			  Ultra.Add_Thro++;
		   break;
		 }
		 case 6://
		 {
		   break;
		 }
		 case 7://
		 {
			  Set_Layer.line7++;
			 if(Set_Layer.line7==1)
			 {
			   Plane_Mode.fly_start=true;
			 }
			 if(Set_Layer.line7>=2)
			 {
				 Set_Layer.line7=0;
			   Plane_Mode.fly_start=false;
			 }
				 

		   break;
		 }
		 
		 default:break;
		 
	 }
}


void Layer2_Reduce_Par(const u16 *next)
{
   u16	line=*next-1;
   switch(line)
	 {
	   case 0://第一行改变比赛模式
		 {
			 Set_Layer.line0--;
			 if( Set_Layer.line0<=(mode_min-1) )
			 {
				  Set_Layer.line0=mode_max;
			 }
			 fly_mode=(_FLY_MODE) Set_Layer.line0;
			 OLED_P6x8Str(30,0,"            ");
		   break;
		 }
		 case 1://
		 {
		   break;
		 }
		 case 2://第二行改变超声波定高高度
		 {
			 Ultra.FINAL_HIGH--;
		   break;
		 }
		 case 3://第三行改变循迹前进的角度
		 {
			 tracking_offset.forward_angle-=0.1;
		   break;
		 }
		 case 4://
		 {
		   break;
		 }
		 case  5://
		 {
			 Ultra.Add_Thro--;
		   break;
		 }
		 case 6://
		 {
		   break;
		 }
		 case 7://
		 {
		   break;
		 }
		 
		 default:break;
		 
	 }
}

void Layer6_Add_Par(const u16 *next)
{
   u16	line=*next-1;
   switch(line)
	 {
	   case 0://
		 {
		   break;
		 }
		 case 1://
		 {
			 cam_pos_ctr.binaryzation_threshold++;
			 Data_Send_Image_Par(Image_Par_buf, PAR_FRAME,cam_pos_ctr.binaryzation_threshold,0);
			 usart1_Send_Data(Image_Par_buf,9);
		   break;
		 }
		 case 2:
		 {
		   break;
		 }
		 case 3:
		 {
		   break;
		 }
		 case 4://
		 {

		   break;
		 }
		 case  5://
		 {
		   break;
		 }
		 case 6://
		 {
		   break;
		 }
		 case 7://
		 {
		   break;
		 }
		 
		 default:break;
		 
	 }
}


void Layer6_Reduce_Par(const u16 *next)
{
   u16	line=*next-1;
   switch(line)
	 {
	   case 0://第一行改变比赛模式
		 {
		   break;
		 }
		 case 1://
		 {
			 cam_pos_ctr.binaryzation_threshold--;
			 Data_Send_Image_Par(Image_Par_buf, PAR_FRAME,cam_pos_ctr.binaryzation_threshold,0);
			 usart1_Send_Data(Image_Par_buf,9);
		   break;
		 }
		 case 2://第二行改变超声波定高高度
		 {
		   break;
		 }
		 case 3://第三行改变循迹前进的角度
		 {
		   break;
		 }
		 case 4://
		 {
		   break;
		 }
		 case  5://
		 {
		   break;
		 }
		 case 6://
		 {
		   break;
		 }
		 case 7://
		 {
		   break;
		 }
		 
		 default:break;
		 
	 }
}



















