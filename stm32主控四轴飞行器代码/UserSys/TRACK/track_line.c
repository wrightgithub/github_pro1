#include "track_line.h"
#include "mode.h"
_TRACKING_OFFSET tracking_offset;
_MODE_ABA mode_ABA;
u16 START_TRACK_DELAY=500;  //等待多少时间开始循迹
u16 AUTO_LAND_DELAY  = 1600;  //等待多少时间自动降落

void Start_Track_Delay(void)
{
	//开始循迹的延时

	//达到设定高度后旋转完成后再过n s开始循迹
	if(struct_delay.start_count_ms_track==true&&(tracking_offset.Change_yaw.toward_line_mode==FINISH_TOWARD))
	{
		//等循迹延时完成并且高度超调结束了
		if(  ((struct_delay.ms_start_track++)>=START_TRACK_DELAY)&&(Ultra.z<= (Ultra.FINAL_HIGH+10) ) )//2s开始循迹
		{
			tracking_offset.track_mode=START_TRACK;//开始循迹
			struct_delay.start_count_ms_track=false;
			struct_delay.ms_start_track=0;
//			if(tracking_offset.Change_yaw.once_in==true)
//			{
//			   if( (struct_delay.ms_B_TO_C++>1000)&&(struct_delay.ms_B_TO_C_OK==false))//2.5s
//				 {
//				   struct_delay.ms_B_TO_C_OK=true;
//				 }
//			}
			
		}
	}

}
/***************************
设定循迹前进的角度
****************************/
void Set_forward_angle(float angle)
{
  tracking_offset.forward_angle =angle;
}
/****************************************
循迹的状态转换
****************************************/
void Mode_Track_Convert(void)
{
	//  <1>如果处于正在循迹模式下，等找到圆了就进入结束循迹模式
	if(tracking_offset.track_mode==TRACK_ING)
	{
		if( (struct_delay.ms_search_circle_B++>=100)&&(struct_delay.start_search_circle_B==false))//2s
		{
		  struct_delay.start_search_circle_B=true;
			struct_delay.ms_search_circle_B=0;
		}
		
		if(struct_delay.start_search_circle_B==true)//可以开始寻找B
		{
			if(tracking_offset.Check.circle_find==true)
			{
				tracking_offset.track_mode=END_TRACK;//进入结束循迹模式
				tracking_offset.circle_count++;  //对找到的圆计数
				
				//  struct_delay.start_ABC_change_yaw =true;//开始ABC的yaw旋转
					struct_delay.start_B_stabilize=true;//开始B点的自稳
				
			}
		}
	}
			
	// <2>如果在开始循迹和正在循迹模式(非结束循迹模式)下
	if(tracking_offset.track_mode!=END_TRACK)
	{			
		//前进
		tracking_offset.goal_angle=tracking_offset.forward_angle;//往前给正
		//if(	((tracking_offset.circle_count==1)&&(  struct_delay.ms_B_TO_C_OK==true)) ||(tracking_offset.circle_count==0) )
		//if(tracking_offset.line_find==1)
		if( tracking_offset.track_mode==TRACK_ING)
		{
			//位移转换
//			if(tracking_offset.circle_count==1)
//			{
//				static u16 last_offset=0;
//				if(Int_Fabs(tracking_offset.offset-last_offset)>40)
//				{
//					tracking_offset.offset=last_offset;
//				}
//				
//				last_offset=	tracking_offset.offset;
//			}
			
			cam_pos_ctr.pos_ctr_y.valid_distance =tracking_offset.offset;
			tracking_offset.off_v=constrain(tracking_offset.off_v,-20,20);
			cam_pos_ctr.pos_ctr_y.v              =tracking_offset.off_v;
			
		
		}
		
	}
	//定点不循迹
	else if(tracking_offset.track_mode==END_TRACK)
	{
		tracking_offset.goal_angle=0;
	}
	//<3>如果处于开始循迹模式，找不到圆后进入完全循迹模式
	 if(tracking_offset.track_mode==START_TRACK)
	{
	  if(tracking_offset.Check.circle_find==false)
		{
			//找不到圆后进入完全循迹模式
			tracking_offset.track_mode=TRACK_ING;
		}
	}
}

/*******************************************
起飞改变偏航角度
*******************************************/
void Change_Yaw(u16 ms_delay)
{
	static int _add_yaw=0;
	static bool rec_yaw_first_once=false;
	static float first_angle_tolerance=0;//上电瞬间的偏航
	u16 T=ms_delay/30;
	//记录下刚上电状态下的yaw
	if((Plane_Mode.ARMED==0)&&(rec_yaw_first_once==false))//上锁状态
	{
	  first_angle_tolerance=angle.yaw;
		rec_yaw_first_once=true;
	}
	
	//记录下每次解锁一瞬间的yaw 
	if((Plane_Mode.ARMED==1)&&(tracking_offset.Change_yaw.rec_yaw_second_once==false))
	{
	  tracking_offset.Change_yaw.second_angle_tolerance=angle.yaw;
		tracking_offset.Change_yaw.rec_yaw_second_once=true;
	}
	
	//开始转向直线
	if(tracking_offset.Change_yaw.toward_line_mode==START_TOWARD)
	{
		if(tracking_offset.Change_yaw.save_angle_tolerance==false)
		{
			tracking_offset.Change_yaw.final_alladd=first_angle_tolerance-tracking_offset.Change_yaw.second_angle_tolerance ;//最终的增量
			if(tracking_offset.Change_yaw.final_alladd>0)//向左旋转
			{ 
				_add_yaw=1;
			}
			else if (tracking_offset.Change_yaw.final_alladd<0)
			{
			  _add_yaw=-1;
			}
			tracking_offset.Change_yaw.save_angle_tolerance=true;
		}
		
		//每200ms累加一次偏航
		if(struct_delay.ms_toward_line++>=T)
		{
			angle.heading+=_add_yaw;
			tracking_offset.Change_yaw.have_addangle++;
			struct_delay.ms_toward_line=0;
		}
		
		if(Int_Fabs(tracking_offset.Change_yaw.have_addangle)>=Int_Fabs(tracking_offset.Change_yaw.final_alladd))
		{
			tracking_offset.Change_yaw.toward_line_mode=FINISH_TOWARD; 
			//旋转偏航完成，复位参数	
			_add_yaw=0;
		}	
		
	}
	
		
}


/***************************
B到C改变yaw的值
***************************/
void B_to_C_yaw(float yaw)
{
	#define CHANGE_NUM 10
	static u16 have_change_yaw_count=0;
			
		if((struct_delay.start_B_stabilize==true)&&(struct_delay.POINT_B_stabilize==NONE))//开始B点定点
		{
			 if(struct_delay.ms_point_B_stabilize++>=1000)//s
			{
				 struct_delay.start_B_stabilize=false;//B点的稳态完成
				 struct_delay.POINT_B_stabilize=STABILIZE_OK;//B点的稳态完成
				
				 struct_delay.start_ABC_change_yaw=true;//开始ABC的yaw旋转
				 struct_delay.ms_point_B_stabilize=0;//计时清零
			}
		}
	//旋转只能进行一次
    if(tracking_offset.Change_yaw.once_in==false)
		{
			if(struct_delay.start_ABC_change_yaw==true)//开始ABC的yaw旋转
			{
				 if((struct_delay.ms_ABC_change_yaw++)>=1)//125ms转一下
				 {
					 struct_delay.ms_ABC_change_yaw=0;
					 
					// angle.heading+=(yaw/(1.0*CHANGE_NUM));
					 angle.heading+=10;
					 DCT(0);
					 Ultra.AltHoldThro-=10;
					 //have_change_yaw_count++;
					 //if(have_change_yaw_count>=CHANGE_NUM)
					 {
							have_change_yaw_count=0;
						 
					    Set_forward_angle(mode_ABA.forward_angle);//往后走
//						  tracking_offset.offset=0; 
//						  tracking_offset.off_v=0;
//						  cam_pos_ctr.pos_ctr_y.valid_distance=0;
//						  cam_pos_ctr.pos_ctr_y.v=0;
						  struct_delay.start_search_circle_B=false;
						 
						  struct_delay.start_ABC_change_yaw=false;//旋转ABC_yaw关闭
							struct_delay.start_count_ms_track=true;//准备开始循迹的延时
						  tracking_offset.Change_yaw.once_in=true;//保证只进来一次
					 }
				
				 }
				
			}
	}
}

/**********************************
ABA往返模式
***********************************/
void Mode_ABA_OF_B_to_A(void)
{
  	if((struct_delay.start_B_stabilize==true)&&(struct_delay.POINT_B_stabilize==NONE))//开始B点定点
		{
			 if(struct_delay.ms_point_B_stabilize++>=2000)//5s
			{
				 struct_delay.start_B_stabilize=false;//B点的稳态完成
				 struct_delay.POINT_B_stabilize=STABILIZE_OK;//B点的稳态完成
				 struct_delay.ms_point_B_stabilize=0;//计时清零
				
				 mode_ABA.start_B_to_A=true;//开始B到A的模式
				 //Set_forward_angle(-2);//往后走
			}
		}
		if(mode_ABA.once_in==false )//保证只进来一次
		{
			if( mode_ABA.start_B_to_A==true)
			{
				Set_forward_angle(mode_ABA.forward_angle);//往后走

				struct_delay.start_count_ms_track=true;//准备开始循迹的延时
				
				mode_ABA.start_B_to_A=false;
				mode_ABA.once_in=true;
			}
	  }
}



void Set_Track_delay(u16 delay_num)
{
  START_TRACK_DELAY=delay_num;
}

void Set_AutoLand_delay(u16 delay_num)
{
  AUTO_LAND_DELAY=delay_num;
}

















