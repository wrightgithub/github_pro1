#include "UserSys.h"
#include "mode.h"
struct _ctrl ctrl;
vs16 Moto_duty[4];
extern u8 Lock;
float pitch_offset=0;//-0.7;//0.2;//-0.5;//-2.9;//-1;  //前飘给正，后飘给负
float roll_offset=0;//0.35;//0.25;//1;//1.1;    //左飘给正，右飘给负
int moto[4]={0,0,0,0};//电机补偿
/// Low pass filter cut frequency for derivative calculation.
// static const float ac_pid_filter = 1.0f / (2.0f * M_PI * (float)cfg.gps_lpf); // Set to  "1 / ( 2 * PI * f_cut )"
//#define NOISE_FRE 200   //200hz  // Low pass filter cut frequency for derivative calculation (default 20Hz)
//#define M_PI       3.14159265358979323846f
//#define AC_PID_FILTER       (1.0f / (2.0f * M_PI * (float)NOISE_FRE))
//static int32_t AC_PID_get_d(struct _pid * ac_pid, int32_t input, float dt)
//{
//    ac_pid->_derivative = (input - ac_pid->_last_input) / dt;  
//    // discrete low pass filter, cuts out the
//    // high frequency noise that can drive the controller crazy
//    ac_pid->_derivative = ac_pid->_last_derivative + (dt/(AC_PID_FILTER + dt)) * (ac_pid->_derivative - ac_pid->_last_derivative);
//    // update state
//    ac_pid->_last_input = input;
//    ac_pid->_last_derivative = ac_pid->_derivative;
//    // add in derivative component
//    return  ac_pid->_derivative;
//}

extern u16 add_yaw;
void CONTROL(float rol, float pit, float yaw)
{
	static float last_roll_error=0,last_pitch_error=0,last_yaw_error=0,
               roll_rate_error_last=0,pitch_rate_error_last=0,yaw_rate_error_last=0;
	
	//if(ctrl.ctrlRate >= 1)  //内环进行2次控制   外环进行1次控制   内环控制频率为外环的2倍 
	{
		//*****************外环PID**************************//
		//俯仰计算//
		float pitch_error = -pit  + pitch_offset -	cam_pos_ctr.PITCH;
		ctrl.pitch.shell.increment += pitch_error;   	
			
			//积分限幅
		ctrl.pitch.shell.ki_out=( ctrl.pitch.shell.ki * ctrl.pitch.shell.increment )/Control_Frequency;
		ctrl.pitch.shell.ki_out =constrain(ctrl.pitch.shell.ki_out , -30, 30);//积分限幅
		
		
		ctrl.pitch.shell.pid_out = ctrl.pitch.shell.kp * pitch_error
                             	 + ctrl.pitch.shell.ki_out 
		                           + ctrl.pitch.shell.kd * (pitch_error - last_pitch_error);
		last_pitch_error = pitch_error; //储存 俯仰偏差
		
		
		//横滚计算//
		float roll_error = -rol +  roll_offset + cam_pos_ctr.ROLL;   
		ctrl.roll.shell.increment += roll_error;
			
			//积分限幅
		ctrl.roll.shell.ki_out= ( ctrl.roll.shell.ki * ctrl.roll.shell.increment )/Control_Frequency;
		ctrl.roll.shell.ki_out =constrain(ctrl.roll.shell.ki_out , -30, 30);//积分限幅

		ctrl.roll.shell.pid_out  = ctrl.roll.shell.kp * roll_error 
		                           + ctrl.roll.shell.ki * ctrl.roll.shell.increment 
		                           + ctrl.roll.shell.kd * (roll_error - last_roll_error);
		last_roll_error = roll_error;  //储存 横滚偏差

    //航向计算//
		float yaw_error=(angle.heading)-yaw;//顺时针角度增加  若顺时转动时，误差变成负的了  那应该加1，3+正，2，4+负
		
		if(0<=angle.heading&&angle.heading<=180)
		{
			 if(yaw_error<-180)
			{
				yaw_error=360+yaw_error;
			}
		
	 }
		else 
		{
		  if(yaw_error>=180)
			{
				yaw_error=-(360-yaw_error);
			}
		}
		
		ctrl.yaw.shell.increment+=(yaw_error);
		ctrl.yaw.shell.ki_out= ( ctrl.yaw.shell.ki * ctrl.yaw.shell.increment )/Control_Frequency;
		ctrl.yaw.shell.ki_out =constrain(ctrl.yaw.shell.ki_out , -30, 30);//积分限幅
	
		//顺时针输出是负的值
    ctrl.yaw.shell.pid_out = ctrl.yaw.shell.kp    * yaw_error  
														 +ctrl.yaw.shell.ki_out
		                         + ctrl.yaw.shell.kd  *(yaw_error - last_yaw_error);    //顺时针是这个输出是负的
		 ctrl.ctrlRate = 0;
	}
	ctrl.ctrlRate ++;
	
  //********************内环(角速度环)PD*********************************//
	//if(ctrl.ctrl_core_rate >= 1)
	{
		
		float roll_rate_error=ctrl.roll.shell.pid_out - sensor.gyro.changed.y; //角速率左负右正
		ctrl.roll.core.kp_out = ctrl.roll.core.kp * roll_rate_error; 
		
		ctrl.roll.core.increment +=roll_rate_error;
		ctrl.roll.core.ki_out = (ctrl.roll.core.ki * ctrl.roll.core.increment)/Control_Frequency;
		ctrl.roll.core.ki_out =constrain(ctrl.roll.core.ki_out , -30, 30);//积分限幅
		
		
		//ctrl.roll.core.kd_out = ctrl.roll.core.kd * AC_PID_get_d(&ctrl.roll.core, roll_rate_error, 0.005);
		ctrl.roll.core.kd_out = ctrl.roll.core.kd * ((roll_rate_error - roll_rate_error_last)); ///内环的微分项也可以改成两次误差之差试试
		roll_rate_error_last = roll_rate_error;
		
		
		float pitch_rate_error=ctrl.pitch.shell.pid_out - sensor.gyro.changed.x;//倾角往前的时候 角度是负的，角速度也是负的 
		ctrl.pitch.core.kp_out = ctrl.pitch.core.kp * pitch_rate_error;
		
		ctrl.pitch.core.increment +=pitch_rate_error;//内环积分
		ctrl.pitch.core.ki_out = ( ctrl.pitch.core.ki * ctrl.pitch.core.increment)/Control_Frequency ;
		ctrl.pitch.core.ki_out =constrain(ctrl.pitch.core.ki_out  , -30, 30);
		
		//ctrl.pitch.core.kd_out = ctrl.pitch.core.kd * AC_PID_get_d(&ctrl.pitch.core, pitch_rate_error, 0.005);
		ctrl.pitch.core.kd_out = ctrl.pitch.core.kd * (pitch_rate_error - pitch_rate_error_last);
		pitch_rate_error_last = pitch_rate_error;

		//顺时针shell——yaw  输出是负的   
		float yaw_rate_error=(-ctrl.yaw.shell.pid_out) - sensor.gyro.changed.z; //偏航顺时针角速度是负的  这样顺时针误差都变大
		
		ctrl.yaw.core.increment+=yaw_rate_error;
		ctrl.yaw.core.ki_out = ( ctrl.yaw.core.ki * ctrl.yaw.core.increment)/Control_Frequency ;
		ctrl.yaw.core.ki_out =constrain(ctrl.yaw.core.ki_out  , -30, 30);
		
		ctrl.yaw.core.kp_out = ctrl.yaw.core.kp * (yaw_rate_error);
		ctrl.yaw.core.kd_out = ctrl.yaw.core.kd * (yaw_rate_error - yaw_rate_error_last);
		yaw_rate_error_last=yaw_rate_error;
		
		
		ctrl.roll.core.pid_out   =  ctrl.roll.core.kp_out  +  ctrl.roll.core.kd_out  + ctrl.roll.core.ki_out;
		ctrl.pitch.core.pid_out  =  ctrl.pitch.core.kp_out +  ctrl.pitch.core.kd_out + ctrl.pitch.core.ki_out;
		ctrl.yaw.core.pid_out    =  ctrl.yaw.core.kp_out   +  ctrl.yaw.core.kd_out   + ctrl.yaw.core.ki_out;  //顺时针是这个输出是负的

		
		ctrl.ctrl_core_rate=0;
  }
	ctrl.ctrl_core_rate++;

	
	if(Plane_Mode.ARMED)  //一旦收油什么也不进行
	{
		int date_THROTTLE=0;
		
		/*         控制采用X模式          */    //yaw顺时针转动时角度增大，1,3电机转增大  
		                                        //  逆时针转动时角度减小  ，2，4电机转增大
		/*           1     4              */    //由于pid输出值是负的，所以顺时针转动时加载1，3上的值要加  -
		/*            \   /               */ 
		/*             \ /                */
		/*             / \                */
		/*            /   \               */
		/*           2     3              */
		/* 1:Moto_duty[0]  2:Moto_duty[1] */
    /* 3:Moto_duty[2]  4:Moto_duty[3] */
		
		if(Ultra.SetHigh.Mode==SetHigh_ing&&Plane_Mode.ARMED)
		{
			date_THROTTLE=(Ultra.AltHoldThro);// /cos(angle.roll/57.3)/cos(angle.pitch/57.3);  //油门倾角补偿，防止因倾斜高度下降太快;
			
			//怠速延时
			if( struct_delay.ms_idling>=1000)//5s
			{
				struct_delay.ms_idling_ok=true;
			}
			#ifndef ATTITUDE_PID_DEBUG  //如果没定义调姿态
			//怠速
			Plane_Idling(&struct_delay.ms_idling_ok,Moto_duty,1200);
			if(struct_delay.ms_idling_ok==true)//怠速完成
			#endif
			{
				//高度大于20cm时将电机偏置归0
				if(Ultra.z>20)
				{
					pitch_offset=0;//0.1;//-0.7;//0.2;//-0.5;//-2.9;//-1;  //前飘给正，后飘给负
					roll_offset=0.25;//0.25;//1;//1.1;    //左飘给正，右飘给负
				  //memset(moto,0,sizeof(moto));
				}
				else
				{
					pitch_offset=0;
					roll_offset=0;
//				Set_Moto_Compensate(moto,3,2,-2,-1);
			   Set_Moto_Compensate(moto,30,0,0,20);
				}
				#ifdef DOUBLE_PIDHOLD_DEBUG  //串级pid定高调试
				Moto_duty[0] =moto[0]+date_THROTTLE + Ultra.SetHigh.HighPidPwm-1000  + ctrl.roll.core.pid_out + ctrl.pitch.core.pid_out + ctrl.yaw.core.pid_out;
				Moto_duty[1] =moto[1]+date_THROTTLE + Ultra.SetHigh.HighPidPwm -1000  + ctrl.roll.core.pid_out - ctrl.pitch.core.pid_out - ctrl.yaw.core.pid_out;
				Moto_duty[2] =moto[2]+date_THROTTLE + Ultra.SetHigh.HighPidPwm -1000  - ctrl.roll.core.pid_out - ctrl.pitch.core.pid_out + ctrl.yaw.core.pid_out;
				Moto_duty[3] =moto[3]+date_THROTTLE + Ultra.SetHigh.HighPidPwm-1000  - ctrl.roll.core.pid_out + ctrl.pitch.core.pid_out - ctrl.yaw.core.pid_out;			
				#endif
				
				#ifdef PID_POS_DEBUG  //串级pid定点调试
				Moto_duty[0] =moto[0]+date_THROTTLE + Ultra.SetHigh.HighPidPwm-1000  + ctrl.roll.core.pid_out + ctrl.pitch.core.pid_out + ctrl.yaw.core.pid_out;
				Moto_duty[1] =moto[1]+date_THROTTLE + Ultra.SetHigh.HighPidPwm -1000  + ctrl.roll.core.pid_out - ctrl.pitch.core.pid_out - ctrl.yaw.core.pid_out;
				Moto_duty[2] =moto[2]+date_THROTTLE + Ultra.SetHigh.HighPidPwm -1000  - ctrl.roll.core.pid_out - ctrl.pitch.core.pid_out + ctrl.yaw.core.pid_out;
				Moto_duty[3] =moto[3]+date_THROTTLE + Ultra.SetHigh.HighPidPwm-1000  - ctrl.roll.core.pid_out + ctrl.pitch.core.pid_out - ctrl.yaw.core.pid_out;			
				#endif

				#ifdef ATTITUDE_PID_DEBUG  //姿态调试
				Moto_duty[0] =0;// date_THROTTLE  -1000  + ctrl.roll.core.pid_out + ctrl.pitch.core.pid_out + ctrl.yaw.core.pid_out;
				Moto_duty[1] = date_THROTTLE  -1000  + ctrl.roll.core.pid_out - ctrl.pitch.core.pid_out - ctrl.yaw.core.pid_out;
				Moto_duty[2] =0;// date_THROTTLE  -1000  - ctrl.roll.core.pid_out - ctrl.pitch.core.pid_out + ctrl.yaw.core.pid_out;
				Moto_duty[3] = date_THROTTLE  -1000  - ctrl.roll.core.pid_out + ctrl.pitch.core.pid_out - ctrl.yaw.core.pid_out;
				#endif
			}
		
		}
		
		if(Moto_duty[0]<=0) Moto_duty[0] = 0;
		if(Moto_duty[1]<=0) Moto_duty[1] = 0;
		if(Moto_duty[2]<=0) Moto_duty[2] = 0;
		if(Moto_duty[3]<=0) Moto_duty[3] = 0;
		
	}
	else  //如果收油
	{																																																																																																																																																																																																																																																																																																																																																																																																																																
		 Moto_duty[0] = Moto_duty[1] = Moto_duty[2] = Moto_duty[3] = 0;
		 ctrl.pitch.shell.increment = 0;
		 ctrl.roll.shell.increment = 0;
	}
	
	//如果是解锁状态
	if(Plane_Mode.ARMED)	
	{
		 Moto_PwmRflash(Moto_duty[0],Moto_duty[1],Moto_duty[2],Moto_duty[3]);
	}
	else  //上锁
	{
		 Moto_PwmRflash(0,0,0,0);
	}		
}

























