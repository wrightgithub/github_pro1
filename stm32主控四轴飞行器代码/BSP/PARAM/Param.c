#include "UserSys.h"


//**************************************************************************
//参数加载
//**************************************************************************
void	paramLoad(void)
{
	// The data of pitch
	ctrl.pitch.shell.kp =4.0;//4.5;//4.5;//4.0;//2.2;// 2.5;//1.9;//5.0;//3.6;    //5
	ctrl.pitch.shell.ki = 0.012;//0.003;//0.0025;//0.01
	ctrl.pitch.shell.kd = 0;//2;    //1.5
	
	ctrl.pitch.core.kp = 0.54;//0.6;//0.7;//0.41;//0.38;//0.58;//0.4;//0.4;//0.8;//1.05;   
	ctrl.pitch.core.ki = 0.04;//0.08;//0.008;//0.02;//0.001;
	ctrl.pitch.core.kd =1.8;//1.5;//0.018;//0.016;//0.006;//0.028;//0.028;//1.8;//1.8;//1.45;//9.10;  //0.05
	
	//The data of roll
	ctrl.roll.shell.kp = 4.0;//4.5;//4.0;//2.2;//2.5;//1.9;//5.0;
	ctrl.roll.shell.ki = 0.012;//0.01;//0.003;
	ctrl.roll.shell.kd = 0;//2;

	ctrl.roll.core.kp =0.54;//0.6;//0.41;//0.38;//0.58;//0.4;//0.4;// 0.38;//0.8;  
	ctrl.roll.core.ki = 0.04;//0.08;//0.008;
	ctrl.roll.core.kd =1.8;//1.5;//0.018;//0.016;//0.006;// 0.028;//0.028;//1.8;//1.8;//1.45;//9.10;
	
	//The data of yaw
	angle.heading=180;
	ctrl.yaw.shell.kp =2;//3;// 3.6;
	ctrl.yaw.shell.ki= 0.005;//0.01
	ctrl.yaw.shell.kd =0;// 2;//0.12;
	
	ctrl.yaw.core.kp = 3.5;//1;//2;//1.4;
	ctrl.yaw.core.ki = 0.05;
	ctrl.yaw.core.kd = 0.5;//1;//0.08;
	
	
	//limit for the max increment
	ctrl.pitch.shell.increment_max = 240;
	ctrl.roll.shell.increment_max  = 240;
	ctrl.yaw.shell.increment_max   = 200;
	
	ctrl.ctrlRate = 0;//串级pid控制频率

}



