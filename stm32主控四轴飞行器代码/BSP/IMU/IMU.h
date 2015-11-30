#ifndef __IMU_H
#define	__IMU_H
#include "stm32f10x.h"

#define RtA 		57.324841		//  180/3.1415  角度制 转化为弧度制		
#define AtR    	0.0174533		//  1/RtA             RtA倒数		
#define Acc_G 	0.0011963		//  1/32768/4/9.8     加速度量程为4G		
#define Gyro_G  0.061035156	//  1/32768/2000     陀螺仪量程为 +—2000			
struct _angle
{
	float pitch;
	float roll;
	float _yaw;
	float yaw;
	float heading;
};

extern struct _angle angle;

void Prepare_Data(void);
void Get_Attitude(void);
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az);
 float KalmanFilter_KS103_v(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R);
float KalmanFilter_dis_x(const float ResrcData, float a);
float KalmanFilter_dis_y(const float ResrcData, float a);
float FL_ABS(float x);
int Int_Fabs(float x);
 float KalmanFilter_Tracking_Offset(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R);
 float KalmanFilter_KS103_high(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R);
#endif













