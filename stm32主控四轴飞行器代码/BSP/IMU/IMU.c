#include "IMU.h"
#include "math.h"

float mag=0;
struct _angle angle;


/*	
	Q:过程噪声，Q增大，动态响应变快，收敛稳定性变坏
	R:测量噪声，R增大，动态响应变慢，收敛稳定性变好	
*/
#define KALMAN_Q        0.02
#define KALMAN_R        7.0000
/*           卡尔曼对三个轴加速度进行滤波处理           */
 float KalmanFilter_KS103_v(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R)
{
   float R = MeasureNoise_R;
   float Q = ProcessNiose_Q;
   static float x_last;
   float x_mid = x_last;
   float x_now;
   static float p_last;
   float p_mid ;
   float p_now;
   float kg;        

	 x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)  //x_last:上一次的最优预测值，x_mid:现在从上一次来的预测值
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声   //p_mid：本次从上一次来的协方差，p_last:上次的协方差
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声     //kg卡尔曼增益   ResrcData：仪器的测量值
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值    //本次的最优预测值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance        //本次的协方差
   p_last = p_now; //更新covariance值               //以下两句完成递归
   x_last = x_now; //更新系统状态值
   return x_now;   //返回最优估计值          
 }

 float KalmanFilter_KS103_high(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R)
{
   float R = MeasureNoise_R;
   float Q = ProcessNiose_Q;
   static float x_last;
   float x_mid = x_last;
   float x_now;
   static float p_last;
   float p_mid ;
   float p_now;
   float kg;        

	 x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)  //x_last:上一次的最优预测值，x_mid:现在从上一次来的预测值
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声   //p_mid：本次从上一次来的协方差，p_last:上次的协方差
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声     //kg卡尔曼增益   ResrcData：仪器的测量值
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值    //本次的最优预测值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance        //本次的协方差
   p_last = p_now; //更新covariance值               //以下两句完成递归
   x_last = x_now; //更新系统状态值
   return x_now;   //返回最优估计值          
 }
 
  float KalmanFilter_Tracking_Offset(const float ResrcData,float ProcessNiose_Q,float MeasureNoise_R)
{
   float R = MeasureNoise_R;
   float Q = ProcessNiose_Q;
   static float x_last;
   float x_mid = x_last;
   float x_now;
   static float p_last;
   float p_mid ;
   float p_now;
   float kg;        

	 x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)  //x_last:上一次的最优预测值，x_mid:现在从上一次来的预测值
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=噪声   //p_mid：本次从上一次来的协方差，p_last:上次的协方差
   kg=p_mid/(p_mid+R); //kg为kalman filter，R为噪声     //kg卡尔曼增益   ResrcData：仪器的测量值
   x_now=x_mid+kg*(ResrcData-x_mid);//估计出的最优值    //本次的最优预测值
                
   p_now=(1-kg)*p_mid;//最优值对应的covariance        //本次的协方差
   p_last = p_now; //更新covariance值               //以下两句完成递归
   x_last = x_now; //更新系统状态值
   return x_now;   //返回最优估计值          
 }

float KalmanFilter_dis_x(const float ResrcData, float a)
{
	static float last_dis_x=0;
	float filter_data=  (1-a)*last_dis_x+a*ResrcData;
	last_dis_x=filter_data;
	return filter_data;    
}



float KalmanFilter_dis_y(const float ResrcData, float a)
{
	static float last_dis_y=0;
	float filter_data=  (1-a)*last_dis_y+a*ResrcData;
	last_dis_y=filter_data;
	return filter_data;    
}


 
//   快速求平方根倒数
float Q_rsqrt(float number)
{
	long i;
	float x2, y;
	const float threehalfs = 1.5F;
 
	x2 = number * 0.5F;
	y  = number;
	i  = * ( long * ) &y;                      
	i  = 0x5f3759df - ( i >> 1 );               
	y  = * ( float * ) &i;
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration （第一次牛顿迭代）
	return y;
} 
//  求float型数据绝对值
float FL_ABS(float x)
{
   if(x < 0)  return -x;
	 else return x; 
}

int Int_Fabs(float x)
{
   if(x < 0)  return -x;
	 else return x; 
}

/*   采用三角函数的泰勒展开式 求近似值*/
float COS(float x)
{
	float result;
  result = 1 - x * x/2;
	return result; 
}

float SIN(float y)
{
	float result;
  result = y - y * y * y /6;
	return result; 
}
/***********************************************
  * @brief  可变增益自适应参数
  * @param  None
  * @retval None
************************************************/
float VariableParameter(float error)
{
	float  result = 0;
	
	if(error < 0)
	{
	   error = -error;
	}
  if(error >0.8f)
	{
	   error = 0.8f;
	}
	result = 1 - 1.25 * error;
	if(result < 0)
	{
	   result = 0;
	}
	return result;
}













