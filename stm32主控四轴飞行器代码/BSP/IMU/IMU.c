#include "IMU.h"
#include "math.h"

float mag=0;
struct _angle angle;


/*	
	Q:����������Q���󣬶�̬��Ӧ��죬�����ȶ��Ա仵
	R:����������R���󣬶�̬��Ӧ�����������ȶ��Ա��	
*/
#define KALMAN_Q        0.02
#define KALMAN_R        7.0000
/*           ����������������ٶȽ����˲�����           */
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

	 x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)  //x_last:��һ�ε�����Ԥ��ֵ��x_mid:���ڴ���һ������Ԥ��ֵ
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����   //p_mid�����δ���һ������Э���p_last:�ϴε�Э����
   kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����     //kg����������   ResrcData�������Ĳ���ֵ
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ    //���ε�����Ԥ��ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance        //���ε�Э����
   p_last = p_now; //����covarianceֵ               //����������ɵݹ�
   x_last = x_now; //����ϵͳ״ֵ̬
   return x_now;   //�������Ź���ֵ          
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

	 x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)  //x_last:��һ�ε�����Ԥ��ֵ��x_mid:���ڴ���һ������Ԥ��ֵ
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����   //p_mid�����δ���һ������Э���p_last:�ϴε�Э����
   kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����     //kg����������   ResrcData�������Ĳ���ֵ
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ    //���ε�����Ԥ��ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance        //���ε�Э����
   p_last = p_now; //����covarianceֵ               //����������ɵݹ�
   x_last = x_now; //����ϵͳ״ֵ̬
   return x_now;   //�������Ź���ֵ          
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

	 x_mid=x_last; //x_last=x(k-1|k-1),x_mid=x(k|k-1)  //x_last:��һ�ε�����Ԥ��ֵ��x_mid:���ڴ���һ������Ԥ��ֵ
	p_mid=p_last+Q; //p_mid=p(k|k-1),p_last=p(k-1|k-1),Q=����   //p_mid�����δ���һ������Э���p_last:�ϴε�Э����
   kg=p_mid/(p_mid+R); //kgΪkalman filter��RΪ����     //kg����������   ResrcData�������Ĳ���ֵ
   x_now=x_mid+kg*(ResrcData-x_mid);//���Ƴ�������ֵ    //���ε�����Ԥ��ֵ
                
   p_now=(1-kg)*p_mid;//����ֵ��Ӧ��covariance        //���ε�Э����
   p_last = p_now; //����covarianceֵ               //����������ɵݹ�
   x_last = x_now; //����ϵͳ״ֵ̬
   return x_now;   //�������Ź���ֵ          
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


 
//   ������ƽ��������
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
	y  = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration ����һ��ţ�ٵ�����
	return y;
} 
//  ��float�����ݾ���ֵ
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

/*   �������Ǻ�����̩��չ��ʽ �����ֵ*/
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
  * @brief  �ɱ���������Ӧ����
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













