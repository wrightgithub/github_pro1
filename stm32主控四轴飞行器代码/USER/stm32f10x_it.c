#include "stm32f10x_it.h"
#include "UserSys.h"
#include "mode.h"
u8 SendCount=0;
u16 ms0=0,ms1=0,ms2=0,ms3=0,ms4=0,ms50=0,ms7=0,ms8=0,ms6_oled;	//中断次数计数器
u16 ms1s=0;//测试数据用
u8 check_count=0;
bool sonar_valid = false;

u16 tick_count=0;
bool start_tick=false;
void TIM3_IRQHandler(void)		    //2.5ms中断一次
{
	if(TIM3->SR & TIM_IT_Update)	
	{    	
    TIM3->SR = ~TIM_FLAG_Update;//清除中断标志	
    UltraSendFlag++;	
    ms0++;		
		ms1++;
		ms2++;
		ms3++;
  	ms4++;
		ms6_oled++;
		ms50++;
		ms7++;
		ms8++;
		
		//测试1s接收到多少数据
		if(start_tick==true)
		{
			ms1s++;
			if(ms1s>=400)
			{
				ms1s=400;
			}
	  }
		//////////////////////////////
		
		//怠速延时
		if(struct_delay.ms_idling_ok==false)
		   struct_delay.ms_idling++;
		//超声波发送延时
		if(struct_delay.ms_ultra_ok==false)
		  struct_delay.ms_ultra++;

	
		if(ms0>=2)//5ms  //200hz
		{
			ms0=0;
			/***这两个必须在主函数里读***/
			if(!mpu_dmp_get_data(&angle.roll,&angle.pitch,&angle._yaw))//读取mpu6050的角度，角速度，加速度
			{		
				angle.yaw=constrain((180-angle._yaw),0,359);
				sensor.gyro.changed.x=(sensor.gyro.origin.x)*Gyro_G;
				sensor.gyro.changed.y=(sensor.gyro.origin.y)*Gyro_G;
				sensor.gyro.changed.z=(sensor.gyro.origin.z)*Gyro_G;
			}
			
       //姿态控制核心算法		//控制频率200hz
		   CONTROL(angle.roll,angle.pitch,angle.yaw); 
		}
		
		
			//保护措施
			#ifndef ATTITUDE_PID_DEBUG //如果没定义调姿态
			if(FL_ABS(angle.roll)>=30||FL_ABS(angle.pitch)>=30)
			{
				//上锁
				Plane_Mode.ARMED=0;
				Plane_Mode.unlock  =false;
			}
			#endif
			
		 //自动模式下的落地判断
			AutoLand_Mode(&Ultra.z); 
			
			//状态机
			State_Machine(&fly_mode);
		
	
	
	}
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/
u8 USART3_RX_BUF[50];
u8 USART3_RX_STA = 0;
void USART3_IRQHandler(void)
{
		u8 res;		
	if(USART3->SR&(1<<5))//接收到数据
	{	 
		res=USART3->DR;	
		USART3_RX_BUF[USART3_RX_STA]=res;
		USART3_RX_STA++;
		/*****************************/
		//一下若放在USART3_RX_STA++;之前则要(USART3_RX_STA&0X3F)==(USART3_RX_BUF[3]+4)舍弃最后的校验位
		//所以放在	USART3_RX_STA++;之后 ，并且命令函数Mode_Select(&USART3_RX_BUF[4],&Plane_Mode);得响应快所以放在中断里
		if((USART3_RX_BUF[0]!=0xAA)||(USART3_RX_BUF[1]!=0XAF))
		{
      USART3_RX_STA=0;//从头开始
			return ;
    }
		if((USART3_RX_STA)==(USART3_RX_BUF[3]+5))
		{
			 if( (USART3_RX_BUF[0]==0XAA)&(USART3_RX_BUF[1]=0XAF))  
	     {
					if(USART3_RX_BUF[2]==0X01)//0X01帧
					{
						Mode_Select(&USART3_RX_BUF[4],&Plane_Mode);					
						memset(USART3_RX_BUF,0,6);//必须把前六个字节置零
					}
			 }
			 
			USART3_RX_STA=0;//从头开始
		}	
		else if(USART3_RX_STA>50)//防止丢包导致数据溢出
		{
		  USART3_RX_STA=0;
		}
	} 	 
}


//接收缓存区 	
#define ks103max 18446744073709551615
u8  USART2_RX_BUF[64];  	//接收缓冲,最大64个字节.
u8  USART2_TX_BUF[2];   //发送缓冲
u8  USART2_RX_STA=0;//标记接收到一帧数据；
uint64_t  ks103_count=0;
void USART2_IRQHandler(void)
{
	u8 res;	
 static float last_high=0;	
//	if(USART2->SR&(1<<5))//接收到数据
//	{	 
//			res=USART2->DR;
//		  ks103_count= USART2_RX_STA;
//			Ultra.KS103.KS103_RX[(ks103_count^1)]=res;
//			USART2_RX_STA++;
//		  if(USART2_RX_STA>=2)
//			{
//				USART2_RX_STA=0;
//				Ultra.KS103High=Ultra.KS103.High/1000.00;
//			}				
//  }
	if(USART2->SR&(1<<5))//接收到数据
	{	 
				res=USART2->DR; 
			if((USART2_RX_STA&0X3F)==2)
			{
				Ultra.KS103High=(USART2_RX_BUF[0]<<8|USART2_RX_BUF[1])/1000.00;
				
				if(FL_ABS(Ultra.KS103High - last_high)>=0.35)//30cm
				{
				  Ultra.KS103High=0;
				}
				else
				{
				  last_high=Ultra.KS103High;
				}
				
				if( ks103_count%2==0)
				  USART2_RX_STA=0;
			}		
			USART2_RX_BUF[USART2_RX_STA&0X3F]=res;
			USART2_RX_STA++;
			
			ks103_count++;
		  if(ks103_count==2555555551)
				ks103_count=1;	
	}			
} 




u8 USART1_RX_BUF[64];
//接收状态
//bit7，接收完成标志 0未完成 1:完成
//bit6，接收到0x0d
//bit5~0，接收到的有效字节数目
u8 USART1_RX_STA = 0;
bool StartReceive=false;
bool start_receive_img;
u16 pix_count=0;
u16 rx_start=0;
void USART1_IRQHandler(void)
{
	u8 res;
	if(USART1->SR&(1<<5))//接收到数据
	{	 
		//LED4(ON);
		res=USART1->DR;
		if(Plane_Mode.Receive_Img==false)
		{			
			if((USART1_RX_STA&0X3F)==USART1_RX_BUF[2]+5)
			{
				//USART1_RX_BUF[USART1_RX_STA&0X3F]=0xff;
			//接收摄像头数据
				Receive_CameraData(USART1_RX_BUF); 
				
				//测试1s接收到多少数据
				if(ms1s<400)
				{
					tick_count++;
				}
				else if(ms1s==400)
				{
					tick_count=tick_count;
				}
				
				StartReceive=false;
				//USART1_RX_STA=0;
			}
			if(res==0xAA)
			{
				StartReceive=true;
				USART1_RX_STA=0;
			}
			
			if(StartReceive==true)
			{
				USART1_RX_BUF[USART1_RX_STA&0X3F]=res;
				USART1_RX_STA++;
				
			}		 
		} 
		else if( (Plane_Mode.Receive_Img==true)&&(Plane_Mode.Rec_Img_ok==false) )
		{
			
			if(start_receive_img==false)
			{
				//头码检验
				 if( (res==0x13)&&(rx_start==0) )//开始一幅图像的接收
				 {
					 rx_start++;//1
				 }
				 else if((res==0x14)&&(rx_start==1))
				 {
					 rx_start++;//2
				 }
				 else if((res==0x41)&&(rx_start==2))
				 {
					 pix_count=0;
					 start_receive_img=true;
				 }
		 }
		  else if(start_receive_img==true)
			{

					camera_image_buf[pix_count]=res;
					pix_count++;
					if(pix_count>=IMAGE_BUF_NUM)
					{
						Plane_Mode.Rec_Img_ok=true;
						pix_count=IMAGE_BUF_NUM;
					}
			}				
		}
	}		

}






/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
		
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
