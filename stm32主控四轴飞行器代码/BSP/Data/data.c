#include "data.h"
#include "mode.h"
#include "menu.h"
u8 camera_image_buf[IMAGE_BUF_NUM]={0};  //ͼ�񻺳���
u8 USART3_TX_BUF[64];     //���ͻ���,���64���ֽ�.
//u8 SendPidFlag=0;//1:�յ���λ�����󣬷���Pid2

//���ͻ�������  18���ֽ�
void Data_Send_Status(const float rol,const float pit,const float yaw,const float Alt_CSB,const float Alt,const u8 ARMED)
{
	u8 sum = 0,i=0;
		vs16 _temp;
		vs32 _temp2 = Alt;//��ѹ�Ƹ߶�
		_temp = Alt_CSB;//�������߶�
	 USART3_TX_BUF[0]=0xAA;
	 USART3_TX_BUF[1]=0xAA;
	 USART3_TX_BUF[2]=0x01;
	 USART3_TX_BUF[3]=0X0D;
	
	_temp = (int)(rol*100);//���
	USART3_TX_BUF[4]=BYTE1(_temp);//�߰�λ����
	USART3_TX_BUF[5]=BYTE0(_temp);//ȡ�Ͱ�λ����
	_temp = (int)(pit*100);//����
	USART3_TX_BUF[6]=BYTE1(_temp);
	USART3_TX_BUF[7]=BYTE0(_temp);
	_temp = (int)(yaw*100);//ƫ��
	//_temp = (int)(Mag_Heading*100);
	USART3_TX_BUF[8]=BYTE1(_temp);
	USART3_TX_BUF[9]=BYTE0(_temp);

	USART3_TX_BUF[10]=BYTE1(_temp);//�������߶�
	USART3_TX_BUF[11]=BYTE0(_temp);

	USART3_TX_BUF[12]=BYTE3(_temp2);//��ѹ�Ƹ߶�
	USART3_TX_BUF[13]=BYTE2(_temp2);
	USART3_TX_BUF[14]=BYTE1(_temp2);
	USART3_TX_BUF[15]=BYTE0(_temp2);
//�����ж�		
	if(Plane_Mode.ARMED==false)	
		USART3_TX_BUF[16]=0xA0;	//����
	else if(Plane_Mode.ARMED==true)	
		USART3_TX_BUF[16]=0xA1;  //����
	
	for( i=0;i<17;i++)
		sum += USART3_TX_BUF[i];  //��У��ȡsum�ĵͰ�λ
	USART3_TX_BUF[17]=sum;

}

/********���ʹ���������*********/  //23���ֽ�
void Data_Send_Senser(const int ax,const int ay,const int az,const int gx,const int gy,const int gz,const int mx,const int my,const int mz)
{
		u8 i=0,sum=0;
		USART3_TX_BUF[0]=0xAA;
		USART3_TX_BUF[1]=0xAA;
		USART3_TX_BUF[2]=0x02;
		USART3_TX_BUF[3]=0x12;
	
		USART3_TX_BUF[4]=BYTE1(ax);  //���ٶ�   //�߰�λ
		USART3_TX_BUF[5]=BYTE0(ax);            //�Ͱ�λ
		USART3_TX_BUF[6]=BYTE1(ay);
		USART3_TX_BUF[7]=BYTE0(ay);
		USART3_TX_BUF[8]=BYTE1(az);
		USART3_TX_BUF[9]=BYTE0(az);
	
		USART3_TX_BUF[10]=BYTE1(gx);//������   
		USART3_TX_BUF[11]=BYTE0(gx);
		USART3_TX_BUF[12]=BYTE1(gy);
		USART3_TX_BUF[13]=BYTE0(gy);
		USART3_TX_BUF[14]=BYTE1(gz);
		USART3_TX_BUF[15]=BYTE0(gz);
	
		USART3_TX_BUF[16]=BYTE1(mx);  //�ش�
		USART3_TX_BUF[17]=BYTE0(mx);
		USART3_TX_BUF[18]=BYTE1(my);
		USART3_TX_BUF[19]=BYTE0(my);
		USART3_TX_BUF[20]=BYTE1(mz);
		USART3_TX_BUF[21]=BYTE0(mz);
	
	
	for(i=0;i<22;i++)
		sum += 	USART3_TX_BUF[i];
		USART3_TX_BUF[22] = sum;

}

/***********************����PID2����************/
void Data_Send_PID2(float alt_p,float alt_i,float alt_d)
{
	
		u8 sum = 0;
		vs16 _temp;
	USART3_TX_BUF[0]=0xAA;
	USART3_TX_BUF[1]=0xAA;
	USART3_TX_BUF[2]=0x10;
	USART3_TX_BUF[3]=0x12;
	
	_temp = alt_p * 100;
	USART3_TX_BUF[4]=BYTE1(_temp);
	USART3_TX_BUF[5]=BYTE0(_temp);
	_temp = alt_i * 100;
	USART3_TX_BUF[6]=BYTE1(_temp);
	USART3_TX_BUF[7]=BYTE0(_temp);
	_temp = alt_d * 100;
	USART3_TX_BUF[8]=BYTE1(_temp);
	USART3_TX_BUF[9]=BYTE0(_temp);

	USART3_TX_BUF[10]=0;
	USART3_TX_BUF[11]=0;

		USART3_TX_BUF[12]=0;
	USART3_TX_BUF[13]=0;

			USART3_TX_BUF[14]=0;
	USART3_TX_BUF[15]=0;

		USART3_TX_BUF[16]=0;
	USART3_TX_BUF[17]=0;

	USART3_TX_BUF[18]=0;
	USART3_TX_BUF[19]=0;
	
	USART3_TX_BUF[20]=0;
	USART3_TX_BUF[21]=0;
	
	for(u8 i=0;i<22;i++)
		sum += USART3_TX_BUF[i];
	
	USART3_TX_BUF[22]=sum;
	

}


void Data_Send_Image_Par(u8 * USART_TX_BUF,u8 function,const int ax,const int ay)
{
	u8 i=0,sum=0;
	
	USART_TX_BUF[0]=0xAA;

	USART_TX_BUF[1]=function;//����֡
	USART_TX_BUF[2]=0x04;//����λ���ֽ���
	
	USART_TX_BUF[3]=BYTE1(ax);  //�߰�λ
	USART_TX_BUF[4]=BYTE0(ax);  //�Ͱ�λ

	USART_TX_BUF[5]=BYTE1(ay);
	USART_TX_BUF[6]=BYTE0(ay);
	
	
	for(i=0;i<7;i++)
	  sum+=USART_TX_BUF[i];
	USART_TX_BUF[7]=sum;

	USART_TX_BUF[8]=0xff;
	
}


void Track_Mode_Send()
{
  if(tracking_offset.track_mode==END_TRACK)
	{
	   Data_Send_Image_Par(Image_Par_buf,ORDER_FRAME, 0x12,0);
	}
	else if(tracking_offset.track_mode==START_TRACK)
	{
	  Data_Send_Image_Par(Image_Par_buf,ORDER_FRAME, 0x13,0);
	}
	else if(tracking_offset.track_mode==TRACK_ING)
	{
		Data_Send_Image_Par(Image_Par_buf,ORDER_FRAME, 0x14,0);
	}
}
/*************������λ������***************/
extern u16 moto[4];
void PidDataReceive( )
{
//	�˺����ﲻ�ܼ�USART3_RX_STA=0;�����ж����Mode_Select(&USART3_RX_BUF[4],&Plane_Mode);	�ͽ�������
	//�Ҵ˺���ִ��ʱ��ϳ����ܷ��ж���
	if( (USART3_RX_BUF[0]==0XAA)&(USART3_RX_BUF[1]=0XAF))  
	{

		if(USART3_RX_BUF[2]==0X10)//0X10֡
		{
			#ifdef DOUBLE_PIDHOLD_DEBUG  //����pid����
			sonar_shell_pid.P=( int16_t)(vs16)(USART3_RX_BUF[4]<<8|USART3_RX_BUF[5]);
			sonar_shell_pid.I=( int16_t)(vs16)(USART3_RX_BUF[6]<<8|USART3_RX_BUF[7]);
			sonar_shell_pid.D=( int16_t)(vs16)(USART3_RX_BUF[8]<<8|USART3_RX_BUF[9]);
			
			sonar_core_pid.P=( int16_t)(vs16)(USART3_RX_BUF[10]<<8|USART3_RX_BUF[11]);
			sonar_core_pid.I=( int16_t)(vs16)(USART3_RX_BUF[12]<<8|USART3_RX_BUF[13]);
			sonar_core_pid.D=( int16_t)(vs16)(USART3_RX_BUF[14]<<8|USART3_RX_BUF[15]);
			
			Ultra.AltHoldThro =(vs16)(USART3_RX_BUF[16]<<8|USART3_RX_BUF[17]);
			Ultra.Add_Thro=(vs16)(USART3_RX_BUF[18]<<8|USART3_RX_BUF[19]);	
			Ultra.FINAL_HIGH =(vs16)(USART3_RX_BUF[20]<<8|USART3_RX_BUF[21]);
			
			Data_Send_Check(USART3_RX_BUF[22],USART3_TX_BUF);
			usart3_Send_Data(USART3_TX_BUF,8);     //����1����8���ֽ� 
			#endif
			#ifdef PID_POS_DEBUG
			pos_pid_ctr.PosPid_X.Shell.P  = ( float)(vs16)(USART3_RX_BUF[4]<<8|USART3_RX_BUF[5])/100.00;    //5
	    pos_pid_ctr.PosPid_X.Shell.I  = ( float)(vs16)(USART3_RX_BUF[6]<<8|USART3_RX_BUF[7])/1000.0;//0.01
	    pos_pid_ctr.PosPid_X.Shell.D  = ( float)(vs16)(USART3_RX_BUF[8]<<8|USART3_RX_BUF[9])/100.00;  
	
	    pos_pid_ctr.PosPid_X.Core.P = ( float)(vs16)(USART3_RX_BUF[10]<<8|USART3_RX_BUF[11])/100.00; 
      pos_pid_ctr.PosPid_X.Core.I = ( float)(vs16)(USART3_RX_BUF[12]<<8|USART3_RX_BUF[13])/1000.00;			
	    pos_pid_ctr.PosPid_X.Core.D = ( float)(vs16)(USART3_RX_BUF[14]<<8|USART3_RX_BUF[15])/100.00;  
			
		//	roll_offset  = ( float)(vs16)(USART3_RX_BUF[16]<<8|USART3_RX_BUF[17]);

			Data_Send_Check(USART3_RX_BUF[22],USART3_TX_BUF);
			usart3_Send_Data(USART3_TX_BUF,8);     //����1����8���ֽ� 
			
			#endif
			#ifdef ATTITUDE_PID_DEBUG
		  ctrl.pitch.shell.kp = ( float)(vs16)(USART3_RX_BUF[4]<<8|USART3_RX_BUF[5])/100.00;    //5
	    ctrl.pitch.shell.ki = ( float)(vs16)(USART3_RX_BUF[6]<<8|USART3_RX_BUF[7])/10000.0;//0.01
	    ctrl.pitch.shell.kd = ( float)(vs16)(USART3_RX_BUF[8]<<8|USART3_RX_BUF[9])/100.00;  
	
	    ctrl.pitch.core.kp = ( float)(vs16)(USART3_RX_BUF[10]<<8|USART3_RX_BUF[11])/100.00; 
      ctrl.pitch.core.ki = ( float)(vs16)(USART3_RX_BUF[12]<<8|USART3_RX_BUF[13])/10000.00;			
	    ctrl.pitch.core.kd = ( float)(vs16)(USART3_RX_BUF[14]<<8|USART3_RX_BUF[15])/10000.00;  
			
			Ultra.AltHoldThro =(vs16)(USART3_RX_BUF[16]<<8|USART3_RX_BUF[17]);
      roll_offset  = ( float)(vs16)(USART3_RX_BUF[18]<<8|USART3_RX_BUF[19])/100;
			
			Data_Send_Check(USART3_RX_BUF[22],USART3_TX_BUF);
			usart3_Send_Data(USART3_TX_BUF,8);     //����1����8���ֽ� 
			#endif
		}
		if(USART3_RX_BUF[2]==0X11)//0X11֡
		{
			
//			#ifdef DOUBLE_PIDHOLD_DEBUG  //����pid����
//			moto[0] = (vs16)(USART3_RX_BUF[4]<<8|USART3_RX_BUF[5]); 
//	    moto[1] = (vs16)(USART3_RX_BUF[6]<<8|USART3_RX_BUF[7]);
//	    moto[2] = (vs16)(USART3_RX_BUF[8]<<8|USART3_RX_BUF[9]);  
//	
//	    moto[3] = (vs16)(USART3_RX_BUF[10]<<8|USART3_RX_BUF[11]); 
//     // pos_pid_ctr.PosPid_Y.Core.I = ( float)(vs16)(USART3_RX_BUF[12]<<8|USART3_RX_BUF[13]);			
//	    //pos_pid_ctr.PosPid_Y.Core.D = ( float)(vs16)(USART3_RX_BUF[14]<<8|USART3_RX_BUF[15]);  
//			
//			Data_Send_Check(USART3_RX_BUF[22],USART3_TX_BUF);
//			usart3_Send_Data(USART3_TX_BUF,8);     //����1����8���ֽ� 
//			#endif
			#ifdef PID_POS_DEBUG
			pos_pid_ctr.PosPid_Y.Shell.P  = ( float)(vs16)(USART3_RX_BUF[4]<<8|USART3_RX_BUF[5])/100.00;    //5
	    pos_pid_ctr.PosPid_Y.Shell.I  = ( float)(vs16)(USART3_RX_BUF[6]<<8|USART3_RX_BUF[7])/1000.0;//0.01
	    pos_pid_ctr.PosPid_Y.Shell.D  = ( float)(vs16)(USART3_RX_BUF[8]<<8|USART3_RX_BUF[9])/100.00;  
	
	    pos_pid_ctr.PosPid_Y.Core.P = ( float)(vs16)(USART3_RX_BUF[10]<<8|USART3_RX_BUF[11])/100.00; 
      pos_pid_ctr.PosPid_Y.Core.I = ( float)(vs16)(USART3_RX_BUF[12]<<8|USART3_RX_BUF[13])/1000.00;			
	    pos_pid_ctr.PosPid_Y.Core.D = ( float)(vs16)(USART3_RX_BUF[14]<<8|USART3_RX_BUF[15])/100.00;  
			
			Data_Send_Check(USART3_RX_BUF[22],USART3_TX_BUF);
			usart3_Send_Data(USART3_TX_BUF,8);     //����1����8���ֽ� 
			#endif
			
			#ifdef ATTITUDE_PID_DEBUG
			ctrl.roll.shell.kp =  (float)(vs16)(USART3_RX_BUF[4]<<8|USART3_RX_BUF[5])/100.00;
	    ctrl.roll.shell.ki =  (float)(vs16)(USART3_RX_BUF[6]<<8|USART3_RX_BUF[7])/1000.00;
	    ctrl.roll.shell.kd =  (float)(vs16)(USART3_RX_BUF[8]<<8|USART3_RX_BUF[9])/100.00;

	    ctrl.roll.core.kp  =  (float)(vs16)(USART3_RX_BUF[10]<<8|USART3_RX_BUF[11])/100.00;
			ctrl.roll.core.ki  =  (float)(vs16)(USART3_RX_BUF[12]<<8|USART3_RX_BUF[13])/1000.00;
	    ctrl.roll.core.kd  =  (float)(vs16)(USART3_RX_BUF[14]<<8|USART3_RX_BUF[15])/10000.00; 
		  Data_Send_Check(USART3_RX_BUF[22],USART3_TX_BUF);
			usart3_Send_Data(USART3_TX_BUF,8);     //����1����8���ֽ� 
			#endif
		}
			if(USART3_RX_BUF[2]==0X12)//0X12֡
		{
			#ifdef ATTITUDE_PID_DEBUG
			ctrl.yaw.shell.kp =  ( float)(vs16)(USART3_RX_BUF[4]<<8|USART3_RX_BUF[5])/100.00;
	    ctrl.yaw.shell.ki = ( float)(vs16)(USART3_RX_BUF[6]<<8|USART3_RX_BUF[7])/1000.0;
	    ctrl.yaw.shell.kd =  ( float)(vs16)(USART3_RX_BUF[8]<<8|USART3_RX_BUF[9])/100.00;

	    ctrl.yaw.core.kp = ( float)(vs16)(USART3_RX_BUF[10]<<8|USART3_RX_BUF[11])/100.00;
			ctrl.yaw.core.ki = ( float)(vs16)(USART3_RX_BUF[12]<<8|USART3_RX_BUF[13])/1000.00;
	    ctrl.yaw.core.kd =( float)(vs16)(USART3_RX_BUF[14]<<8|USART3_RX_BUF[15])/100.00; 
		  Data_Send_Check(USART3_RX_BUF[22],USART3_TX_BUF);
			usart3_Send_Data(USART3_TX_BUF,8);     //����1����8���ֽ� 
			#endif
		}
	}

}
/************************************************************
��������ͷ��Ϣ
*************************************************************/
void Receive_CameraData(u8 *buf)
{
	u8 i=0,sum=0,j=0;
	for(i=0;i<64;i++)
	{
    if(buf[i]==0xAA)
		  break;
  }
   if(buf[i]==0XAA&&(buf[i+13]==0xff))//ȷ�����ݵ�׼ȷ�ԣ���ֹ���ݼ��λ��
	 {
		 for(j=0;j<12;j++)//���
		 {
       sum+=buf[i+j];
     }
		 if(sum==buf[i+12])//��У��
		 {
       if(buf[i+1]==0x51)
		   {
         cam_pos_ctr.pos_ctr_x.distance=(vs16)(buf[i+3]<<8|buf[i+4]);
				 cam_pos_ctr.pos_ctr_y.distance=(vs16)(buf[i+5]<<8|buf[i+6]);
         tracking_offset.offset        =(vs16)(buf[i+7]<<8|buf[i+8]);
				 tracking_offset.Check.receive_find   =buf[i+9];
				 tracking_offset.line_find     =(vs16)(buf[i+10]<<8|buf[i+11]);
				 
       }
	   }

	}

}

//����У����Ϣ
void Data_Send_Check(u16 check,u8 *data_to_send)
{
	data_to_send[0]=0xAA;
	data_to_send[1]=0xAA;
	data_to_send[2]=0xF0;
	data_to_send[3]=3;
	data_to_send[4]=0xBA;
	
	data_to_send[5]=0;//BYTE1(check);
	data_to_send[6]=check;//BYTE0(check);
	
	u8 sum = 769+check;
//	for(u8 i=0;i<7;i++)
//		sum += data_to_send[i];
	
	data_to_send[7]=sum;
}


/*********************************
����λ����������
*********************************/
extern  float target_speed;
extern u16 sonar_shell_iout,sonar_core_iout;
extern u16 sonar_shell_out;
void Report_Message(u8* _SendCount)
{
  if(*_SendCount==0)
	{
		Data_Send_Status(angle.roll,angle.pitch,angle.yaw,0,Ultra.z*100,Plane_Mode.ARMED);//���ͻ�������֡,�ɿ���pitch��roll�Ƿ��ģ�ѡ����y����ͷ
		usart3_Send_Data(USART3_TX_BUF,18);     //����1����18���ֽ� 
		*_SendCount=1;
	}
	else if(*_SendCount==1)  
	{
		#ifdef DOUBLE_PIDHOLD_DEBUG //����pid���ߵ��� 
		Data_Send_Senser( Ultra.ultra_v,sonar_shell_pid.setgoal, Ultra.SetHigh.HighPidPwm,sonar_shell_iout,sonar_core_iout,sonar_shell_out,sonar_shell_pid.static_inc.integral,angle.heading,Ultra.AltHoldThro);
	// Data_Send_Senser( Ultra.ultra_v,sonar_shell_pid.setgoal, Ultra.SetHigh.HighPidPwm,sonar_shell_iout,sonar_core_iout,moto[1],moto[2],moto[3],Ultra.AltHoldThro);

		#endif
		
		#ifdef ATTITUDE_PID_DEBUG  //��̬����
    Data_Send_Senser(ctrl.pitch.shell.ki_out,ctrl.pitch.core.ki_out ,ctrl.pitch.shell.pid_out,ctrl.pitch.core.pid_out,sensor.gyro.changed.x,sensor.gyro.changed.y,sensor.gyro.changed.z,Ultra.KS103High*100,Ultra.AltHoldThro);
   // Data_Send_Senser(sensor.acc.origin.x ,sensor.acc.origin.y,sensor.acc.origin.z,sensor.gyro.changed.x,sensor.gyro.changed.y,sensor.gyro.changed.z,TIM2->CCR1,TIM2->CCR3,ctrl.yaw.shell.increment);
	
		#endif
		
		#ifdef PID_POS_DEBUG      //��λ����
//			  Data_Send_Senser(tracking_offset.valid_offset,tracking_offset.offset,tracking_offset.off_v,
//		                trackline_roll[1], trackline_roll[2],
//                  	  trackline_roll[3], trackline_roll[4],trackline_roll[5],trackline_roll[6]);
//					  Data_Send_Senser(cam_pos_ctr.pos_ctr_y.distance,cam_pos_ctr.pos_ctr_y.valid_distance,roll[0],
//		                roll[1], roll[2],
//                  	  roll[3], roll[4],roll[5],0);
		
//	  Data_Send_Senser(cam_pos_ctr.pos_ctr_x.valid_distance,cam_pos_ctr.pos_ctr_y.valid_distance, cam_pos_ctr.pos_ctr_x.distance,
//											cam_pos_ctr.pos_ctr_y.distance,tracking_offset.track_mode*10,
//                  	tracking_offset.off_v, tracking_offset.offset,cam_pos_ctr.ROLL,	tracking_offset.goal_angle*100  );
	  Data_Send_Senser(cam_pos_ctr.pos_ctr_x.valid_distance,cam_pos_ctr.pos_ctr_y.valid_distance, cam_pos_ctr.pos_ctr_x.v  ,
											cam_pos_ctr.pos_ctr_y.v,tracking_offset.track_mode*10,
                  pos_pid_ctr.PosPid_X.Shell.I_OUT*100 , cam_pos_ctr.PITCH ,cam_pos_ctr.ROLL,	tracking_offset.goal_angle*100  );
		
//		  Data_Send_Senser( tracking_offset.track_mode,tracking_offset.circle_count, tracking_offset.offset ,
//		                 tracking_offset.off_v,cam_pos_ctr.pos_ctr_x.v,
//                  	tracking_offset.Change_yaw.final_alladd,0,angle.yaw,	angle.heading);
//		
		#endif
		
		usart3_Send_Data(USART3_TX_BUF,23);    //����1����23���ֽ�
		*_SendCount=0;			
	}		
}


//extern u8 USART1_ov7620_rxbuf[4];

//void position_data_decode()
//{
//	//�ϳ�λ������
//	cam_pos_ctr.pos_ctr_x.distance = ( USART1_ov7620_rxbuf[0] & 0x7f) << 8;
//	cam_pos_ctr.pos_ctr_x.distance += USART1_ov7620_rxbuf[1] & 0xff;				
//	cam_pos_ctr.pos_ctr_x.distance *= (USART1_ov7620_rxbuf[0] >> 7) > 0 ? -1 : 1;
//	
//	cam_pos_ctr.pos_ctr_y.distance = ( USART1_ov7620_rxbuf[2] & 0x7f) << 8;
//	cam_pos_ctr.pos_ctr_y.distance  += USART1_ov7620_rxbuf[3] & 0xff;
//	cam_pos_ctr.pos_ctr_y.distance  *= (USART1_ov7620_rxbuf[2] >> 7) > 0 ? -1 : 1;

//}



