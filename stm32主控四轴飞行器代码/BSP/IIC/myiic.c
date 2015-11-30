#include "myiic.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//IIC ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/6
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
void SDA_IN(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
	  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
} 

void SDA_OUT(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;	
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE);
	
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}
//��ʼ��IIC
void IIC_Init(void)
{			
//  GPIO_InitTypeDef  GPIO_InitStructure;

//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��GPIOBʱ��

//  //GPIOB8,B9��ʼ������
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//��ͨ���ģʽ
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
//  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��
//	SCL_H;
//	SDA_H;
	
	
		GPIO_InitTypeDef gpio;
	//�Ѹ���
	  RCC->APB2ENR|=1<<3;
    gpio.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &gpio);
    SDA_H;	     //����������
    SCL_H;	     //����ʱ����
}
//����IIC��ʼ�ź�
void IIC_Start(void)
{
	SDA_OUT();     //sda�����
	SDA_H;	  	  
	SCL_H;
	delay_us(4);
 	SDA_L;//START:when CLK is high,DATA change form high to low 
	delay_us(4);
	SCL_L;//ǯסI2C���ߣ�׼�����ͻ�������� 
}	  
//����IICֹͣ�ź�
void IIC_Stop(void)
{
	SDA_OUT();//sda�����
	SCL_L;
	SDA_L;//STOP:when CLK is high DATA change form low to high
 	delay_us(4);
	SCL_H; 
	SDA_H;//����I2C���߽����ź�
	delay_us(4);							   	
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA����Ϊ����  
	SDA_H;delay_us(1);	   
	SCL_H;delay_us(1);	 
	while(SDA_read)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	SCL_L;//ʱ�����0 	   
	return 0;  
} 
//����ACKӦ��
void IIC_Ack(void)
{
	SCL_L;
	SDA_OUT();
	SDA_L;
	delay_us(2);
	SCL_H;
	delay_us(2);
	SCL_L;
}
//������ACKӦ��		    
void IIC_NAck(void)
{
	SCL_L;
	SDA_OUT();
	SDA_H;
	delay_us(2);
	SCL_H;
	delay_us(2);
	SCL_L;
}					 				     
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��			  
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;  	
	  SDA_OUT(); 	    
    SCL_L;//����ʱ�ӿ�ʼ���ݴ���
    for(t=0;t<8;t++)
    {        
      
       // IIC_SDA=(txd&0x80)>>7;
			u8 n=(txd&0x80)>>7;
			if(n==1)
			{
				SDA_H;
			}
			else if(n==0)
			{
				SDA_L;
	    }		
			txd<<=1; 	  
			delay_us(2);   //��TEA5767��������ʱ���Ǳ����
			SCL_H;
			delay_us(2); 
			SCL_L;	
			delay_us(2);
    }	 
} 	    
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK   
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA����Ϊ����
  for(i=0;i<8;i++ )
	{
        SCL_L; 
        delay_us(2);
		    SCL_H;
        receive<<=1;
        if(SDA_read) receive++;   
		    delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK   
    return receive;
}



























