#include "usart.h"
void usart3_config(u32 bound )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
		/* config USART3 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //ʹ��AFIO���ܵ�ʱ��
  GPIO_PinRemapConfig(GPIO_FullRemap_USART3 ,ENABLE);  //������ӳ��
	
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	    
  /* Configure USART2 Rx (PD.09) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	  USART_InitStructure.USART_BaudRate = bound;                 /*���ò�����Ϊ115200*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  /*��������λΪ8λ*/
    USART_InitStructure.USART_StopBits = USART_StopBits_1;       /*����ֹͣλΪ1λ*/
    USART_InitStructure.USART_Parity = USART_Parity_No;          /*����żУ��*/    
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*û��Ӳ������*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;      /*���������*/
    /*��ɴ���COM3��ʱ�����á�GPIO���ã���������������ʼ����ʹ��*/
	
	USART_Init(USART3, &USART_InitStructure);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);
}

/**************************ʵ�ֺ���********************************************
*����ԭ��:		void UART3_Put_Char(unsigned char DataToSend)
*��������:		��������һ���ֽ�
���������
		unsigned char DataToSend   Ҫ���͵��ֽ�����
���������û��	
*******************************************************************************/
uint8_t UART3_Put_Char(unsigned char DataToSend)
{
	//��Ҫ���͵��ֽ�д��UART1�ķ��ͻ�����
	USART_SendData(USART3, (unsigned char) DataToSend);
	//�ȴ��������
  	while (!(USART3->SR & USART_FLAG_TXE));
	return DataToSend;
}





/********************************************************
         my_usart
********************************************************/

/*************����3���ݷ���****************/
void usart3_Send_Data(u8 *buf,u8 len)
{
	u8 t;
  for(t=0;t<len;t++)		//ѭ����������
	{
		while((USART3->SR&0X40)==0);//�ȴ����ͽ���		  
		USART3->DR=buf[t];
	}	 
	while((USART3->SR&0X40)==0);//�ȴ����ͽ���	
	
}


//��ʼ��IO ����2
//pclk1:PCLK1ʱ��Ƶ��(Mhz)
//bound:������	  
void usart2_Init(u32 pclk1,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk1*1000000)/(bound*16);//�õ�USARTDIV
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
    mantissa<<=4;
	mantissa+=fraction; 
 
	RCC->APB2ENR|=1<<5;   	//ʹ��PORTD��ʱ��  
	GPIOD->CRL&=0XF00FFFFF;	//IO״̬����
	GPIOD->CRL|=0X08B00000;	//IO״̬����	  

	RCC->APB1ENR|=1<<17;  //ʹ�ܴ���2ʱ�� 
	RCC->APB2ENR|=1<<0;  //��������IOʱ�ӿ���  
	AFIO->MAPR|=1<<3;//USART2_REMAPΪ1 UART2������ӳ�䡣TX/PD5��RX/PD6
	
	RCC->APB1RSTR|=1<<17;   //��λ����2
	RCC->APB1RSTR&=~(1<<17);//ֹͣ��λ	   	   
	//����������
 	USART2->BRR=mantissa; 	// ����������	 
	USART2->CR1|=0X200C;  	//1λֹͣ,��У��λ.
	//ʹ�ܽ����ж�
	USART2->CR1|=1<<8;    	//PE�ж�ʹ��
	USART2->CR1|=1<<5;    	//���ջ������ǿ��ж�ʹ��	    	
//	MY_NVIC_Init(0,0,USART2_IRQChannel,2);//��2 
}


/*************����2���ݷ���****************/
void usart2_Send_Data(u8 *buf,u8 len)
{
	u8 t;
  for(t=0;t<len;t++)		//ѭ����������
	{
		while((USART2->SR&0X40)==0);//�ȴ����ͽ���		  
		USART2->DR=buf[t];
	}	 
	while((USART2->SR&0X40)==0);//�ȴ����ͽ���	
	
}



//��ʼ��IO ����1
//pclk2:PCLK2ʱ��Ƶ��(Mhz)
//bound:������
//CHECK OK
//091209
void usart1_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV
	mantissa=temp;				 //�õ���������
	fraction=(temp-mantissa)*16; //�õ�С������	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //ʹ��PORTA��ʱ��  
	RCC->APB2ENR|=1<<14;  //ʹ�ܴ���ʱ�� 
	GPIOA->CRH&=0XFFFFF00F; 
	GPIOA->CRH|=0X000008B0;//IO״̬����
		  
	RCC->APB2RSTR|=1<<14;   //��λ����1
	RCC->APB2RSTR&=~(1<<14);//ֹͣ��λ	   	   
	//����������
 	USART1->BRR=mantissa; // ����������	 
	USART1->CR1|=0X200C;  //1λֹͣ,��У��λ.
	//ʹ�ܽ����ж�
	USART1->CR1|=1<<8;    //PE�ж�ʹ��
	USART1->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
//	MY_NVIC_Init(2,0,USART1_IRQChannel,2);//��2��������ȼ� 
}

u8 uart1_sendbyte(u8 dat)
{
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = dat;      
	return dat;
}

void usart1_Send_Data(u8 *buf,u8 len)
{
	u8 t;
  for(t=0;t<len;t++)		//ѭ����������
	{
		while((USART1->SR&0X40)==0);//�ȴ����ͽ���		  
		USART1->DR=buf[t];
	}	 
	while((USART1->SR&0X40)==0);//�ȴ����ͽ���	
	
}

