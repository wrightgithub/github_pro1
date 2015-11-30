#include "usart.h"
void usart3_config(u32 bound )
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
		/* config USART3 clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);  //使能AFIO功能的时钟
  GPIO_PinRemapConfig(GPIO_FullRemap_USART3 ,ENABLE);  //进行重映射
	
	
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	    
  /* Configure USART2 Rx (PD.09) as input floating */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOD, &GPIO_InitStructure);
	
	  USART_InitStructure.USART_BaudRate = bound;                 /*设置波特率为115200*/
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;  /*设置数据位为8位*/
    USART_InitStructure.USART_StopBits = USART_StopBits_1;       /*设置停止位为1位*/
    USART_InitStructure.USART_Parity = USART_Parity_No;          /*无奇偶校验*/    
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; /*没有硬件流控*/
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;      /*发送与接收*/
    /*完成串口COM3的时钟配置、GPIO配置，根据上述参数初始化并使能*/
	
	USART_Init(USART3, &USART_InitStructure);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);
}

/**************************实现函数********************************************
*函数原型:		void UART3_Put_Char(unsigned char DataToSend)
*功　　能:		蓝牙发送一个字节
输入参数：
		unsigned char DataToSend   要发送的字节数据
输出参数：没有	
*******************************************************************************/
uint8_t UART3_Put_Char(unsigned char DataToSend)
{
	//将要发送的字节写到UART1的发送缓冲区
	USART_SendData(USART3, (unsigned char) DataToSend);
	//等待发送完成
  	while (!(USART3->SR & USART_FLAG_TXE));
	return DataToSend;
}





/********************************************************
         my_usart
********************************************************/

/*************串口3数据发送****************/
void usart3_Send_Data(u8 *buf,u8 len)
{
	u8 t;
  for(t=0;t<len;t++)		//循环发送数据
	{
		while((USART3->SR&0X40)==0);//等待发送结束		  
		USART3->DR=buf[t];
	}	 
	while((USART3->SR&0X40)==0);//等待发送结束	
	
}


//初始化IO 串口2
//pclk1:PCLK1时钟频率(Mhz)
//bound:波特率	  
void usart2_Init(u32 pclk1,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk1*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
    mantissa<<=4;
	mantissa+=fraction; 
 
	RCC->APB2ENR|=1<<5;   	//使能PORTD口时钟  
	GPIOD->CRL&=0XF00FFFFF;	//IO状态设置
	GPIOD->CRL|=0X08B00000;	//IO状态设置	  

	RCC->APB1ENR|=1<<17;  //使能串口2时钟 
	RCC->APB2ENR|=1<<0;  //辅助功能IO时钟开启  
	AFIO->MAPR|=1<<3;//USART2_REMAP为1 UART2功能重映射。TX/PD5，RX/PD6
	
	RCC->APB1RSTR|=1<<17;   //复位串口2
	RCC->APB1RSTR&=~(1<<17);//停止复位	   	   
	//波特率设置
 	USART2->BRR=mantissa; 	// 波特率设置	 
	USART2->CR1|=0X200C;  	//1位停止,无校验位.
	//使能接收中断
	USART2->CR1|=1<<8;    	//PE中断使能
	USART2->CR1|=1<<5;    	//接收缓冲区非空中断使能	    	
//	MY_NVIC_Init(0,0,USART2_IRQChannel,2);//组2 
}


/*************串口2数据发送****************/
void usart2_Send_Data(u8 *buf,u8 len)
{
	u8 t;
  for(t=0;t<len;t++)		//循环发送数据
	{
		while((USART2->SR&0X40)==0);//等待发送结束		  
		USART2->DR=buf[t];
	}	 
	while((USART2->SR&0X40)==0);//等待发送结束	
	
}



//初始化IO 串口1
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率
//CHECK OK
//091209
void usart1_init(u32 pclk2,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
	mantissa=temp;				 //得到整数部分
	fraction=(temp-mantissa)*16; //得到小数部分	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB2ENR|=1<<14;  //使能串口时钟 
	GPIOA->CRH&=0XFFFFF00F; 
	GPIOA->CRH|=0X000008B0;//IO状态设置
		  
	RCC->APB2RSTR|=1<<14;   //复位串口1
	RCC->APB2RSTR&=~(1<<14);//停止复位	   	   
	//波特率设置
 	USART1->BRR=mantissa; // 波特率设置	 
	USART1->CR1|=0X200C;  //1位停止,无校验位.
	//使能接收中断
	USART1->CR1|=1<<8;    //PE中断使能
	USART1->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
//	MY_NVIC_Init(2,0,USART1_IRQChannel,2);//组2，最低优先级 
}

u8 uart1_sendbyte(u8 dat)
{
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = dat;      
	return dat;
}

void usart1_Send_Data(u8 *buf,u8 len)
{
	u8 t;
  for(t=0;t<len;t++)		//循环发送数据
	{
		while((USART1->SR&0X40)==0);//等待发送结束		  
		USART1->DR=buf[t];
	}	 
	while((USART1->SR&0X40)==0);//等待发送结束	
	
}

