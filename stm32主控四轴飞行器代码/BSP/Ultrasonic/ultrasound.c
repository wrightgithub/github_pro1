/*
*********************************************************************************************************
*                                   Embedded Systems Building Blocks
*                                Complete and Ready-to-Use Modules in C
*
*                                        ULTRASOUND Module Driver
*
*                            			(c) Copyright 2014, Fisher
*                                           
* Filename   : ultrasound.c
* Programmer : Fisher
*********************************************************************************************************
*                                              DESCRIPTION
*
*	这个文件初始化超声波模块所需的IO口以及定时器等的相关配置
*	提供的接口：ultrasound_init	初始化超声波模块相关配置
*				high	超声波测得的距离
*	消耗的IO口：PA1		下拉输入	TIM2 CH2输入捕获IO口
*				PA0		下拉输出	超声波触发口
*********************************************************************************************************
*/

/*$PAGE*/
/*
*********************************************************************************************************
*											INCLUDE FILES
*********************************************************************************************************
*/
#include "ultrasound.h"
#include "led.h"
#include "usart.h"
#include "delay.h"
/*$PAGE*/

////////////////////////////////////////////////////////////////////////////////// 	  

//设置向量表偏移地址
//NVIC_VectTab:基址
//Offset:偏移量
//CHECK OK
//091207
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset)	 
{ 
  	//检查参数合法性
	assert_param(IS_NVIC_VECTTAB(NVIC_VectTab));
	assert_param(IS_NVIC_OFFSET(Offset));  	 
	SCB->VTOR = NVIC_VectTab|(Offset & (u32)0x1FFFFF80);//设置NVIC的向量表偏移寄存器
	//用于标识向量表是在CODE区还是在RAM区
}
//设置NVIC分组
//NVIC_Group:NVIC分组 0~4 总共5组 
//CHECK OK
//091209
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group)	 
{ 
	u32 temp,temp1;	  
	temp1=(~NVIC_Group)&0x07;//取后三位
	temp1<<=8;
	temp=SCB->AIRCR;  //读取先前的设置
	temp&=0X0000F8FF; //清空先前分组
	temp|=0X05FA0000; //写入钥匙
	temp|=temp1;	   
	SCB->AIRCR=temp;  //设置分组	    	  				   
}
//设置NVIC 
//NVIC_PreemptionPriority:抢占优先级
//NVIC_SubPriority       :响应优先级
//NVIC_Channel           :中断编号
//NVIC_Group             :中断分组 0~4
//注意优先级不能超过设定的组的范围!否则会有意想不到的错误
//组划分:
//组0:0位抢占优先级,4位响应优先级
//组1:1位抢占优先级,3位响应优先级
//组2:2位抢占优先级,2位响应优先级
//组3:3位抢占优先级,1位响应优先级
//组4:4位抢占优先级,0位响应优先级
//NVIC_SubPriority和NVIC_PreemptionPriority的原则是,数值越小,越优先
//CHECK OK
//100329
void MY_NVIC_Init(u8 NVIC_PreemptionPriority,u8 NVIC_SubPriority,u8 NVIC_Channel,u8 NVIC_Group)	 
{ 
	u32 temp;	
	u8 IPRADDR=NVIC_Channel/4;  //每组只能存4个,得到组地址 
	u8 IPROFFSET=NVIC_Channel%4;//在组内的偏移
	IPROFFSET=IPROFFSET*8+4;    //得到偏移的确切位置
	MY_NVIC_PriorityGroupConfig(NVIC_Group);//设置分组
	temp=NVIC_PreemptionPriority<<(4-NVIC_Group);	  
	temp|=NVIC_SubPriority&(0x0f>>NVIC_Group);
	temp&=0xf;//取低四位

	if(NVIC_Channel<32)NVIC->ISER[0]|=1<<NVIC_Channel;//使能中断位(要清除的话,相反操作就OK)
	else NVIC->ISER[1]|=1<<(NVIC_Channel-32);    
	NVIC->IPR[IPRADDR]|=temp<<IPROFFSET;//设置响应优先级和抢断优先级   	    	  				   
}

//外部中断配置函数
//只针对GPIOA~G;不包括PVD,RTC和USB唤醒这三个
//参数:GPIOx:0~6,代表GPIOA~G;BITx:需要使能的位;TRIM:触发模式,1,下升沿;2,上降沿;3，任意电平触发
//该函数一次只能配置1个IO口,多个IO口,需多次调用
//该函数会自动开启对应中断,以及屏蔽线   
//待测试...
void Ex_NVIC_Config(u8 GPIOx,u8 BITx,u8 TRIM) 
{
	u8 EXTADDR;
	u8 EXTOFFSET;
	EXTADDR=BITx/4;//得到中断寄存器组的编号
	EXTOFFSET=(BITx%4)*4;

	RCC->APB2ENR|=0x01;//使能io复用时钟

	AFIO->EXTICR[EXTADDR]&=~(0x000F<<EXTOFFSET);//清除原来设置！！！
	AFIO->EXTICR[EXTADDR]|=GPIOx<<EXTOFFSET;//EXTI.BITx映射到GPIOx.BITx
	
	//自动设置
	EXTI->IMR|=1<<BITx;//  开启line BITx上的中断
	//EXTI->EMR|=1<<BITx;//不屏蔽line BITx上的事件 (如果不屏蔽这句,在硬件上是可以的,但是在软件仿真的时候无法进入中断!)
 	if(TRIM&0x01)EXTI->FTSR|=1<<BITx;//line BITx上事件下降沿触发
	if(TRIM&0x02)EXTI->RTSR|=1<<BITx;//line BITx上事件上升降沿触发
} 


//不能在这里执行所有外设复位!否则至少引起串口不工作.		    
//把所有时钟寄存器复位
//CHECK OK
//091209
void MYRCC_DeInit(void)
{										  					   
	RCC->APB1RSTR = 0x00000000;//复位结束			 
	RCC->APB2RSTR = 0x00000000; 
	  
  	RCC->AHBENR = 0x00000014;  //睡眠模式闪存和SRAM时钟使能.其他关闭.	  
  	RCC->APB2ENR = 0x00000000; //外设时钟关闭.			   
  	RCC->APB1ENR = 0x00000000;   
	RCC->CR |= 0x00000001;     //使能内部高速时钟HSION	 															 
	RCC->CFGR &= 0xF8FF0000;   //复位SW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0],ADCPRE[1:0],MCO[2:0]					 
	RCC->CR &= 0xFEF6FFFF;     //复位HSEON,CSSON,PLLON
	RCC->CR &= 0xFFFBFFFF;     //复位HSEBYP	   	  
	RCC->CFGR &= 0xFF80FFFF;   //复位PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE 
	RCC->CIR = 0x00000000;     //关闭所有中断
	//配置向量表				  
#ifdef  VECT_TAB_RAM
	MY_NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else   
	MY_NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif
}
//THUMB指令不支持汇编内联
//采用如下方法实现执行汇编指令WFI
//CHECK OK
//091209
__asm void WFI_SET(void)
{
	WFI;    
}
//进入待机模式	 
//check ok 
//091202
void Sys_Standby(void)
{
	SCB->SCR|=1<<2;//使能SLEEPDEEP位 (SYS->CTRL)	   
  	RCC->APB1ENR|=1<<28;     //使能电源时钟	    
 	PWR->CSR|=1<<8;          //设置WKUP用于唤醒
	PWR->CR|=1<<2;           //清除Wake-up 标志
	PWR->CR|=1<<1;           //PDDS置位		  
	WFI_SET();				 //执行WFI指令		 
}	  
    
//系统软复位
//CHECK OK
//091209
void Sys_Soft_Reset(void)
{   
	SCB->AIRCR =0X05FA0000|(u32)0x04;	  
} 

//JTAG模式设置,用于设置JTAG的模式
//mode:jtag,swd模式设置;00,全使能;01,使能SWD;10,全关闭;
//CHECK OK	
//100818		  
void JTAG_Set(u8 mode)
{
	u32 temp;
	temp=mode;
	temp<<=25;
	RCC->APB2ENR|=1<<0;     //开启辅助时钟	   
	AFIO->MAPR&=0XF8FFFFFF; //清除MAPR的[26:24]
	AFIO->MAPR|=temp;       //设置jtag模式
} 
//系统时钟初始化函数
//pll:选择的倍频数，从2开始，最大值为16	
//CHECK OK
//091209
void Stm32_Clock_Init(u8 PLL)
{
	unsigned char temp=0;   
	MYRCC_DeInit();		  //复位并配置向量表
	RCC->CR|=0x00010000;  //外部高速时钟使能HSEON
	while(!(RCC->CR>>17));//等待外部时钟就绪
	RCC->CFGR=0X00000400; //APB1=DIV2;APB2=DIV1;AHB=DIV1;
	PLL-=2;//抵消2个单位
	RCC->CFGR|=PLL<<18;   //设置PLL值 2~16
	RCC->CFGR|=1<<16;	  //PLLSRC ON 
	FLASH->ACR|=0x32;	  //FLASH 2个延时周期

	RCC->CR|=0x01000000;  //PLLON
	while(!(RCC->CR>>25));//等待PLL锁定
	RCC->CFGR|=0x00000002;//PLL作为系统时钟	 
	while(temp!=0x02)     //等待PLL作为系统时钟设置成功
	{   
		temp=RCC->CFGR>>2;
		temp&=0x03;
	}    
}		    









/*
*********************************************************************************************************
*											CONSTANTS
*********************************************************************************************************
*/
/* 
** 捕获状态
** [7]:0,没有成功的捕获;1,成功捕获到一次.
** [6]:0,还没捕获到高电平;1,已经捕获到高电平了.
** [5:0]:捕获高电平后溢出的次数
*/
u8   TIM2CH2_CAPTURE_STA = 0;  								/* 输入捕获状态								*/           
u16  TIM2CH2_CAPTURE_VAL;  									/* 输入捕获值								*/
float high = 0.0;

/*$PAGE*/


/*$PAGE*/
/*
*********************************************************************************************************
*                                  		初始化定时器输入捕获功能
*
*	描述：	此函数初始化STM32定时器2的输入捕获功能
*			Tout = （(arr+1) * （psc+1））/ Tclk
*	参数：	arr 	自动重装值。
*			psc 	时钟预分频数
*	返回值：无
*********************************************************************************************************
*/
void TIM2_Cap_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR |= 1 << 0;						/* TIM2时钟使能    										*/
	RCC->APB2ENR |= 1 << 2;						/* GPIOA 时钟使能 										*/
	
	GPIOA->CRL &= 0XFFFFFF0F;
	GPIOA->CRL |= 0X00000080;					/* PA1下拉输入 											*/
	GPIOA->ODR |= 0 << 1;
	/*
	** 自动重装载值，计数器计数到这个值就会触发溢出中断
	*/
 	TIM2->ARR = arr;
	/*
	** 预分频器,得到计数时钟，
	** 比如预分频器为7200，计数时钟就为10KHz，即计数到10000为1s	
	*/
	TIM2->PSC = psc; 
	
	/* CH1通道捕获配置 */
// 	TIM2->CCMR1 |= 2 << 0;						/* CC1S=01 	选择输入端 IC1映射到TI2上 					*/
//  	TIM2->CCMR1 |= 0 << 4; 						/* IC1F=0000 配置输入滤波器 不滤波 						*/
//  	TIM2->CCMR1 |= 0 << 10; 					/* IC2PS=00 	配置输入分频,不分频 					*/
// 	TIM2->CCER |= 0 << 1; 						/* CC1P=0	上升沿捕获 									*/
// 	TIM2->CCER |= 1 << 0; 						/* CC1E=1 	允许捕获计数器的值到捕获寄存器中 			*/
// 	TIM2->DIER |= 1 << 1; 						/* 允许捕获1中断											*/
	
	/* CH2通道捕获配置 */
	TIM2->CCMR1 |= 1 << 8;						/* CC2S = 01, CC2通道配置为输入，IC2映射到TI2上			*/
	TIM2->CCMR1 |= 0 << 12;						/* IC2F=0000  配置输入滤波器  不滤波 					*/
	TIM2->CCMR1 |= 0 << 10;						/* IC2PS=00    配置输入分频,不分频						*/
	TIM2->CCER |= 0 << 5;						/* CC2P=0  上升沿捕获 									*/
	TIM2->CCER |= 1 << 4;						/* CC2E=1    允许捕获计数器的值到捕获寄存器中 			*/
	TIM2->DIER |= 1 << 2;   					/* 允许捕获2中断										*/
	
	TIM2->DIER |= 1 << 0;   					/* 允许更新中断											*/
	
	TIM2->CR1 |= 0x01;    						/* 使能定时器2											*/
  MY_NVIC_Init(2,2,TIM2_IRQn,2);		/* 抢占2，子优先级2，组2								*/		 
}

/*$PAGE*/
/*
*********************************************************************************************************
*                                  		定时器2中断服务程序
*
*	描述：	此函数提供定时器2的中断服务程序
*	参数：	无
*	返回值：无
*********************************************************************************************************
*/

	void TIM2_IRQHandler(void)
	{
		u16 tsr;
		
		tsr = TIM2->SR;
		if((TIM2CH2_CAPTURE_STA & 0X80) == 0)					/* 还未成功捕获 							*/
		{
			if(tsr & 0X01)										/* 溢出										*/
			{
				if(TIM2CH2_CAPTURE_STA & 0X40)					/* 已经捕获到高电平了						*/
				{
					if((TIM2CH2_CAPTURE_STA & 0X3F) == 0X3F)	/* 高电平太长了								*/
					{
						TIM2CH2_CAPTURE_STA |= 0X80;			/* 标记成功捕获了一次						*/
						TIM2CH2_CAPTURE_VAL = 0XFFFF;
					} else TIM2CH2_CAPTURE_STA++;
				}
			}
			if(tsr & 0X04)										/* 捕获 2 发生捕获事件						*/
			{
				if(TIM2CH2_CAPTURE_STA & 0X40)    				/* 捕获到一个下降沿							*/
				{ 
					TIM2CH2_CAPTURE_STA |= 0X80;  				/* 标记成功捕获到一次高电平脉宽				*/
					TIM2CH2_CAPTURE_VAL = TIM2->CCR2;			/* 获取当前的捕获值.						*/
					
					high = TIM2CH2_CAPTURE_STA & 0X3F; 												
					high *= 65536;								/* 溢出时间总和 							*/
					high += TIM2CH2_CAPTURE_VAL;				/* 得到总的高电平时间 						*/
					high = high * 170 / 1000000; 
					printf("High:%lf m\r\n",high);				/* 打印总的高点平时间 						*/
					
					TIM2CH2_CAPTURE_STA = 0;					/* 开启下一次捕获 							*/				
					TIM2->CCER &= ~(1 << 5);      				/* CC2P=0  设置为上升沿捕获					*/
				} else {                   						/* 还未开始,第一次捕获上升沿				*/
					TIM2CH2_CAPTURE_STA = 0;    				/* 清空										*/
					TIM2CH2_CAPTURE_VAL = 0;
					TIM2CH2_CAPTURE_STA |= 0X40;  				/* 标记捕获到了上升沿						*/
					TIM2->CNT = 0;          					/* 计数器清空								*/
					TIM2->CCER |= 1 << 5;          				/* CC2P=1  设置为下降沿捕获					*/
				} 
			}                   
		}				   
		TIM2->SR = 0;											/* 清除中断标志位 	   						*/
	}



/*$PAGE*/
/*
*********************************************************************************************************
*                                  		超声波初始化
*
*	描述：	此函数初始化超声波相关的IO口
*	参数：	无
*	返回值：无
*********************************************************************************************************
*/
void ultrasound_init(void) 
	{
	
	TIM2_Cap_Init(0XFFFF,71);					/* 1MHz频率计数				 					*/
	Timer1_Init(999,7199);            //发射周期
	
	RCC->APB2ENR |= 1 << 2; 					/* 使能PORTA时钟								*/
	GPIOA->CRL &= 0XFFFFFFF0;					/* PA0 清除之前设置 							*/
	GPIOA->CRL |= 0X00000003;					/* PA0 输出 									*/
	GPIOA->ODR |= 0 << 0;						/* PA0 下拉 									*/
}

/*$PAGE*/
/*
*********************************************************************************************************
*                                  		处理超声波测得的距离
*
*	描述：	此函数处理超声波测得的距离
*	参数：	无
*	返回值：无
*********************************************************************************************************
*/
void deal_high(void)
{
}
