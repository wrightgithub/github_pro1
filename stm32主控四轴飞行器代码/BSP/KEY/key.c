#include "stm32f10x_gpio.h"
#include "key.h"
#include "mode.h"

_SET_LAYER Set_Layer;

void KEY_Init(void)//矩阵键盘
{
	RCC->APB2ENR|=1<<6;     //使能PORTE时钟
	GPIOE->CRH&=0X0000FFFF;//PE12-15 
	GPIOE->CRH|=0X88880000; //输入模式
	GPIOE->ODR|=0XF000<<0;	 //上拉
	
	memset(&Set_Layer,0,sizeof(Set_Layer));
} 

u8 Key_Scan(void)
{	 
	static u8 key_up=1;//按键按松开标志	
	if(key_up&&(KEY0_GET==0||KEY1_GET==0||KEY2_GET==0||KEY3_GET==0))
	{
		delay_ms(10);//去抖动 
		key_up=0;
		if(KEY0_GET==0)
		{
			return 1;
		}
		else if(KEY1_GET==0)
		{
			while(KEY1_GET==0);
			return 2;
		}
		else if(KEY2_GET==0)
		{
			while(KEY2_GET==0);
			return 3;
		}
		else if(KEY3_GET==0)
		{
			while(KEY3_GET==0);
			return 4;
		}
	}
	else if(KEY0_GET==1&&KEY2_GET==1&&KEY2_GET==1&&KEY3_GET==1)
	{
		key_up=1; 
	}		
	return 0;// 无按键按下
}

void Menu_Layer_Select(u8* key)
{
	static u16 mode_key=0;
	static u16 select_key=0;
//	static u16 add_key=0;
//	static u16 reduce_key=0;
  switch(*key)
	{
	  case 1://切换层
		{
				mode_key++;
				if(mode_key>=LAYER_NUM+1)
				{
					mode_key=1;
				}
				switch(mode_key)
				{
					case 1: Now_Layer=Layer0;  	OLED_Fill(0x00);  break;
					case 2: Now_Layer=Layer1;  	OLED_Fill(0x00);   break;
					case 3: Now_Layer=Layer2;  	OLED_Fill(0x00);   break;
					case 4: Now_Layer=Layer3;  	OLED_Fill(0x00);   break;
					case 5: Now_Layer=Layer4;  	OLED_Fill(0x00);   break;
					case 6: Now_Layer=Layer5;  	OLED_Fill(0x00);   break;	
					case 7: Now_Layer=Layer6;   OLED_Fill(0x00);	break;
												
				}
				
				break;
		}
		case 2://移动光标
		{
			if((Now_Layer==Layer2)||(Now_Layer==Layer6))
			{
			  select_key++;
				Set_Layer.next_line++;//(0-7line)
				if(	Set_Layer.next_line>=9)
				{
						OLED_P6x8Str(120,Set_Layer.next_line-2," ");
				  	Set_Layer.next_line=1;
				}
				if(Set_Layer.next_line>=1)
				{
					OLED_P6x8Str(120,Set_Layer.next_line-1,"_");
					if(Set_Layer.next_line>=2)
					{
					  OLED_P6x8Str(120,Set_Layer.next_line-2," ");
					}
				}

			}

			
		  break;
		}
	  case 3://增加
		{
		  if(Now_Layer==Layer2)
			{
				//	add_key++;
					Layer2_Add_Par(&Set_Layer.next_line);
			}
      if(Now_Layer==Layer6)
			{
			  Layer6_Add_Par(&Set_Layer.next_line);
			}
		  break;
		}
		case 4://减小
		{
			if(Now_Layer==Layer2)
			{
				// reduce_key++;
				Layer2_Reduce_Par(&Set_Layer.next_line);
			}
			 if(Now_Layer==Layer6)
			{
			  Layer6_Reduce_Par(&Set_Layer.next_line);
			}
		  break;
		}
	}
}

void Key_Mode_Start(void)
{
  if(Plane_Mode.fly_start==true)
	{
		switch(fly_mode)
		{
			case ALT_HOLD:
			{
				Plane_Mode.unlock=true;
				Static_Par.lock_num++;
			  Plane_Mode.althold=true;
	      Plane_Mode.ARMED=true;	
				Plane_Mode.fly_start=false;

				break;
			}
			case FIXED_POINT:
			{
				Plane_Mode.unlock=true;		
        Static_Par.lock_num++;				
				Plane_Mode.poshold=true;
				Static_Par.poshold_num++;
				Plane_Mode.althold=true; //同时开定高
	      Plane_Mode.ARMED=true;	
				Plane_Mode.fly_start=false;
				
			  break;
			}
			case TRACK_LINE:
			{

				Plane_Mode.unlock=true;	
        Static_Par.lock_num++;				
				Plane_Mode.poshold=true;
				Static_Par.poshold_num++;
				Plane_Mode.althold=true; //同时开定高
				
	      Plane_Mode.ARMED=true;	
				Plane_Mode.fly_start=false;
			  break;
			}
			case LINE_ABC:
			{
				//DCT(1);
				Plane_Mode.unlock=true;	
        Static_Par.lock_num++;				
			  Plane_Mode.poshold=true;
				Static_Par.poshold_num++;
				Plane_Mode.althold=true; //同时开定高
	      Plane_Mode.ARMED=true;	
				Plane_Mode.fly_start=false;
			  break;
			}
			case TRACK_ABA:
			{
				Plane_Mode.unlock=true;	
        Static_Par.lock_num++;				
			  Plane_Mode.poshold=true;
				Static_Par.poshold_num++;
				Plane_Mode.althold=true; //同时开定高
	      Plane_Mode.ARMED=true;	
				Plane_Mode.fly_start=false;
			  break;
			}
			default :break;
		}
		
			OLED_Fill(0x00);
	}
}


void Fly_Start_Count(u8 *count)
{
  if((*count)==0)
	{
		OLED_Fill(0x00); 
	  OLED_P8x16Str(30,3,"3");
		(*count)++;
	}
	else if(*count==1)
	{
		OLED_Fill(0x00); 
	  OLED_P8x16Str(30,3,"2");
		(*count)++;
	}
	else if(*count==2)
	{
		OLED_Fill(0x00); 
	  OLED_P8x16Str(30,3,"1");
		(*count)++;
	}
	
}













