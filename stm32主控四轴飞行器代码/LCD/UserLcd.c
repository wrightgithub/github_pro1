#include "UserLcd.h"
#define  EN_LCD  2 //0:5110,   1:OLED,  2:TFTLCD 

DMPresult *DMP_result; 



void UserLcd_Init()
{
 #if (EN_LCD==0)//ʹ��5110
		{
			N5110_IO_Init();
 	    LCD5110_init();
		}
#endif
#if (EN_LCD==1)//ʹ��oled
	{
	  OLED_Init();			//��ʼ��Һ��  
		
		 OLED_ShowString(0,0, "Now:",16); 				
		 OLED_ShowString(0,32, "TEMP:",16); 
		 OLED_ShowString(0,48, "HUM:  %",16); 
		
		OLED_ShowString(76,0,"key:",16); 
		OLED_ShowString(76,16,"UP:",16); 
		OLED_ShowString(68,32,"DOWN:",16); 
		OLED_ShowString(76,48,"DIR:",16); 
     OLED_Refresh_Gram();
		
	}
#endif             
#if (EN_LCD==2)   //ʹ��tftlcd
	{	
		 LCD_Init();//I/O �ڸĹ���
	///	SPI_Flash_Init();	//SPI FLASH��ʼ�� 
	  //Font_Init();//�ֿⲻ����,������ֿ�

	}
#endif
 


}


void UserLcdShow()
{
	/***********5110***************/
#if (EN_LCD==0)//ʹ��5110
  {
	    LCD_write_String(0,0,"KeyNum");//һ�����14���ַ���һ���ַ�6��X��λ���0-5��,X���83��y���5
  }
		#endif
	/*********oled****************/
#if (EN_LCD==1)//ʹ��oled
  {

		OLED_ShowString(0,16, "DOWN...",16); 
		OLED_ShowNum(40,32,Enment->temperature,2,16);
		OLED_Refresh_Gram();
	}
	#endif
	
	/**********TFTLCD************/
#if (EN_LCD==2)   //ʹ��tftlcd
	{
		
   POINT_COLOR=RED;	
   LCD_ShowString(30,20,"mpu6050");		
	 LCD_ShowString(10,40,"pitch");	
	 LCD_ShowString(10,60,"roll");	
	 LCD_ShowString(10,80,"yaw");	
	 DMP_result = Get_DMP_RES();	
   LCD_float(140,40,3,2,DMP_result->Pitch);
   LCD_float(140,60,3,2,DMP_result->Roll);
	 LCD_float(140,80,3,2,DMP_result->Yaw);
		
//	 LCD_ShowNum(140,40,(DMP_result->Pitch)*10,4,16);
//	 LCD_ShowNum(140,60,(DMP_result->Roll)*10,4,16);
	// LCD_ShowNum(140,80,(DMP_result->Yaw)*10,4,16);	  //�Ƕ�
	// delay_ms(200);		    
		
	}
	#endif

}




u8 *RemoteShow()//����TFT��ʾ
{
		u8 key=0;
	  u8 string=0,*str=&string;
		if(Remote_Rdy)
		{
			key=Remote_Process();
		//	LCD_ShowNum(86,130,key,3,16);//��ʾ��ֵ
		//	LCD_ShowNum(186,130,Remote_Cnt,3,16);//��ʾ��������		  
			switch(key)
			{
		    case 0:str="ERROR     ";break;			   
				case 162:str="POWER   ";break;	    
				case 98:str="UP       ";break;	
	      case 168:str="DOWN     ";break;						
				case 2:str="PLAY      ";break;		 
				case 226:str="ALIENTEK";break;		  
				case 194:str="RIGHT    ";break;	   
				case 34:str="LEFT     ";break;		  
				case 224:str="VOL-    ";break;		   
				case 144:str="VOL+    ";break;		    
				case 104:str="1       ";break;		  
				case 152:str="2       ";break;	   
				case 176:str="3      ";break;	    
				case 48:str="4       ";break;		    
				case 24:str="5       ";break;		    
				case 122:str="6      ";break;		  
				case 16:str="7       ";break;			   					
				case 56:str="8       ";break;	 
				case 90:str="9       ";break;
				case 66:str="0       ";break;
				case 82:str="DELETE  ";break;		     
			}
				LCD_ShowString(86,150,str);
			  return str;
		}
		
		return 0;
}

