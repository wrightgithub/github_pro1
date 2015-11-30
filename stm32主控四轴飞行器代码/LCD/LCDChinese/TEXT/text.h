#ifndef __TEXT_H__
#define __TEXT_H__	 
#include "sys.h"		    
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//文本显示程序 驱动代码		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2012/10/25 
//版本：V1.1
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
//********************************************************************************
//V1.1修改说明 20121025
//修正y大于255显示出错的bug
//////////////////////////////////////////////////////////////////////////////////	 

					    	    
void Get_HzMat(unsigned char *code,unsigned char *mat,u8 size);//得到汉字的点阵码
void Show_Font(u16 x,u16 y,u8 *font,u8 size,u8 mode);//在指定位置显示一个汉字
void Show_Str(u16 x,u16 y,u8*str,u8 size,u8 mode);//在指定位置显示一个字符串 
void Show_Str_Mid(u16 x,u16 y,u8*str,u8 size,u8 len);								 
void my_stradd(u8*str1,u8*str2);//将str2与str1相加,结果保存在str1


#endif
