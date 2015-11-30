#ifndef __5110_H
#define __5110_H

#include "sys.h"


#define uchar unsigned char
#define uint  unsigned int

#define LCD_SCE PCout(8)  
#define 	LCD_REST PCout(6)
#define  LCD_DC PEout(0)
#define SDIN  PEout(2)
#define 	LCD_CLK PEout(4)
#define lcd5110_bl PEout(6) 

// sbit    LCD_SCE = P2^1;  //片选
// sbit    LCD_REST = P2^0;  //复位,0复位
// sbit    LCD_DC  = P2^2;  //1写数据，0写指令
// sbit    SDIN = P2^3;  //数据
// sbit    LCD_CLK =  P2^4;  //时钟

/*function
************************/
void N5110_IO_Init(void);
void zhuansu1( unsigned char X,unsigned char Y,unsigned int tem_value);
///////////SPI写操作
void LCD_write_byte(unsigned char dat, unsigned char command);        
///////////5110初始化
void LCD5110_init(void);
///////////清屏
void LCD_clear(void);
void LCD_set_XY(unsigned char X, unsigned char Y);
///////////显示特殊符号
void LCD_write_sign(uchar X,uchar Y,uchar char_num  );
//////////显示一个字符
void LCD_write_char(unsigned char c);
//////////显示字符串
void LCD_write_number( unsigned char X,unsigned char Y,unsigned int number,u8 flag);
/////////////显示数值
void LCD_write_String(unsigned char X,unsigned char Y,unsigned char *s);
//////////显示一个汉字
void LCD_WRITE_ZH(unsigned char X, unsigned char Y,unsigned char  ZH[][32],unsigned char index); 
///////////显示汉字串
void LCD_WRITE_ZH_STRING(uchar X,uchar Y,          // 起始坐标
                              uchar ZHS[][32],     // 汉字字符串
							  uchar width,         // 每个字符边长   
							  uchar index,         // 起始索引号
							  uchar num,		   // 显示个数
                   			  uchar space);         // 间距				 
///////////////////////画图
void LCD_draw_bmp_pixel(uchar X,uchar Y,uchar *map, uchar width,uchar height);

#endif 
