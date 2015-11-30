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

// sbit    LCD_SCE = P2^1;  //Ƭѡ
// sbit    LCD_REST = P2^0;  //��λ,0��λ
// sbit    LCD_DC  = P2^2;  //1д���ݣ�0дָ��
// sbit    SDIN = P2^3;  //����
// sbit    LCD_CLK =  P2^4;  //ʱ��

/*function
************************/
void N5110_IO_Init(void);
void zhuansu1( unsigned char X,unsigned char Y,unsigned int tem_value);
///////////SPIд����
void LCD_write_byte(unsigned char dat, unsigned char command);        
///////////5110��ʼ��
void LCD5110_init(void);
///////////����
void LCD_clear(void);
void LCD_set_XY(unsigned char X, unsigned char Y);
///////////��ʾ�������
void LCD_write_sign(uchar X,uchar Y,uchar char_num  );
//////////��ʾһ���ַ�
void LCD_write_char(unsigned char c);
//////////��ʾ�ַ���
void LCD_write_number( unsigned char X,unsigned char Y,unsigned int number,u8 flag);
/////////////��ʾ��ֵ
void LCD_write_String(unsigned char X,unsigned char Y,unsigned char *s);
//////////��ʾһ������
void LCD_WRITE_ZH(unsigned char X, unsigned char Y,unsigned char  ZH[][32],unsigned char index); 
///////////��ʾ���ִ�
void LCD_WRITE_ZH_STRING(uchar X,uchar Y,          // ��ʼ����
                              uchar ZHS[][32],     // �����ַ���
							  uchar width,         // ÿ���ַ��߳�   
							  uchar index,         // ��ʼ������
							  uchar num,		   // ��ʾ����
                   			  uchar space);         // ���				 
///////////////////////��ͼ
void LCD_draw_bmp_pixel(uchar X,uchar Y,uchar *map, uchar width,uchar height);

#endif 
