#ifndef __TEXT_H__
#define __TEXT_H__	 
#include "sys.h"		    
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//Mini STM32������
//�ı���ʾ���� ��������		   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2012/10/25 
//�汾��V1.1
//��Ȩ���У�����ؾ���
//Copyright(C) ����ԭ�� 2009-2019
//All rights reserved
//********************************************************************************
//V1.1�޸�˵�� 20121025
//����y����255��ʾ�����bug
//////////////////////////////////////////////////////////////////////////////////	 

					    	    
void Get_HzMat(unsigned char *code,unsigned char *mat,u8 size);//�õ����ֵĵ�����
void Show_Font(u16 x,u16 y,u8 *font,u8 size,u8 mode);//��ָ��λ����ʾһ������
void Show_Str(u16 x,u16 y,u8*str,u8 size,u8 mode);//��ָ��λ����ʾһ���ַ��� 
void Show_Str_Mid(u16 x,u16 y,u8*str,u8 size,u8 len);								 
void my_stradd(u8*str1,u8*str2);//��str2��str1���,���������str1


#endif
