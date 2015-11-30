#ifndef __INCLUDE_H_
#define	__INCLUDE_H_

#include "stm32f10x.h"
#include "stdbool.h"
#include "string.h"
#include "math.h"
#include "stdlib.h"
//�⼸��typedefһ��Ҫ����ͷ�ļ�֮ǰ,��Ȼ���뱨��
//typedef enum {BOOLFALSE = 0, BOOLTRUE = !BOOLFALSE} bool;
typedef enum { HOLDCLOSED=0,HOLDOPEN=1,SetHigh_ing=2} AltHold;// ����ģʽ
typedef enum { CLOSED  = 0,OPEN = 1,FIX_ING=2} FLOW_fixedposition_mode; //��λģʽ 
typedef enum { END_TRACK=0,START_TRACK=1,TRACK_ING=2 } Track_Mode;// ѭ��ģʽ

#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#define STRUCT(type) typedef struct _tag_##type type;\
struct _tag_##type
#define EXTERN(_struct,type) extern _struct type 
#define Reset(p) ((p)=0)



////�ط�ָ���־
//extern u8 g_OV7620_retrans_flag;			//1:��ʾ���ط�̽��ָ��

//enum ov7620_receive_flag 
//{
//	x_0,
//	x_1,
//	y_2,
//	y_3,
//};

//extern enum ov7620_receive_flag g_ov7620_receive_flag;

//extern u8 USART1_ov7620_rxbuf[4];

#endif /* __INCLUDE_H */
