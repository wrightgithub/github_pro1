#ifndef __USART_H_
#define __USART_H_
#include "stm32f10x.h"
#include <stdio.h>

void usart3_config(u32 bound );
void usart2_Init(u32 pclk1,u32 bound);

void usart3_Send_Data(u8 *buf,u8 len);
void usart2_Send_Data(u8 *buf,u8 len);
void usart1_init(u32 pclk2,u32 bound);
u8 uart1_sendbyte(u8 dat);
void usart1_Send_Data(u8 *buf,u8 len);
#endif
