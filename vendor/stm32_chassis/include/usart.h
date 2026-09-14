#ifndef __USART_H
#define __USART_H
#include "sys.h"
void usart1_send(u8 data);
void usart1_init(u32 pclk2,u32 bound);

void usart2_send(u8 data);
void usart2_init(u32 pclk2,u32 bound);

void usart3_init(u32 pclk2,u32 bound);

void usart1_sent(u8 fun,u8*data,u8 len);
void Send_data_ROS(void);
typedef unsigned char byte;
float b2f(byte m0, byte m1, byte m2, byte m3);

void LANYAO_APP(int data);

#endif	   
	   
















