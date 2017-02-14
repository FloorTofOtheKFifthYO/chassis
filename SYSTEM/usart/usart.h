#ifndef __USART_H
#define __USART_H
#include "configuration.h"
#include "stdarg.h"


void usart_init(USART_TypeDef *USARTx, uint32_t USART_BaudRate);

//void usart_init(int *x,int *y);

extern u8  UART5_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 UART5_RX_STA;         		//接收状态标记	
extern u8  USART3_RX_BUF[USART_REC_LEN];    
extern u16 USART3_RX_STA;

void USART_SendString(USART_TypeDef* USARTx, char *fmt, ...);


#endif


