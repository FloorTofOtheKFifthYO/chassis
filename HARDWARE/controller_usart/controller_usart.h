#ifndef _CON_USART_H_
#define _CON_USART_H_

#include "sys.h"
#include <stdbool.h>


#define LU_KEY 0
#define LR_KEY 1
#define LD_KEY 2
#define LL_KEY 3
#define RU_KEY 4
#define RR_KEY 5
#define RD_KEY 6
#define RL_KEY 7
#define L1_KEY 8
#define L2_KEY 9
#define R1_KEY 10
#define R2_KEY 11

typedef struct 
{
	bool ispressed;
	u8 cnt;
//  void * func;
}bottons;

typedef struct 
{
	s8 x,y;
}sticks;

void controller_usart_init(int *Hx,int *Hy);

void bottons_check();

void sticks_check(int Hx,int Hy);

//mainº¯ÊýÖÐÂÖÑ¯
void control_usart_main();

//TIM2ÖÐÂÖÑ¯
void control_usart_TIM();

#endif