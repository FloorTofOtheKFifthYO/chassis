#include "global.h"
#include "usart.h"

void pitch_move(float v){
	if (v - 3000.f > 0.0000001)
		v = 3000;
	else if (v + 3000.f < 0.0000001)
		v=-3000;
	USART_SendString(UART4,"3v%d\r3v%d\r3v%d\r",(int)v);
}

void roll_move(float v){
	if (v - 1200.f > 0.0000001)
		v = 1200;
	else if (v + 1200.f < 0.0000001)
		v=-1200;
	USART_SendString(UART4,"4v%d\r4v%d\r4v%d\r",(int)v);
}



//void TIM2_IRQHandler(void){
//	int i;
//	if( TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET ) 
//		{
//			if (wait_cnt>-1)
//				if (++wait_cnt==50)
//				{	USART_SendString(UART5,"Good to go\n");
//					USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);wait_cnt=-1;}
//			TIM_ClearITPendingBit(TIM2,TIM_FLAG_Update);//必须清除中断标志位否则一直中断
//			for (i=0;i<12;i++) 
//			 {
//				 if (b[i]->cnt>0) b[i]->cnt--;
//				 if (b[i]->cnt==0)
//					 b[i]->ispressed=false;
//			 }
//			//USART_SendString(UART5,"%d\n",rounds*2000 + TIM4->CNT);
//		}	
//}





