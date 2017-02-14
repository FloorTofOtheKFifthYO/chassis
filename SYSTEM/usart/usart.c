#include "usart.h"	
#include "math.h"


#define X_mid 125
#define Y_mid 125

u8  UART5_RX_BUF[USART_REC_LEN];
u16 UART5_RX_STA;         		
u8  USART3_RX_BUF[USART_REC_LEN];    
u16 USART3_RX_STA;



void usart_init(USART_TypeDef *USARTx, uint32_t USART_BaudRate)
{
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = USART_BaudRate;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
   
	USART_Init(USARTx, &USART_InitStructure); //初始化串口x
    USART_Cmd(USARTx, ENABLE);  //使能串口x
	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);//开启相关中断
}

//初始化IO 串口1 
//bound:波特率
/*void usart_init(int *Hx,int *Hy){
   //GPIO端口设置
	USART_InitTypeDef USART_InitStructure;
	
//////////////////////////////////////////////////////////////////////////////////////////	
	
	USART_InitStructure.USART_BaudRate = 115200;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
   
	USART_Init(USART1, &USART_InitStructure); //初始化串口1
    USART_Cmd(USART1, ENABLE);  //使能串口1
	USART_Init(USART2, &USART_InitStructure); //初始化串口2
    USART_Cmd(USART2, ENABLE);  //使能串口2
	USART_Init(UART5, &USART_InitStructure); //初始化串口5
    USART_Cmd(UART5, ENABLE);  //使能串口5
	USART_Init(USART3, &USART_InitStructure); //初始化串口3
    USART_Cmd(USART3, ENABLE);  //使能串口3
		
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//开启相关中断
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_InitStructure.USART_BaudRate = 115200;//波特率设置
	USART_Init(UART4, &USART_InitStructure); //初始化串口4
    USART_Cmd(UART4, ENABLE);  //使能串口4
	x=Hx;y=Hy;
	ptrB=-1;
	ptrS=0;
}*/

//void USART3_IRQHandler(void)                	
//{
//	u8 Res;	u8 tmp;
//	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) 
//	{
//		Res =USART_ReceiveData(USART3);
//	
//	}
//}



//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if _printf
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
_ttywrch(int ch)
{
ch = ch;
}
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART_printf->SR&0X40)==0);//循环发送,直到发送完毕  
	USART_printf->DR = (u8) ch;      
	return ch;
}
#endif




void USART_SendString(USART_TypeDef* USARTx, char *fmt, ...)
 {
	char buffer[STR_BUFFER_LEN+1];  // CMD_BUFFER_LEN长度自己定义吧
	u8 i = 0;
	
	va_list arg_ptr;
	va_start(arg_ptr, fmt);  
	vsnprintf(buffer, STR_BUFFER_LEN+1, fmt, arg_ptr);
	 va_end(arg_ptr);
	USART_ClearFlag(USARTx,USART_FLAG_TXE);
	while ((i < STR_BUFFER_LEN) && buffer[i])
	{
		if(buffer[i] == '\n'){
			
			USART_SendData(USARTx,(u8)'\r');
			while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET); 
			USART_SendData(USARTx,(u8)buffer[i++]);
		}else{
	    USART_SendData(USARTx, (u8) buffer[i++]);
		}
		while (USART_GetFlagStatus(USARTx, USART_FLAG_TXE) == RESET); 
	}
 } 


 
