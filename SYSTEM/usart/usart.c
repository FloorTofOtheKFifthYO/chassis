#include "usart.h"	
#include "math.h"


#define X_mid 125
#define Y_mid 125

u8  UART5_RX_BUF[USART_REC_LEN];
u16 UART5_RX_STA;         		
u8  USART3_RX_BUF[USART_REC_LEN];    
u16 USART3_RX_STA;
s8 ptrB,ptrS;

static int *x,*y;

//��ʼ��IO ����1 
//bound:������
void usart_init(int *Hx,int *Hy){
   //GPIO�˿�����
	USART_InitTypeDef USART_InitStructure;
	
//////////////////////////////////////////////////////////////////////////////////////////	
	
	USART_InitStructure.USART_BaudRate = 115200;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
   
	USART_Init(USART1, &USART_InitStructure); //��ʼ������1
    USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1
	USART_Init(USART2, &USART_InitStructure); //��ʼ������2
    USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���2
	USART_Init(UART5, &USART_InitStructure); //��ʼ������5
    USART_Cmd(UART5, ENABLE);  //ʹ�ܴ���5
	USART_Init(USART3, &USART_InitStructure); //��ʼ������3
    USART_Cmd(USART3, ENABLE);  //ʹ�ܴ���3
		
	USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//��������ж�
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_InitStructure.USART_BaudRate = 115200;//����������
	USART_Init(UART4, &USART_InitStructure); //��ʼ������4
    USART_Cmd(UART4, ENABLE);  //ʹ�ܴ���4
	x=Hx;y=Hy;
	ptrB=-1;
	ptrS=0;
}

void USART3_IRQHandler(void)                	
{
	u8 Res;	u8 tmp;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) 
	{
		Res =USART_ReceiveData(USART3);
	
	}
}

void USART1_IRQHandler(void)                	
{
	u8 Res;	u8 tmp;
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) 
	{
		OPEN_Hander = 1;
		Res =USART_ReceiveData(USART1);
		if (Res==0x0d&&ptrS<0) {
			ptrS=-ptrS;
			return;
		}
		if (ptrB==-1){
		switch (Res){
			case 'A':
				ptrB=LU_KEY;
				break;
			case 'D':
				ptrB=LR_KEY;
				break;
			case 'B':
				ptrB=LD_KEY;
				break;
			case 'C':
				ptrB=LL_KEY;
				break;
			case 'F':     
				ptrB=L2_KEY;
				break;
			case 'N':
				ptrB=R2_KEY;
				break;
			case 'M':        
				ptrB=R1_KEY;
				break;
			case 'E':
				ptrB=L1_KEY;
				break;
			case 'I':       
				ptrB=RU_KEY;
				break;
			case 'L':       
				ptrB=RR_KEY;
				break;
			case 'J':       
				ptrB=RD_KEY;
				break;
			case 'K':        
				ptrB=RL_KEY;
				break;
			default:
				break;
		}}
		if (ptrS<=0){
		switch(Res){
			case 'W':       
				ptrS=-1;
				*x=0;
				break;
			case 'P':      
				*y=*x;
				*x=0;
				break;
			case 'Q':      
				ptrS=-2;
				*x=0;
				break;
			case 'S':      
				*y=*x;
				*x=0;
				break;
			default :
				*x=*x * 10 + Res - 48;
				break;
		}}
		//USART_SendString(UART5,"D=%f \n",direction_angle);
     }		
} 


//void USART3_IRQHandler(void)                	
//{
//	u8 Res;	
//	static u8 rec_buf [4];
//	static u8 index =0;
//	static s8 WPQS_Mark=0;
//	static u16 W_number = 0 ,P_number = 0 ,Q_number = 0 , S_number = 0;
//	int ten = 1;
//	int i=0;
//	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) 
//	{
//		Res =USART_ReceiveData(USART3);
//		switch (Res){
//			case 'A'://ǰ��       �н�
//				Hand_flag = 5 ;
//				direction_angle = -PI/2;
//				break;
//			case 'B'://����
//				Hand_flag = 5 ;
//				direction_angle = PI/2;
//				break;
//			case 'D'://��
//				Hand_flag = 5 ;
//				direction_angle = PI;
//				break;
//			case 'C'://��
//				Hand_flag = 5 ;
//				direction_angle = 0;
//				break;
//			case 'F'://L2         ����
//				WANTSPEED-=50;
//				if(WANTSPEED <0)
//					WANTSPEED = 0;
//				//USART_SendString(UART5,"%d  \n",WANTSPEED);
//				break;
//			case 'N'://R2
//				WANTSPEED+=50;
//			   // USART_SendString(UART5,"%d  \n",WANTSPEED);
//				break;
//			
//			case 'M'://L1         ��ת ��
//				TURN_Flag = 5;
//				Turn_R_Flag = 1;
//				break;
//			case 'E'://R1
//				TURN_Flag = 5;
//				Turn_L_Flag = 1;
//				break;
//			
//			case 'I'://Y         ���ֱ�����
//				OPEN_Hander = 1;
//				break;
//			case 'J'://R2        ���ֱ�����
//				OPEN_Hander = 0;
//				break;
//			
////			case 'W'://ǰ��
////				if(WPQS_Mark == 2){
////					for(i = index - 1;i>=0 ;i--){
////						if(i>=0){
////							P_number += (rec_buf[i]-48)*ten;
////							ten = ten*10;
////						}
////					}
////					USART_SendString(UART5,"P=%d",P_number);
////				}
////				if(P_number>90&&P_number<140){
////					P_number = 125;
////				}
////				WPQS_Mark = 1;
////				index = 0;
////				
////				break;
////			case 'P'://����
////				Hand_flag = true ;
////				if(WPQS_Mark == 1){
////					for(i = index - 1;i>=0 ;i--){
////						if(i>=0){
////							W_number += (rec_buf[i]-48)*ten;
////							ten = ten*10;
////						}
////					
////					}
////					USART_SendString(UART5,"W=%d \n",W_number);
////				}
////				if(W_number>90&&W_number<140){
////					W_number = 125;
////				}
////				direction_angle = atan2 ((W_number-125), (P_number-125));
////				WPQS_Mark = 2;
////				index = 0;
////				break;

//			default :
////				if(Res >=48 &&Res <=57&&(WPQS_Mark == 2||WPQS_Mark ==1)){
////					rec_buf[index] = Res;
////					//USART_SendString(UART5,"N=%d \n",rec_buf[index]);
////					index ++;
////				}else{
////					
////				}
//				OPEN_Hander = 10;
//				break;
//		}
//		//USART_SendString(UART5,"D=%f \n",direction_angle);
//  		 
//     }		
//} 





//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if _printf
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
_ttywrch(int ch)
{
ch = ch;
}
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART_printf->SR&0X40)==0);//ѭ������,ֱ���������  
	USART_printf->DR = (u8) ch;      
	return ch;
}
#endif




void USART_SendString(USART_TypeDef* USARTx, char *fmt, ...)
 {
	char buffer[STR_BUFFER_LEN+1];  // CMD_BUFFER_LEN�����Լ������
	u8 i = 0;
	
	va_list arg_ptr;
	va_start(arg_ptr, fmt);  
	vsnprintf(buffer, STR_BUFFER_LEN+1, fmt, arg_ptr);
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
	va_end(arg_ptr);
 } 


 
