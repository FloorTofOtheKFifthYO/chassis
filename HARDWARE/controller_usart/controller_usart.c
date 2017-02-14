#include "controller_usart.h"
#include "configuration.h"
#include "usart.h"
#include "chassis.h"
#include <math.h>

extern int wait_cnt;

bottons LU,LR,LD,LL,RU,RR,RD,RL,L1,L2,R1,R2;
bottons* b[12]={&LU,&LR,&LD,&LL,&RU,&RR,&RD,&RL,&L1,&L2,&R1,&R2};
sticks L,R;
sticks* s[2]={&L,&R};

s8 ptrB,ptrS;
static int *x,*y;

extern float pos_x,pos_y, angle; 
extern u8 OPEN_Hander;

extern Chassis chassis;
static float CH_angle_M0 = -PI + 2*PI/3, CH_angle_M1 = -PI - 2*PI/3, CH_angle_M2 = -PI;

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

void controller_usart_init(int *Hx,int *Hy)
{
	usart_init(USART1,115200);
	x=Hx;y=Hy;
	ptrB=-1;
	ptrS=0;
}

void bottons_check(){
	if (ptrB>-1){
		if (ptrB>=4&ptrB<=7)
			b[ptrB]->cnt=2;
		else b[ptrB]->cnt=55;
		b[ptrB]->ispressed=true;
		ptrB=-1;
	}
}

void sticks_check(int Hx,int Hy){
	if (ptrS>0){
		if (Hx>=0&&Hx<256&&Hy>=0&&Hy<256){
			s[ptrS-1]->x=Hx-127;
			s[ptrS-1]->y=128-Hy;}
		ptrS=0;
	}
}


void control_usart_main()
{
	int Xianding = 0;
	int ChassisSpeed;
	
	
	Xianding = 0;
	if (RU.ispressed)
	{ PGout(11) = !GPIO_ReadOutputDataBit(GPIOG,GPIO_Pin_11);
		USART_SendString(bluetooth,"msg: RU is pressed\n");
	}
	if (RR.ispressed) {
		PGout(12) = !GPIO_ReadOutputDataBit(GPIOG,GPIO_Pin_12);
		USART_SendString(bluetooth,"msg: RR is pressed\n");
	}
	if (RD.ispressed) {
		PGout(13) = !GPIO_ReadOutputDataBit(GPIOG,GPIO_Pin_13);
		USART_SendString(bluetooth,"msg: RD is pressed\n");
	}
	if (RL.ispressed) {
		if (TIM9->CCR2<1500) TIM_SetCompare2(TIM9,1800);
		else TIM_SetCompare2(TIM9,1060);
		USART_SendString(bluetooth,"msg: RL is pressed\n");
	}
	if (L2.ispressed) {
		if (LU.ispressed){
			TIM14->CCER |= TIM_CCER_CC1E;
			PFout(5) = 0;
		}else				
		if (LD.ispressed){
			PFout(5) = 1;
			TIM14->CCER |= TIM_CCER_CC1E;
		}else{ 
		//USART_SendString(bluetooth,"msg: 000!\n");
			TIM14->CCER &= ~TIM_CCER_CC1E;
		}Xianding=0;
		if (LR.ispressed) Xianding -=2000;					
		if (LL.ispressed) Xianding +=2000;
		if (Xianding !=0){
			//USART_SendString(bluetooth,"msg: %d\n",Xianding);
			USART_SendString(UART4,"5V%d\r",Xianding);
		}else{
			//USART_SendString(bluetooth,"msg: %d\n",Xianding);
			USART_SendString(UART4,"5V0\r");
		}
	}else{ 
		//USART_SendString(bluetooth,"msg: 000!\n");
			TIM14->CCER &= ~TIM_CCER_CC1E;
			USART_SendString(UART4,"5V0\r");
	}
	if (R2.ispressed){
		Xianding=0;
		if (LU.ispressed) Xianding +=200;					
		if (LD.ispressed) Xianding -=200;
		if (Xianding !=0){
			//USART_SendString(bluetooth,"msg: %d\n",Xianding);
			USART_SendString(UART4,"3V%d\r",Xianding);
		}else{
			USART_SendString(UART4,"3V0\r");
		}
		Xianding=0;
		if (LL.ispressed) Xianding +=200;					
		if (LR.ispressed) Xianding -=200;
		if (Xianding !=0){
			//USART_SendString(bluetooth,"msg: %d\n",Xianding);
			USART_SendString(UART4,"4V%d\r",Xianding);
		}else{
			USART_SendString(UART4,"4V0\r");
		}
	}else{
		USART_SendString(UART4,"3V0\r4v0\r");
	}
	Xianding=0;
	if (L1.ispressed) Xianding +=200;					
	if (R1.ispressed) Xianding -=200;
	if (Xianding !=0){
		//USART_SendString(bluetooth,"msg: %d\n",Xianding);
		USART_SendString(UART4,"6V%d\r",Xianding);
	}else{
		USART_SendString(UART4,"6V0\r");
	}
	ChassisSpeed = 1000;
	if (!L2.ispressed&&!R2.ispressed){
	if (LU.ispressed) direction_angle = -PI/2;
	else if (LD.ispressed) direction_angle = PI/2;
	else if (LL.ispressed) direction_angle = PI;
	else if (LR.ispressed) direction_angle = 0;
	else ChassisSpeed = 0;}else
	ChassisSpeed = 0;
	chassis_handle(direction_angle, ChassisSpeed);
	if(OPEN_Hander == 1){
		chassis.END.X=chassis.pos_x;
		chassis.END.Y=chassis.pos_y;
		chassis.END.ANG=chassis.angle;
	}
}

void control_usart_TIM()
{
	int i;
	for (i=0;i<12;i++) 
	{
		if (b[i]->cnt>0) b[i]->cnt--;
		if (b[i]->cnt==0)
			 b[i]->ispressed=false;
	}
}

