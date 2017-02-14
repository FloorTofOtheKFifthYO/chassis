#include "main.h" 
#include "sys.h"
#include "chassis.h"

//extern u8 cmd, sticks[4];
//extern u8 ptr;

//ȫ�ֱ���������
bool g_stop_flag = false;
//--------------��ʱ�����ݸ�-------------

int ms = 0;

void TIM2_IRQHandler(void){
	if( TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET ) 
	{
		ms++;
		control_usart_TIM();
		TIM_ClearITPendingBit(TIM2, TIM_FLAG_Update);//��������жϱ�־λ����һֱ�ж�
	}	
}


//-----------------�ݸ����------------------------

int main(void)
{   
	int temp_speed = 0;
	int Hx, Hy;

	rcc_config();
	gpio_config();
	delay_init(168);  //��ʼ����ʱ����
	nvic_config();
	
	usart_init(bluetooth,115200);
	cmd_init();

	can_init();
	chassis_init();

	TIM2_Init();
	USART_SendString(bluetooth,"msg: Let's go!\n");
	
	OPEN_Hander = 0;

    while(1) 
	{
		if(g_stop_flag){//ֹͣһ���˶�
			chassis_stop();
		}else{
			chassis_updata();
			
			if(OPEN_Hander ==0){
				/**-------------------------�Զ�����--------------------------------**/		
				chassis_auto();
			}
			else if(OPEN_Hander ==1)
			{
				control_usart_main();
			}
		}
		delay_ms(2);
	}
}
