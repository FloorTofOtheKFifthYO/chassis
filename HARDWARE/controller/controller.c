#include "controller.h"
#include "delay.h"
#include <math.h>

uint8_t data[10] = {0};
uint8_t PSX_CONN[5] = {0x01,0x43,0x00,0x01,0x00};                       //进入设置模式
uint8_t PSX_MODE[9] = {0x01,0x44,0x00,0x01,0x03,0x00,0x00,0x00,0x00};  //设置为Analog Mode
uint8_t PSX_POLL[9] = {0x01,0x42,0x00,0x00,0x00,0x00,0x00,0x00,0x00};  //轮询
uint8_t PSX_EXIT[9] = {0x01,0x43,0x00,0x00,0x5A,0x5A,0x5A,0x5A,0x5A};  //退出，这个在Motor版本里被删掉了

struct key_t keys[KEYS_NUM] = {0};

void sending_config(void)
{
	keys[LU_KEY].data_pos = 4;
	keys[LD_KEY].data_pos = 4;
	keys[LL_KEY].data_pos = 4;
	keys[LR_KEY].data_pos = 4;
	keys[RU_KEY].data_pos = 5;
	keys[RD_KEY].data_pos = 5;
	keys[RL_KEY].data_pos = 5;
	keys[RR_KEY].data_pos = 5;
	keys[L1_KEY].data_pos = 5;
	keys[L2_KEY].data_pos = 5;
	keys[R1_KEY].data_pos = 5;
	keys[R2_KEY].data_pos = 5;

	keys[LU_KEY].id = 0x10;
	keys[LD_KEY].id = 0x40;
	keys[LL_KEY].id = 0x80;
	keys[LR_KEY].id = 0x20;
	keys[RU_KEY].id = 0x10;
	keys[RD_KEY].id = 0x40;
	keys[RL_KEY].id = 0x80;
	keys[RR_KEY].id = 0x20;
	keys[L1_KEY].id = 0x04;
	keys[L2_KEY].id = 0x01;
	keys[R1_KEY].id = 0x08;
	keys[R2_KEY].id = 0x02;
}


int handle_init(){
//	GPIO_Configuration(GPIO_Pin_14,GPIO_Mode_IN_FLOATING, GPIO_Speed_50MHz,GPIOB);
//	GPIO_Configuration(GPIO_Pin_12,GPIO_Mode_Out_PP, GPIO_Speed_50MHz,GPIOB);
	
	int i,try_times = 0;
	uint8_t PSX_Ana = 0x00;	   //进入Analog Mode的标志
	//uint8_t PSX_Ana_check = 0;
	
	GPIO_InitTypeDef GPIO_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;        
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, ATT);
	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;      
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	
	while(PSX_Ana != 255)	//0xF3是Analog Mode的Data值   255
	{
	    GPIO_ResetBits(GPIOB, ATT);	   //拉低ATT
		delay_us(5);	
		for (i = 0; i < 5; i++)	   //进入设置模式
		{ 
			
			SPI_I2S_SendData(controller, PSX_CONN[i]); 					//发送   //0x01,0x43,0x00,0x01,0x00
			while(SPI_I2S_GetFlagStatus(controller, SPI_I2S_FLAG_RXNE) == RESET){} 	//等待接收完成
 			SPI_I2S_ReceiveData(controller);				 //读数据
		}			 
		GPIO_SetBits(GPIOB, ATT);					 //拉高ATT
		delay_us(5);									 //延时
		GPIO_ResetBits(GPIOB, ATT); 				 //再拉低	
		delay_us(5);
		for (i = 0; i < 9; i++)					 //设置为Analog Mode
		{ 
			SPI_I2S_SendData(controller, PSX_MODE[i]);   //0x01,0x44,0x00,0x01,0x03,0x00,0x00,0x00,0x00
			while(SPI_I2S_GetFlagStatus(controller, SPI_I2S_FLAG_RXNE) == RESET){} 		
				if (i == 1){
					PSX_Ana = SPI_I2S_ReceiveData(controller);
//					if(PSX_Ana == 255){
//						uprintf(USART3,"PSX_Ana = 0x%x\n",PSX_Ana);
//					}
				}
				else
					SPI_I2S_ReceiveData(controller);
		}	
	 	GPIO_SetBits(GPIOB, ATT);
		delay_us(5);				  
//		GPIO_ResetBits(GPIOB, ATT);	
//		delay_us(5);
//		for (i = 0; i < 9; i++)					 //检查是否设置成功
//		{ 		
//			SPI_I2S_SendData(controller, PSX_EXIT[i]); 
//			while(SPI_I2S_GetFlagStatus(controller, SPI_I2S_FLAG_RXNE) == RESET){}  //0x01,0x43,0x00,0x00,0x5A,0x5A,0x5A,0x5A,0x5A
//			if (i == 1){
//				PSX_Ana_check = SPI_I2S_ReceiveData(controller);
////				uprintf(USART3,"PSX_Ana = 0x%x\n",PSX_Ana);
////				uprintf(USART3,"PSX_Ana_check = 0x%x\n",PSX_Ana_check);
//			}
//			else	 
//					SPI_I2S_ReceiveData(controller);
//		}
//		GPIO_SetBits(GPIOB, ATT);
//		delay_us(5);//2
//		if(PSX_Ana != 255){
			try_times++;
			if(try_times == 100){  //尝试连接100次不成功
				return -1;
			}
			delay_ms(100);
//		}
//		printf("PSX_Ana = 0x%x\n",PSX_Ana);
	}		
	return 0;
}
void handler_test(void){
	USART_SendString(DEBUG,"%x\t%x\t%x\t%x\t%x\n",data[4],data[5],data[6],data[7],data[8]);
}

void check(void){
	int i;
	GPIO_ResetBits(GPIOB, ATT); 
	delay_us(5);
			 
	for (i = 0; i < 9; i++)					   //发送轮询数据包
	{
		SPI_I2S_SendData(SPI2, PSX_POLL[i]);
		while(SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET); 
		data[i] = SPI_I2S_ReceiveData(SPI2);
	}
	GPIO_SetBits(GPIOB, ATT);
	delay_us(5);
}
void update(void)
{
	uint8_t i;
	for(i = 0; i < KEYS_NUM; i++) {
		struct key_t *key = keys + i;

		key->is_pressed = !(data[key->data_pos] & key->id);

		if(key->is_pressed) {
			key->pressed_times++;
		} else {
			key->is_pressed = 0;
		}
	}
}
bool isp(struct key_t *k)
{
	if(k->pressed_times >= CMD_TIMES)
		return true;
	else
		return false;
}

void send_control_data(void)
{
	int8_t spd_x, spd_y, r_spd;
	uint8_t i;
	static uint8_t old_spd_x, old_spd_y, old_spd_r;
	static uint8_t loop_times;         //attention
	update();
	
//	if(isp(keys + L1_KEY)&&
//		isp(keys + L2_KEY)&&
//		isp(keys + R1_KEY)&&
//		isp(keys + R2_KEY)) { //L1 L2 R1 R2

////			#ifdef DEBUG_KEYS
////			printf("l1 l2 r1 r2 pressed\n");
////			#endif
//			
////			delay_ms(200);
//	} else 
	if(isp(keys + RR_KEY)) { 
      //        ○

	} else if(isp(keys + L1_KEY)) { //L1组合键

		if(isp(keys + LU_KEY)) {
//           L1+up
		} else if(isp(keys + LD_KEY)) {
//           L1+down
		} else if(isp(keys + LL_KEY)) {
//           L1+left
		} else if(isp(keys + LR_KEY)) {
//           L1+right
		} else {
			keys[L1_KEY].pressed_times = 0;
		}

	} else if(isp(keys + L2_KEY)) {  //L2组合键

		if(isp(keys + LU_KEY)) {
//           L2+up
		} else if(isp(keys + LD_KEY)) {
//           L2+down
		} else if(isp(keys + LL_KEY)) {
//           L2+left
		} else if(isp(keys + LR_KEY)){
//           L2+right
		} else {
			keys[L2_KEY].pressed_times = 0;
		}

	} else if(isp(keys + R2_KEY)) {  //R2组合键

		if(isp(keys + LU_KEY)) {
//           R2+up
		} else if(isp(keys + LD_KEY)) {
//           R2+down
		} else if(isp(keys + LL_KEY)) {
//           R2+left
		} else if(isp(keys + LR_KEY)){
//           R2+right
		} else if(isp(keys + RD_KEY)){
//           R2+×
		} else {
			keys[R2_KEY].pressed_times = 0;
		}

	}  else if(isp(keys + R1_KEY)) { //R1组合键
		if(isp(keys + LL_KEY)) {
//           R1+left
		} else if(isp(keys + LR_KEY)) {
//           R1+right
		} else {
			keys[R1_KEY].pressed_times = 0;
		}
	} else if(isp(keys + LU_KEY)) {
//           up
	} else if(isp(keys + LD_KEY)) {
//           down
	} else {
		
		loop_times++;
		if(loop_times > CMD_TIMES) {
			loop_times = 0;
			for(i = 0; i < KEYS_NUM; i++) {
				keys[i].pressed_times = 0;
			}
		}

	}
	
	r_spd = 0x7f - data[8];
	spd_x = data[6] - 0x80;
	spd_y = 0x7f - data[7];
	
	if(fabsf(r_spd) < HAND_ZERO) {
		r_spd = 0;
	}
	if(fabsf(spd_x) < HAND_ZERO) {
		spd_x = 0;
	}
	if(fabsf(spd_y) < HAND_ZERO) {
		spd_y = 0;
	}
	
	if(r_spd == old_spd_r && spd_x == old_spd_x && spd_y == old_spd_y) {
		return;
	} else {
//      L&R sticks
		old_spd_x = spd_x;
		old_spd_y = spd_y;
		old_spd_r = r_spd;
	}
}


