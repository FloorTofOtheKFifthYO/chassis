#include "TIM.h"

TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
TIM_ICInitTypeDef   TIM_ICInitStructure;

/**
  * @brief  时钟初始化
  *
  * @param 	TIM_TypeDef *TIMx 		相应时钟
  *			uint16_t TIM_Prescaler 	分频数
  *			uint32_t TIM_Period 	装载值
  *         bool IT					是否开中断
  * @retval 
  */
void TIM_Init(TIM_TypeDef *TIMx, uint16_t TIM_Prescaler, uint32_t TIM_Period, bool IT)
{
	TIM_DeInit(TIMx);
	TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
	TIM_TimeBaseStructure.TIM_Prescaler = TIM_Prescaler;  //    
	TIM_TimeBaseStructure.TIM_Period = TIM_Period;  
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;  //设置了时钟分割
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;// 向上  
	TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure);
	if(IT){
		TIM_ClearFlag(TIMx, TIM_FLAG_Update);
		TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
	}
	TIM_Cmd(TIMx, ENABLE);
}

/**
  * @brief  相应通道输出PWM
  *
  * @param 	TIM_TypeDef *TIMx 		相应时钟
  *			float duty				占空比
  *         int OC					通道号
  * @retval 
  */
void TIM_PWM_Init(TIM_TypeDef *TIMx, float duty, int OC)
{
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = (int)(duty*(TIMx->ARR+1)) - 1;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low; //输出极性
	
	switch(OC)
	{
		case 1:
			TIM_OC1Init(TIMx, &TIM_OCInitStructure);  //根据T指定的参数初始化外设OC1
			TIM_OC1PreloadConfig(TIM9, TIM_OCPreload_Enable);  //使能TIMx在CCR1上的预装载寄存器
			break;
		case 2:
			TIM_OC2Init(TIMx, &TIM_OCInitStructure);  //根据T指定的参数初始化外设OC2
			TIM_OC2PreloadConfig(TIM9, TIM_OCPreload_Enable);  //使能TIMx在CCR2上的预装载寄存器
			break;
		case 3:
			TIM_OC3Init(TIMx, &TIM_OCInitStructure);  //根据T指定的参数初始化外设OC3
			TIM_OC3PreloadConfig(TIM9, TIM_OCPreload_Enable);  //使能TIMx在CCR3上的预装载寄存器
			break;
		case 4:
			TIM_OC4Init(TIMx, &TIM_OCInitStructure);  //根据T指定的参数初始化外设OC4
			TIM_OC4PreloadConfig(TIM9, TIM_OCPreload_Enable);  //使能TIMx在CCR4上的预装载寄存器
			break;
		default:
			break;
	}
	TIM_ARRPreloadConfig(TIMx,ENABLE);//ARPE使能 
	TIM_Cmd(TIMx, ENABLE);
}

/**
  * @brief  相应通道设置PWM占空比
  *
  * @param 	TIM_TypeDef *TIMx 		相应定时器
  *			float duty				占空比
  *         int OC					通道号
  * @retval 
  */
void TIM_SetDuty(TIM_TypeDef* TIMx,float duty, int OC)
{
	switch(OC)
	{
		case 1:
			TIM_SetCompare1(TIMx, (int)((TIMx->ARR + 1) * duty)-1);
			break;
		case 2:
			TIM_SetCompare2(TIMx, (int)((TIMx->ARR + 1) * duty)-1);
			break;
		case 3:
			TIM_SetCompare3(TIMx, (int)((TIMx->ARR + 1) * duty)-1);
			break;
		case 4:
			TIM_SetCompare4(TIMx, (int)((TIMx->ARR + 1) * duty)-1);
			break;
		default:
			break;
	}
}

void TIM2_Init()        //0.005ms 定时
{
	/*TIM2*/
	TIM_Init(TIM2,42000-1,1,true);
}

void TIM9_Init()  //无刷舵机PWM输出
{	
	TIM_Init(TIM9,168-1,20000-1,false);
	TIM_PWM_Init(TIM9,0.077,1);
	//TIM_PWM_Init(TIM9,0.052,2);
}

void TIM13_Init(void)
{
	TIM_Init(TIM13,84-1,1000-1,true);
	TIM_PWM_Init(TIM13,0.5,1);
	TIM13->CCER &= ~TIM_CCER_CC1E;
}

void TIM14_Init(void)
{
	TIM_Init(TIM14,84-1,1000-1,true);
	TIM_PWM_Init(TIM14,0.5,1);
	TIM14->CCER &= ~TIM_CCER_CC1E;
}

