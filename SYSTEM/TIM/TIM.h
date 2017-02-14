#ifndef __TIM_H
#define __TIM_H
#include "configuration.h"
 
/**
  * @brief  时钟初始化
  *
  * @param 	TIM_TypeDef *TIMx 		相应时钟
  *			uint16_t TIM_Prescaler 	分频数
  *			uint32_t TIM_Period 	装载值
  *         bool IT					是否开中断
  * @retval 
  */
void TIM_Init(TIM_TypeDef *TIMx, uint16_t TIM_Prescaler, uint32_t TIM_Period, bool IT);


/**
  * @brief  相应通道输出PWM
  *
  * @param 	TIM_TypeDef *TIMx 		相应时钟
  *			float duty				占空比
  *         int OC					通道号
  * @retval 
  */
void TIM_PWM_Init(TIM_TypeDef *TIMx, float duty, int OC);

/**
  * @brief  相应通道设置PWM占空比
  *
  * @param 	TIM_TypeDef *TIMx 		相应定时器
  *			float duty				占空比
  *         int OC					通道号
  * @retval 
  */
void TIM_SetDuty(TIM_TypeDef* TIMx,float duty, int OC);

void TIM13_Init();
void TIM14_Init();
void TIM2_Init();
void TIM9_Init();

#endif

