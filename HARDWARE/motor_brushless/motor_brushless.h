#ifndef MOTOR_BRUSHLESS_H_
#define MOTOR_BRUSHLESS_H_

#include "sys.h"

/* 无刷电机
 */
  
typedef struct{
	TIM_TypeDef* TIMx;	//定时器
	u8 OC;				//通道数
	int GetTimx;		//码盘板中使用的定时器号
	int *speed;
	float init_duty;	//初始占空比
}Brushless;
  
/**
  * @brief 初始化,只初始化对应的定时器和通道
  *     
  * @note	参数需要地址，因为需要对码盘板传回的数据进行绑定
  *
  * @param  motor_struct: 电机实体指针  
  *          
  * @retval void
  */
void Brushless_init(Brushless* motor_struct);

/**
  * @brief 更改占空比
  *     
  * @note	
  *
  * @param  motor_struct: 电机实体 
			float duty:   占空比
  *          
  * @retval void
  */
#define Brushless_duty(motor_struct, duty) \
	TIM_SetDuty(motor_struct.TIMx,duty,motor_struct.OC)


/**
  * @brief 设置速度,暂时不实现，由于电调用PWM控制，有延时，
  *        PID算法效果不好
  *     
  * @note	
  *
  * @param  motor_struct: 电机实体 
			float duty:   占空比
  *          
  * @retval void
  */
void Brushless_setSpeed(Brushless motor_struct,int speed);

#endif
