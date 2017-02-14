#ifndef MOTOR_USART_H_
#define MOTOR_USART_H_

#include "usart.h"

/**
 * 该文件是串口控制博创电机的一个模版
 * 使用该文件中的结构体进行初始化
 * 该结构体应作为实例保存在调用者中
*/

typedef struct
{
	USART_TypeDef* usartx;
	int id;		//节点号
	int AC; 	//加速度 0~30000
	int DEC; 	//减速度 0~30000
	int SP; 	//最大速度 0~30000
	int MV;		//最小速度 0~30000
	float rate;	//调节参数
}motor_usart;

/**
  * @brief  重新按照结构体中的值复位驱动器参数，并保存到flash
  *     
  * @note	
  *
  * @param  motor_struct: 电机实体  
  *          
  * @retval void
  */
void motor_usart_reset(motor_usart motor_struct);

/**
  * @brief  设置加速度，并保存到flash
  *     
  * @note	
  *
  * @param  motor_struct: 电机实体  
  *          
  * @retval void
  */
void motor_usart_AC(motor_usart motor_struct);

/**
  * @brief  设置减速度，并保存到flash
  *     
  * @note	
  *
  * @param  motor_struct: 电机实体  
  *          
  * @retval void
  */
void motor_usart_DEC(motor_usart motor_struct);

/**
  * @brief  设置最小速度，并保存到flash
  *     
  * @note	
  *
  * @param  motor_struct: 电机实体  
  *          
  * @retval void
  */
void motor_usart_MV(motor_usart motor_struct);

/**
  * @brief  设置最大速度，并保存到flash
  *     
  * @note	
  *
  * @param  motor_struct: 电机实体  
  *          
  * @retval void
  */
void motor_usart_SP(motor_usart motor_struct);

/*
 * 电机一般使用速度模式和位置模式
 * 所以需要两个接口，设置速度和设置位置的接口
 * 位置又分为相对位置和绝对位置
 *
**/

//速度
#define motor_usart_setspeed(motor_struct, speed) \
	USART_SendString(motor_struct.usartx,"%dv%d\r%dv%d\r%dv%d\r", \
	motor_struct.id, motor_struct.rate * (speed),motor_struct.id, motor_struct.rate * (speed),motor_struct.id, motor_struct.rate * (speed));
	
	
//相对位置
#define motor_usart_relpos(motor_struct, relpos) \
	USART_SendString(MOTOR_USARTx,"%dLR%d\r", motor_struct.id, relpos); \
	USART_SendString(MOTOR_USARTx,"%dM\r%dM\r%dM\r",motor_struct.id,motor_struct.id,motor_struct.id);


//绝对位置
#define motor_usart_abspos(motor_struct, abspos) \
	USART_SendString(MOTOR_USARTx,"%dLA%d\r", motor_struct.id, abspos); \
	USART_SendString(MOTOR_USARTx,"%dM\r%dM\r%dM\r",motor_struct.id,motor_struct.id,motor_struct.id);

#endif

