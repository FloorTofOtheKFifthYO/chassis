#include "sys.h"
#include "usart.h"
#include "motor_usart.h"
#include <assert.h>

/**
 * 该文件是串口控制博创电机的一个模版
 * 使用.h文件中的结构体进行初始化
 * 该结构体应作为实例保存在调用者中
*/

/**
  * @brief  重新按照结构体中的值复位驱动器参数，并保存到flash
  *     
  * @note	
  *
  * @param  motor_struct: 电机实体  
  *          
  * @retval void
  */
  
void motor_usart_reset(motor_usart motor_struct)
{
	assert(motor_struct.AC>=0 && motor_struct.AC<=30000);
	assert(motor_struct.DEC>=0 && motor_struct.DEC<=30000);
	assert(motor_struct.MV>=0 && motor_struct.MV<=30000);
	assert(motor_struct.SP>=0 && motor_struct.SP<=30000);
	USART_SendString(motor_struct.usartx,"%dAC%d\r%dDEC%d\r%dMV\r%dSP%d\rEEPSAV\r",motor_struct.id,motor_struct.AC,
	motor_struct.id,motor_struct.DEC,motor_struct.id,motor_struct.MV,motor_struct.id,motor_struct.SP);
}


/**
  * @brief  设置加速度，并保存到flash
  *     
  * @note	
  *
  * @param  motor_struct: 电机实体  
  *          
  * @retval void
  */
void motor_usart_AC(motor_usart motor_struct)
{
	assert(motor_struct.AC>=0 && motor_struct.AC<=30000);
	USART_SendString(motor_struct.usartx,"%dAC%d\rEEPSAV\r",motor_struct.id,motor_struct.AC);
}

/**
  * @brief  设置减速度，并保存到flash
  *     
  * @note	
  *
  * @param  motor_struct: 电机实体  
  *          
  * @retval void
  */
void motor_usart_DEC(motor_usart motor_struct)
{
	assert(motor_struct.DEC>=0 && motor_struct.DEC<=30000);
	USART_SendString(motor_struct.usartx,"%dDEC%d\rEEPSAV\r",motor_struct.id,motor_struct.DEC);
}

/**
  * @brief  设置最小速度，并保存到flash
  *     
  * @note	
  *
  * @param  motor_struct: 电机实体  
  *          
  * @retval void
  */
void motor_usart_MV(motor_usart motor_struct)
{
	assert(motor_struct.MV>=0 && motor_struct.MV<=30000);
	USART_SendString(motor_struct.usartx,"%dMV%d\rEEPSAV\r",motor_struct.id,motor_struct.MV);
}

/**
  * @brief  设置最大速度，并保存到flash
  *     
  * @note	
  *
  * @param  motor_struct: 电机实体  
  *          
  * @retval void
  */
void motor_usart_SP(motor_usart motor_struct)
{
	assert(motor_struct.SP>=0 && motor_struct.SP<=30000);
	USART_SendString(motor_struct.usartx,"%dSP%d\rEEPSAV\r",motor_struct.id,motor_struct.SP);
}