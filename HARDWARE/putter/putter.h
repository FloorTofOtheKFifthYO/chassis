#ifndef _PUTTER_H_
#define _PUTTER_H_
#include "sys.h"
#include <stdbool.h>
#include "motor_usart.h"

typedef struct{
	motor_usart motor;
	int GetTimx;		//码盘板中使用的定时器号
	int SP;				//最大速度
	
	int *position;		//位置
	float pur_pos;		//预期位置
	float d_last;
	int d_rate;
	int dd_rate;
	bool putter_flag;
	int cnt;
}putter;

/**
  * @brief 初始化,将码盘板对应的定时器绑定
  *     
  * @note	参数需要地址，因为需要对码盘板传回的数据进行绑定
  *
  * @param  putter_struct: 电推杆实体指针  
  *          
  * @retval void
  */
void putter_init(putter *putter_struct);

/**
  * @brief main函数中轮询部分
  *     
  * @note	
  *
  * @param  putter_struct: 电推杆实体  
  *          
  * @retval void
  */
void putter_main(putter putter_struct);

/**
  * @brief 推电推杆
  *     
  * @note	
  *
  * @param  motor_struct: 电推杆实体指针，
			floatpur_pos   希望到达的位置
  *          
  * @retval void
  */
void putter_moveto(putter *putter_struct,float pur_pos);
#endif
