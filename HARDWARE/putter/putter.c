#include "putter.h"
#include "encoder.h"
#include <math.h>

extern SwitchTIM * Which;

static void putter_move(putter putter_struct, int v)
{
	if (v - putter_struct.SP*1.0 > 0.0000001)
		v = putter_struct.SP;
	else if (v + putter_struct.SP*1.0 < 0.0000001)
		v=-putter_struct.SP;
	motor_usart_setspeed(putter_struct.motor,v);
}

/**
  * @brief 初始化,将码盘板对应的定时器绑定
  *     
  * @note	参数需要地址，因为需要对码盘板传回的数据进行绑定
  *
  * @param  putter_struct: 电推杆实体指针  
  *          
  * @retval void
  */
void putter_init(putter *putter_struct)
{
	SetUsed(putter_struct->GetTimx,1);
	switch(putter_struct->GetTimx)
	{
		case 1:
			putter_struct->position = &(Which->GetTim1);
			break;
		case 3:
			putter_struct->position = &(Which->GetTim3);
			break;
		case 4:
			putter_struct->position = &(Which->GetTim4);
			break;
		case 5:
			putter_struct->position = &(Which->GetTim5);
			break;
		case 8:
			putter_struct->position = &(Which->GetTim8);
			break;
		defult:
			break;
	}
	putter_struct->d_last = 0;
	putter_struct->cnt = 0;
	putter_struct->putter_flag = false;
}

/**
  * @brief main函数中轮询部分
  *     
  * @note	
  *
  * @param  putter_struct: 电推杆实体  
  *          
  * @retval void
  */
void putter_main(putter putter_struct)
{
	float d,dd;
	
	if (putter_struct.putter_flag)
	{
		d = putter_struct.pur_pos - (*(putter_struct.position));
		dd = d - putter_struct.d_last;
		putter_struct.d_last = d;
		if (fabs(d) < 0.01)
		{
			if (putter_struct.cnt > 10)
			{
				putter_move(putter_struct,0);
				putter_struct.cnt = 0;
				putter_struct.putter_flag=false;
			}
			else
			{
				putter_struct.cnt++;
				putter_move(putter_struct,d * putter_struct.d_rate + dd * putter_struct.dd_rate);
			}
		}
		else
		{
			putter_struct.cnt = 0;
			putter_move(putter_struct,d * putter_struct.d_rate + dd * putter_struct.dd_rate);
		}
	}
}

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
void putter_moveto(putter *putter_struct,float pur_pos)
{
	putter_struct->pur_pos = pur_pos;
	putter_struct->putter_flag = true;
}

