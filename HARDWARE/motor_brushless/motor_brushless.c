#include "motor_brushless.h"
#include "encoder.h"
#include "configuration.h"
#include "TIM.h"

extern SwitchTIM * Which;

void Brushless_init(Brushless* motor_struct)
{
	TIM_Init(motor_struct->TIMx,168-1, 20000-1, false);
	TIM_PWM_Init(motor_struct->TIMx, motor_struct->init_duty,1);
	SetUsed(motor_struct->GetTimx,2);
	switch(motor_struct->GetTimx)
	{
		case 1:
			motor_struct->speed = &(Which->GetTim1);
			break;
		case 3:
			motor_struct->speed = &(Which->GetTim3);
			break;
		case 4:
			motor_struct->speed = &(Which->GetTim4);
			break;
		case 5:
			motor_struct->speed = &(Which->GetTim5);
			break;
		case 8:
			motor_struct->speed = &(Which->GetTim8);
			break;
		default:
			break;
	}
	delay_ms(50);
}


