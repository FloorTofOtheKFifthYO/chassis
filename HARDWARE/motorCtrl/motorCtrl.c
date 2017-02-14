#include "motorCtrl.h"
#include "can.h"
#include "stdarg.h"
#include <stdio.h>
#include <stdlib.h>
#include "string.h"

u8 motor_message[8];
BackDATA *Backdata = NULL;
BackDATA *TempData = NULL;
u16 CheckMotor = 0xFFFF;

/*
*功能：复位
*参数： Number为电机编号
*	   
*
*/
void RoboModule_RESET(char Number , ...){
	
	u8 CANID = CANID_OFFSET_MOTORRESET;
	char buffer[MOTOR_MAX];
	int i = 0;
	va_list arg_ptr;
	char number;
	
	if (Backdata == NULL)
	{
		Backdata = (BackDATA *)malloc(sizeof(BackDATA));
	}
	if (TempData == NULL)
	{
		TempData = (BackDATA *)malloc(sizeof(BackDATA)*16);
	}
	
	if(Number > 15){
		Number = 0;
	}
	motor_message[0] = 0x55;
	motor_message[1] = 0x55;
	motor_message[2] = 0x55;
	motor_message[3] = 0x55;
	motor_message[4] = 0x55;
	motor_message[5] = 0x55;
	motor_message[6] = 0x55;
	motor_message[7] = 0x55;
	
	if(Number == 0){
		CANID = CANID_OFFSET_MOTORRESET;
		CANID |= Number<<4;
		can_send_msg(CANID , motor_message , 8);
		return ;
	}
	
	va_start(arg_ptr, Number);  
	//vsnprintf(buffer, MOTOR_MAX, &Number, arg_ptr);
	
	while(i < MOTOR_MAX){
		number  = va_arg(arg_ptr,char);
		if (number >15||number ==0){
			break;
		}
		CANID = CANID_OFFSET_MOTORRESET;
		CANID |= number<<4;
		can_send_msg(CANID , motor_message , 8);
		i++;
	}
	
	CANID = CANID_OFFSET_MOTORRESET;
	CANID |= Number<<4;
	can_send_msg(CANID , motor_message , 8);
	
	va_end(arg_ptr);
}

/*
*功能：模式选择
*参数：Number为电机编号
*	   Mode为所选模式,有 CurrentMode,SpeedMode,PositionMode
*
*
*/
void RoboModule_CHOICE_mode(u8 Mode , char Number , ...){
	u8 CANID = CANID_OFFSET_MOTORMODE;
	char buffer[MOTOR_MAX];
	int i = 0;
	va_list arg_ptr;
	char number = 0;
	
	if (Number > 15){
		return ;
	}
	motor_message[0] = Mode;
	motor_message[1] = 0x55;
	motor_message[2] = 0x55;
	motor_message[3] = 0x55;
	motor_message[4] = 0x55;
	motor_message[5] = 0x55;
	motor_message[6] = 0x55;
	motor_message[7] = 0x55;
	
	
	va_start(arg_ptr, Number);  
	//vsnprintf(buffer, MOTOR_MAX, &Number, arg_ptr);
	
	while(i < MOTOR_MAX){
		number  = va_arg(arg_ptr,char);
		if (number >15||number ==0){
			break;
		}
		CANID = CANID_OFFSET_MOTORMODE;
		CANID |= number<<4;
		can_send_msg(CANID , motor_message , 8);
		i++;
	}
	CANID = CANID_OFFSET_MOTORMODE;
	CANID |= Number<<4;
	can_send_msg(CANID , motor_message , 8);
	
	va_end(arg_ptr);
}

/*
*功能：配置
*参数： Number：为电机编号
*	   Time：为信息返回周期
*	   CTL1：可以决定 CTL1 和 CTL2 端口在作为左右限位功能后，在电平发生变化时，是否对外发送当前的电平值。
*注意：驱动器只在进入上文的 8 种运动模式之后，才会周期性的对外发送以上所述的电流、速度、位置等信息。
*	   同理，只有进入上文的 8 种运动模式之后，才会在 CTL 端口电平变化后对外发送其电平值
*/

void  RoboModule_SETUP(u8 Time ,u8 CTL1 , char Number , ...){
	u8 CANID = CANID_OFFSET_MOTORSETUP;
	char buffer[MOTOR_MAX];
	int i = 0;
	va_list arg_ptr;
	char number;
	
	if (Number > 15){
		return ;
	}
	motor_message[0] = Time;
	motor_message[1] = CTL1;
	motor_message[2] = 0x55;
	motor_message[3] = 0x55;
	motor_message[4] = 0x55;
	motor_message[5] = 0x55;
	motor_message[6] = 0x55;
	motor_message[7] = 0x55;
	
	
	va_start(arg_ptr, Number);  
	
	while(i < MOTOR_MAX){
		number  = va_arg(arg_ptr,char);
		if (number >15||number <=0){
			break;
		}
		CANID = CANID_OFFSET_MOTORSETUP;
		CANID |= number<<4;
		can_send_msg(CANID , motor_message , 8);
		i++;
	}
	
	CANID = CANID_OFFSET_MOTORSETUP;
	CANID |= Number<<4;
	can_send_msg(CANID , motor_message , 8);
	
	va_end(arg_ptr);
}

/*
*功能：设置速度
*参数： Number为电机编号
*	   PWM为P波限定，范围0~5000
*	   speed为设置速度，范围0~30000+，我们的电机是0~8100
*
*/
void RoboModule_SET_speed(u8 Number ,u16 PWM , int speed){
	u8 CANID = CANID_OFFSET_MOTORSPEED;
	if(Number > 15){
		return ;
	}
	
	CANID |= Number<<4;
	
	motor_message[0] = (unsigned char)((PWM>>8)&0xff);
	motor_message[1] = (unsigned char)(PWM&0xff);
	motor_message[2] = (unsigned char)((speed>>8)&0xff);
	motor_message[3] = (unsigned char)(speed&0xff);
	motor_message[4] = 0x55;
	motor_message[5] = 0x55;
	motor_message[6] = 0x55;
	motor_message[7] = 0x55;
	
	can_send_msg(CANID , motor_message , 8);
}

/*
*功能：设置位置
*参数： Number为电机编号
*	   PWM为P波限定，范围0~5000
*	   Position设置位置，范围0 ~ 2^31
*
*/
void RoboModule_SET_Position(u8 Number ,u16 PWM , s32 Position){
	
	u8 CANID = CANID_OFFSET_MOTORPLACE;
	if(Number > 15){
		return ;
	}
	
	CANID |= Number<<4;
	
	motor_message[0] = (unsigned char)((PWM>>8)&0xff);
	motor_message[1] = (unsigned char)((PWM)&0xff);
	motor_message[2] = 0x55;
	motor_message[3] = 0x55;
	motor_message[4] = (unsigned char)((Position>>24)&0xff);
	motor_message[5] = (unsigned char)((Position>>16)&0xff);
	motor_message[6] = (unsigned char)((Position>>8)&0xff);
	motor_message[7] = (unsigned char)(Position&0xff);
	
	can_send_msg(CANID , motor_message , 8);
}

/*
*功能：在线检测，可以检测电机是否连接上
*参数：number：电机编号
*		范围：1~15，否则全部检测
*

**/
void RoboModule_Check(char Number , ...){
	u8 CANID = CANID_OFFSET_MOTORSCHECK;
	char buffer[MOTOR_MAX];
	int i = 0;
	va_list arg_ptr;
	char number;
	
	motor_message[0] = 0x55;
	motor_message[1] = 0x55;
	motor_message[2] = 0x55;
	motor_message[3] = 0x55;
	motor_message[4] = 0x55;
	motor_message[5] = 0x55;
	motor_message[6] = 0x55;
	motor_message[7] = 0x55;
	
	if(Number >15||Number == 0){
		Number = 0;
		CANID = CANID_OFFSET_MOTORSCHECK;
		CANID |= Number<<4;
		can_send_msg(CANID , motor_message , 8);
	}else {
		
		va_start(arg_ptr, Number);  
		//vsnprintf(buffer, MOTOR_MAX, &Number, arg_ptr);
		
		while(i < MOTOR_MAX){
			number  = va_arg(arg_ptr,char);
			if (number >15||number <=0){
				break;
			}
			CANID = CANID_OFFSET_MOTORSCHECK;
			CANID |= number<<4;
			CheckMotor &= ~(0x01<<(number-1));
			can_send_msg(CANID , motor_message , 8);
			i++;
		}
		
		CANID = CANID_OFFSET_MOTORSCHECK;
		CANID |= Number<<4;
		CheckMotor &= ~(0x01<<(Number-1));
		can_send_msg(CANID , motor_message , 8);
		va_end(arg_ptr);
		
		i = 2000;
		while(i--);
	}
	
}

/*
*功能 ：添加回调函数
*参数：Number：电机编号
*      msg_rcv_callback_func：接收函数指针
*	  Backmode :接收信息类型，databack为反馈数据 ,onlineback为在线检测返回
*/
void RoboModule_Add_Callback(u8 BackMode ,void *msg_rcv_callback_func,char Number, ...){
	
	u8 CANID = BackMode;
	char buffer[MOTOR_MAX];
	int i = 0;
	va_list arg_ptr;
	char number;
	
	va_start(arg_ptr, Number);  
	
	while(i < MOTOR_MAX){
		number  = va_arg(arg_ptr,char);
		if (number >15||number <=0){
			break;
		}
		CANID = BackMode;
		CANID |= number<<4;
		can_add_callback(CANID , msg_rcv_callback_func);
		i++;
	}
	
	CANID = BackMode;
	CANID |= Number<<4;
	can_add_callback(CANID , msg_rcv_callback_func);
	
	va_end(arg_ptr);
}

/*
*功能：回调函数
*
*
*
***/
void RoboModule_Feedback_Callback(CanRxMsg *can_rx_msg){
	u8 Tell_ApartOFFSET = (can_rx_msg->StdId)&0x0F;
	u8 Tell_ApartNumber = (can_rx_msg->StdId)>>4;
	u8 i = 0;
	u8 j = 0;
	
	switch(Tell_ApartOFFSET){
		case databack:
			Backdata->Current = can_rx_msg->Data[0];
			Backdata->Current = ((Backdata->Current)<<8)|can_rx_msg->Data[1];
			Backdata->Speed = can_rx_msg->Data[2];
			Backdata->Speed = ((Backdata->Speed)<<8)|can_rx_msg->Data[3];
			Backdata->Position = can_rx_msg->Data[4];
			Backdata->Position = ((Backdata->Position)<<8)| can_rx_msg->Data[5];
			Backdata->Position = ((Backdata->Position)<<8)|can_rx_msg->Data[6];
			Backdata->Position = ((Backdata->Position)<<8)| can_rx_msg->Data[7];

			TempData[Tell_ApartNumber].Current = Backdata->Current;//将结果缓存到数组里，只储存最近一次的结果
			TempData[Tell_ApartNumber].Speed = Backdata->Speed;
			TempData[Tell_ApartNumber].Position = Backdata->Position;
			break;
		case onlineback:
			CheckMotor |= 0x01<<(Tell_ApartNumber-1);//电机正常或者异常电机已经恢复
			break;
		default :break;
	}
}

/*
*功能 ：返回电机最近返回数据指针
*参数 ：电机编号
*使用方法：定义 BackDATA 类型指针，指针等于该函数返回值
*
*
*/
BackDATA* ReturnData(u8 Number){
	return &TempData[Number];
}

/*
*功能 ：返回电机检查结果
*参数 ：无
*使用方法：若果全部正常则返回0xFF，否则说明电机有异常，异常电机编号即为是0的相应位，比如返回值为0xF8,说明标号为4的电机异常
*
*
*/
u16 CheckMOTOR(){
	return CheckMotor;
}