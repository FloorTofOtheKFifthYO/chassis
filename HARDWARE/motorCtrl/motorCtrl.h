
#include "sys.h"


#define MOTOR_MAX 16

//电机can总线ID基数
#define MOTOR10_CANID_BASE 0x0A0//0组10号电机
#define MOTOR11_CANID_BASE 0x0B0
#define MOTOR12_CANID_BASE 0x0C0

//指令选择偏移数
#define CANID_OFFSET_MOTORRESET 0x000//复位
#define CANID_OFFSET_MOTORMODE 0x001//模式选择
#define CANID_OFFSET_MOTORELEC 0x003//电流模式
#define CANID_OFFSET_MOTORSPEED 0x004//速度模式
#define CANID_OFFSET_MOTORPLACE 0x005//位置模式
#define CANID_OFFSET_MOTORSETUP 0X00A//配置模式
#define CANID_OFFSET_MOTORSCHECK 0x00F//在线监测

#define CANID_OFFSET_FEEDBACK 0x00B //获取反馈数据
#define CANID_OFFEST_CHECKONLINE 0x00F //接收在线监测信息

#define CurrentMode 0x02
#define SpeedMode 0x03
#define PositionMode 0x04

#define databack CANID_OFFSET_FEEDBACK
#define onlineback CANID_OFFEST_CHECKONLINE

typedef struct {
	short Current;
	short Speed;
	long Position;
}BackDATA ;

void RoboModule_RESET(char Number , ...);
void RoboModule_CHOICE_mode(u8 Mode , char Number , ...);
void RoboModule_SETUP(u8 Time ,u8 CTL1 , char Number , ...);
void RoboModule_SET_speed(u8 Number ,u16 PWM , int speed);;
void RoboModule_SET_Position(u8 Number ,u16 PWM , s32 Position);
void RoboModule_Check(char Number , ...);
void RoboModule_Add_Callback(u8 BackMode ,void *msg_rcv_callback_func,char Number, ...);
void RoboModule_Feedback_Callback(CanRxMsg *can_rx_msg);
BackDATA* ReturnData(u8 Number);
u16 CheckMOTOR();
