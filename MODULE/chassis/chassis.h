#ifndef _UNDERPAN_H_
#define _UNDERPAN_H_
#include "sys.h"
#include <stdbool.h>
#include "configuration.h"

/*应定义出
	#ifndef CHASSIS
	#define CHASSIS

	#define MOTOR0_ID 0x0A
	#define MOTOR1_ID 0x0B
	#define MOTOR2_ID 0x0C

	#endif
*/
#ifndef CHASSIS
#error 未定义底盘
#endif

typedef struct 
{
	float X;
	float Y;
	float ANG;
}POSITION;

typedef struct
{
	//全场定位传回的数据
	int g_vega_pos_x; 
	int g_vega_pos_y;   
	float g_vega_angle;
	
	float pos_x,pos_y;
	float angle;
	
	//状态
	enum {car_stop, car_running, car_ready} car_state;//车的运动状态
	
	//控制
	POSITION END, START;
	
	//参数
	float direction_angle;
	int Speed_max;
	float Move_radium;
	float Angle_radium;
	int Angle_speed;
	int Move_speed;
	float Start_distance;
} Chassis;

extern Chassis chassis;

/** 底盘初始化
  * 包括全场定位的初始化
  *
  */
void chassis_init(void);

//主循环轮询更新
void chassis_updata();

/**主循环中手柄控制
  *参数：float directoion 	方向角
  *		int speed			速度
  *会检查速度是否为0，若为0，则只发一次
  */
void chassis_handle(float directoion, int speed);

/** 主循环中自动跑点的状态转移函数
  * 通过改chassis.END来控制目的地
  */
void chassis_auto();

//stop
void chassis_stop();

#endif
