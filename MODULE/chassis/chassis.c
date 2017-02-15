#include "chassis.h"
#include "vega.h"
#include "delay.h"
#include <math.h>
#include "motorCtrl.h"
#include "cmd.h"

Chassis chassis;

static int ChassisSpeed;
static int TURN_speed;
static int Chassis_motor0 =0 , Chassis_motor1 =0 , Chassis_motor2 =0 ;

//����vega��װ��ʽ�����������ӵ�ƫ��
static float CH_angle_M0 = PI/2 - 2*PI/3, CH_angle_M1 = PI/2, CH_angle_M2 = PI/2 + 2*PI/3;

void chassis_init(void)
{
	vega_init(&(chassis.g_vega_pos_x),&(chassis.g_vega_pos_y),&(chassis.g_vega_angle));
    vega_reset();
	delay_ms(1000);
	RoboModule_RESET(MOTOR0_ID,MOTOR1_ID,MOTOR2_ID,0);
	delay_ms(500);
	RoboModule_CHOICE_mode(SpeedMode ,MOTOR0_ID ,MOTOR1_ID,MOTOR2_ID,0);
	delay_ms(500);
	RoboModule_Add_Callback(databack,RoboModule_Feedback_Callback,MOTOR0_ID,MOTOR1_ID,MOTOR2_ID,0);
	RoboModule_SETUP(2,0,MOTOR0_ID,MOTOR1_ID,MOTOR2_ID,0);
	delay_ms(500);
	RoboModule_SET_speed(MOTOR0_ID ,5000 , 0);
	RoboModule_SET_speed(MOTOR1_ID ,5000 , 0);
	RoboModule_SET_speed(MOTOR2_ID ,5000 , 0);
	
	chassis.Speed_max = 3500;
	chassis.Move_radium = 0.0004;
	chassis.Angle_radium = 0.001;
	chassis.Angle_speed = 1000;
	chassis.Move_speed = 3500;
	chassis.Start_distance = 0.03;
	
	chassis.direction_angle = PI/2;
	chassis.START.X = chassis.g_vega_pos_x* 0.0001 * 0.81;
	chassis.START.Y = chassis.g_vega_pos_y* 0.0001 * 0.81;
	chassis.START.ANG = (chassis.g_vega_angle/180.f)*PI;
	chassis.END = chassis.START;
	chassis.car_state = car_stop;
	
}

//��ѭ����ѯ����
void chassis_updata()
{
	chassis.pos_x = chassis.g_vega_pos_x* 0.0001 * 0.81;
	chassis.pos_y = chassis.g_vega_pos_y* 0.0001 * 0.81;
	chassis.angle = (chassis.g_vega_angle/180.f)*PI;
	//USART_SendString(UART5,"\n\nX=%f   Y=%f  angle=%f\n",pos_x, pos_y , angle);
	ChassisSpeed= chassis.Move_speed;
}

/**��ѭ�����ֱ�����
  *������float directoion 	�����
  *		int speed			�ٶ�
  *�����ٶ��Ƿ�Ϊ0����Ϊ0����ֻ��һ��
  */
void chassis_handle(float directoion, int speed)
{
	static bool stop = false;
	
	Chassis_motor0 = (speed*cos((CH_angle_M0 + chassis.angle)-direction_angle - directoion));
	Chassis_motor1 = (speed*cos((CH_angle_M1 + chassis.angle)-direction_angle - directoion));
	Chassis_motor2 = (speed*cos((CH_angle_M2 + chassis.angle)-direction_angle - directoion));
	
	if(fabs(Chassis_motor0)<1e-6 && fabs(Chassis_motor1)<1e-6 && fabs(Chassis_motor2)<1e-6)
	{
		 if(!stop)
		 {
			stop = true;
			RoboModule_SET_speed(MOTOR0_ID ,5000 , 0);
			RoboModule_SET_speed(MOTOR1_ID ,5000 , 0);
			RoboModule_SET_speed(MOTOR2_ID ,5000 , 0);
		 }
	}else{
		stop = false;
		RoboModule_SET_speed(MOTOR0_ID ,5000 , Chassis_motor0);
		RoboModule_SET_speed(MOTOR1_ID ,5000 , Chassis_motor1);
		RoboModule_SET_speed(MOTOR2_ID ,5000 , Chassis_motor2);
	}
}

extern int ms;

/** ��ѭ�����Զ��ܵ��״̬ת�ƺ���
  * ͨ����chassis.END������Ŀ�ĵ�
  */
void chassis_auto()
{
	float errorAngle;
	float error_X, error_Y, Serror_X, Serror_Y, direrror_X,direrror_Y;
	double Sroute,Eroute;
	static float Oerror_X,Oerror_Y;
	static float dir_dot_X, dir_dot_Y; 
	static float distance;
	float factor = 3;
	
	float i;
	
	if(chassis.car_state == car_ready){
		chassis.START.X = chassis.g_vega_pos_x* 0.0001 * 0.81;
		chassis.START.Y = chassis.g_vega_pos_y* 0.0001 * 0.81;
		chassis.START.ANG = (chassis.g_vega_angle/180.f)*PI;
		ms = 0;
		chassis.car_state = car_running;
		
		Oerror_X = chassis.END.X - chassis.START.X;
		Oerror_Y = chassis.END.Y - chassis.START.Y;
		distance = sqrtf(powf(Oerror_X,2)+powf(Oerror_Y,2));
		Oerror_X = Oerror_X/distance;
		Oerror_Y = Oerror_Y/distance;
	}
	if(chassis.car_state == car_running){
		errorAngle = chassis.angle - chassis.END.ANG;
		error_X = chassis.END.X - chassis.pos_x;
		error_Y = chassis.END.Y - chassis.pos_y;
		
		Serror_X = chassis.START.X - chassis.pos_x;
		Serror_Y = chassis.START.Y - chassis.pos_y;
		
		Sroute = sqrt(powf(Serror_X,2)+powf(Serror_Y,2));
		Eroute = sqrt(powf(error_X,2)+powf(error_Y,2));
		
		i = Sroute+0.5;
		
		dir_dot_X = Oerror_X*i + chassis.START.X;
		dir_dot_Y = Oerror_Y*i + chassis.START.Y;
		
		direrror_X = dir_dot_X - chassis.pos_x;
		direrror_Y = dir_dot_Y - chassis.pos_y;
		
		if(factor * factor * (powf(Serror_X,2)+powf(Serror_Y,2)) < (powf(error_X,2)+powf(error_Y,2))) {//����
			if(Sroute > chassis.Start_distance) // 0<start_distance<1
				ChassisSpeed = Sroute*chassis.Move_speed * factor;
			else 
				ChassisSpeed = chassis.Start_distance * chassis.Move_speed * factor;
		}else {
			if((powf(error_X,2)+powf(error_Y,2)) > 1)
				ChassisSpeed = Eroute*chassis.Move_speed;
			else if(Eroute > 0.05)
				ChassisSpeed = sqrt(Eroute) * chassis.Move_speed;
			else
				ChassisSpeed = 89.44 * (powf(error_X,2)+powf(error_Y,2)) * chassis.Move_speed;
		}
		/*if((powf(error_X,2)+powf(error_Y,2)) > 1)
			ChassisSpeed = sqrt(powf(error_X,2)+powf(error_Y,2))*Move_speed;
		else if((powf(error_X,2)+powf(error_Y,2)) > 0.01)
			ChassisSpeed = sqrt(sqrt(powf(error_X,2)+powf(error_Y,2)))*Move_speed;
		else 
			ChassisSpeed = 31.6 * (powf(error_X,2)+powf(error_Y,2)) * Move_speed;
		*/
		if(ChassisSpeed>chassis.Speed_max)
			ChassisSpeed = chassis.Speed_max;
		
		if(errorAngle >= chassis.Angle_radium && errorAngle < 2)
		{          //�Ƕȵ���
			TURN_speed = -1*chassis.Angle_speed*errorAngle;
		}
		else if(errorAngle <=-chassis.Angle_radium && errorAngle > -2)
		{
			TURN_speed = -1*chassis.Angle_speed*errorAngle;
		}
		else if(errorAngle>=2)
		{
			TURN_speed =-1*chassis.Angle_speed*2;
		}
		else if(errorAngle<=-2)
		{
			TURN_speed =-1*chassis.Angle_speed*(-2);
		}
		else
		{
			TURN_speed= 0;
		}
		
		if(powf(error_X,2)+powf(error_Y,2) <= chassis.Move_radium)
		{//�Ѿ�����
			i = 1;
			ChassisSpeed = 0;
		}else{
			if(distance<=1 || powf(error_X,2)+powf(error_Y,2) <= 0.5 || Sroute >= distance)
			{	
				direction_angle = atan2(error_Y,error_X);
			}
			else
				direction_angle = atan2(direrror_Y,direrror_X);
		}

		Chassis_motor0 = (ChassisSpeed * cos((CH_angle_M0 - chassis.angle) - direction_angle) - TURN_speed);//Y�᷽������direction_angle����С������ڳ�������ϵ�ķ���
		Chassis_motor1 = (ChassisSpeed * cos((CH_angle_M1 - chassis.angle) - direction_angle) - TURN_speed);
		Chassis_motor2 = (ChassisSpeed * cos((CH_angle_M2 - chassis.angle) - direction_angle) - TURN_speed);
		
		USART_SendString(bluetooth,"%d,%d,%d,%f",Chassis_motor0,Chassis_motor1,Chassis_motor2,direction_angle);
		USART_SendString(bluetooth,",%d,%d,%d,%f\n",ReturnData(MOTOR0_ID)->Speed,ReturnData(MOTOR1_ID)->Speed,ReturnData(MOTOR2_ID)->Speed,Sroute);
		
		//USART_SendString(bluetooth,"dot x:%f,y:%f\n",dir_dot_X,dir_dot_Y);
		
		if(fabs(Chassis_motor2) < 2 && fabs(Chassis_motor1) < 2 && fabs(Chassis_motor0) < 2)
		{
			RoboModule_SET_speed(MOTOR0_ID ,5000 , 0);
			RoboModule_SET_speed(MOTOR1_ID ,5000 , 0);
			RoboModule_SET_speed(MOTOR2_ID ,5000 , 0);
			chassis.car_state = car_stop;
		}
		else
		{
			RoboModule_SET_speed(MOTOR0_ID ,5000 , Chassis_motor0);
			RoboModule_SET_speed(MOTOR1_ID ,5000 , Chassis_motor1);
			RoboModule_SET_speed(MOTOR2_ID ,5000 , Chassis_motor2);
		}
	}
	else if(chassis.car_state == car_stop)
	{
		if(ms>200)
			USART_SendString(CMD_USARTx,"msg: %fs\n",ms*5.0/10000);
		ms = 0;
		/*pos_x = temp_x* 0.0001 * 0.81;
		pos_y = temp_y* 0.0001 * 0.81;
		angle = (temp_angle/180.f)*PI;*/
		//state = ready;
		//OPEN_Hander = 1;
	}
}

//stop
void chassis_stop()
{
	RoboModule_SET_speed(MOTOR0_ID ,5000 , 0);
	RoboModule_SET_speed(MOTOR1_ID ,5000 , 0);
	RoboModule_SET_speed(MOTOR2_ID ,5000 , 0);
	delay_ms(5);
	RoboModule_SET_speed(MOTOR0_ID ,5000 , 0);
	RoboModule_SET_speed(MOTOR1_ID ,5000 , 0);
	RoboModule_SET_speed(MOTOR2_ID ,5000 , 0);
}
