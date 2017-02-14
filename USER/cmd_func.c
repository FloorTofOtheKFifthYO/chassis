#include "cmd_func.h"
#include "cmd.h"
#include "stdlib.h"
#include "step.h"
#include "encoder.h"
#include "configuration.h"
#include "string.h"
#include "delay.h"
#include "math.h"
#include "param.h"
#include "chassis.h"

extern s8 ptrS,ptrB;
extern Param * param;
extern bool g_stop_flag;

u8 target=1;       				//目标0-6

static list_node * now_pos_ptr;
static Pos_data * now_pos;     //当前点的数据指针
static float motor_v;

void cmd_reboot_func(int argc,char *argv[]){
    NVIC_SystemReset();
}

void cmd_stop_func(int argc,char *argv[]){
    if(argc == 1){
        g_stop_flag = !g_stop_flag;
    }else if(argc == 2){
        g_stop_flag = atoi(argv[1]);
    }
}

void cmd_hello_func(int argc,char *argv[]){
    USART_SendString(CMD_USARTx, "msg: Hello World\n");
}

void cmd_test_func(int argc,char *argv[]){
	if(argc == 1){
		
	}
	if(argc == 2){
		OPEN_Hander = 0;
		USART_SendString(MOTOR_USARTx,"5LA%d\r",(int)(atof(argv[1])*10000));
		//delay_ms(1000);
		USART_SendString(MOTOR_USARTx,"5M\r5M\r5M\r");
	}else if(argc == 3){
		if(strcmp(argv[1],"air") == 0)
		{
			if(strcmp(argv[2],"1") == 0){
				PGout(12) = !PGout(12);
			}else if(strcmp(argv[2],"2") == 0){
				PGout(11) = !PGout(11);
			}
		}else{
			
		}
	}else if(argc == 4){
		if(strcmp(argv[1],"step") == 0)
		{
			OPEN_Hander = 0;
			if(strcmp(argv[2],"1") == 0){
				Step1_moveto(atoi(argv[3]));
			}else if(strcmp(argv[2],"2") == 0){
				Step2_moveto(atoi(argv[3]));
			}
		}
	}
}

void cmd_pos_func(int argc,char *argv[])
{
    int no;
    int no0;
    int i;
    float x,y;
    Pos_data *data;
    list_node * ptr;
    int erro_no;
    if (strcmp(argv[1], "now") == 0)
    {
        USART_SendString(CMD_USARTx, "x:%f y:%f\n", chassis.pos_x,chassis.pos_y);
		USART_SendString(CMD_USARTx, "pitch:0 roll:0 speed:0 yaw:%.6f\n",chassis.angle);
    }else
    if(strcmp(argv[1],"add") == 0){
        if(argc < 5){
            USART_SendString(CMD_USARTx,"msg: Error!please enter:\n");
            USART_SendString(CMD_USARTx,"msg:    pos add <no> <x> <y>\n");
        }
        no = atoi(argv[2]);
        x = atof(argv[3]);
        y = atof(argv[4]);
        data = (Pos_data *)malloc(sizeof(Pos_data));
        data->x = x;
        data->y =  y;
        for (i = 0; i < 7; ++i)
        {
            data->d[i].launch_num = 0;
            list_init(&data->d[i].launch_ptr);
        }
        if((erro_no = list_insert(&param-> pos_ptr, no, data)) <= 0){
            USART_SendString(CMD_USARTx,"msg: Error:%d\n",erro_no);
        }else{
			param->pos_num++;
			now_pos_ptr = list_locate(&param->pos_ptr,no);
			now_pos = now_pos_ptr->data;
		}
		print_pos_list(param->pos_ptr->link);
    }else if(strcmp(argv[1],"print") == 0){
        print_pos_list(param->pos_ptr->link);
    }else if(strcmp(argv[1],"del") == 0){
        no = atoi(argv[2]);
        ptr = list_locate(&param->pos_ptr, no);
        if (ptr != NULL)
        {
            data = ptr->data;
            for (i = 0; i < 7; ++i)
            {
                clear_launch(&data->d[i].launch_ptr); 
            }
            free(data);
            list_remove_num(&param->pos_ptr,no);
            param->pos_num-=1;
			USART_SendString(CMD_USARTx,"msg: del success\n");
        }
        print_pos_list(param->pos_ptr->link);
    }else if(strcmp(argv[1],"clear") == 0){
        clear_pos(&param->pos_ptr);
        param->pos_num = 0;
    }else if(strcmp(argv[1],"save") == 0){
        erro_no = param_save();
        if(erro_no < 0){
            USART_SendString(CMD_USARTx,"msg: Save error:%d\n",erro_no);
        }
    }else if(strcmp(argv[1],"modi") == 0){
        no = atoi(argv[2]);
        if((data = local_pos(no)) == NULL){
            USART_SendString(CMD_USARTx,"msg: Not found point No:%d\n",no);
            return;
        }
        if(argc < 5){
            USART_SendString(CMD_USARTx,"msg: Error cmd format\n");
            return;
        }
         x = atof(argv[3]);
         y = atof(argv[4]);

        data->x =  x;
        data->y =  y;
        print_pos_list(param->pos_ptr->link);
    }else if (strcmp(argv[1], "jmp")==0)
    {
        no = atoi(argv[2]);
        no0 = atoi(argv[3]);
        ptr = list_locate(&param->pos_ptr, no);
        if (ptr != NULL)
        {
            if((node_move(&param->pos_ptr, no0, ptr)) == 0)
				USART_SendString(CMD_USARTx, "msg: Error\n");
			else
				USART_SendString(CMD_USARTx, "msg: jmp success\n");
        }else{
            USART_SendString(CMD_USARTx, "msg: Error\n");
        }
        print_pos_list(param->pos_ptr->link);
    }
}

void cmd_action_func(int argc,char *argv[])
{
    int no;
    float x, y, v;
    float yaw;
    list_node * ptr;
    if (strcmp(argv[1],"rotate")==0){
        v = atof(argv[2]);
        yaw = atof(argv[3]);
        //测试底盘电机转动到一定角度
    }else if (argc == 1)
    {
        //跑到下一个点
		if(now_pos_ptr)
        {
			now_pos_ptr = now_pos_ptr->link;
			if(now_pos_ptr)
			{
				now_pos = now_pos_ptr->data;
				x = now_pos->x;
				y = now_pos->y;
				chassis.END.X = x;
				chassis.END.Y = y;
				chassis.END.ANG = chassis.angle;
				OPEN_Hander = 0;
				chassis.car_state = car_ready;
			}
		}
    }else if (argc == 2){
        no = atoi(argv[1]);
        ptr = list_locate(&param->pos_ptr, no);
		if(ptr){
			now_pos = ptr->data;
			now_pos_ptr = ptr;
			x = now_pos->x;
			y = now_pos->y;
			chassis.END.X = x;
			chassis.END.Y = y;
			chassis.END.ANG = chassis.angle;
			OPEN_Hander = 0;
		}
        //跑到指定的点去
    }else if (argc == 4){
        x = atof(argv[1]);
        y = atof(argv[2]);
		yaw = atof(argv[3]);
		
        //跑到指定的位置
		chassis.END.X = x;
		chassis.END.Y = y;
		chassis.END.ANG = yaw;
		OPEN_Hander = 0;
		chassis.car_state = car_ready;
    }
	
}

void cmd_switch_func(int argc,char *argv[])
{
    u8 state1,state2;
    state1 = GPIO_ReadInputDataBit(GPIOF, GPIO_Pin_11);
    state2 = GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_14);
    USART_SendString(CMD_USARTx, "roll_switch:%d pitch_switch:%d", state1, state2);
}

void cmd_param_func(int argc,char *argv[]){
	if (strcmp(argv[1],"speedmax")==0)
		chassis.Speed_max = atoi(argv[2]);
	else if (strcmp(argv[1],"movespeed")==0)
		chassis.Move_speed = atoi(argv[2]);
	else if (strcmp(argv[1],"moveradium")==0)
		chassis.Move_radium = atof(argv[2]);
	else if (strcmp(argv[1],"angleradium")==0)
		chassis.Angle_radium = atof(argv[2]);
	else if (strcmp(argv[1],"anglespeed")==0)
		chassis.Angle_speed = atoi(argv[2]);
	else if (strcmp(argv[1],"start")==0)
		chassis.Start_distance = atof(argv[2]);
}
