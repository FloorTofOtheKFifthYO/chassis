#include "cmd.h"
#include "stm32f4xx_usart.h"
#include "stdlib.h"
#include "configuration.h"
#include "string.h"
#include "stdarg.h"
#include "cmd_func.h"
//#include "misc.h"
//#include "stm32f4xx_it.h"
static cmd_struct cmd_tbl[] = {
    /*
     * 若需添加命令，需要在此加上：
     * CMD_ADD("命令名","命令使用方法（可为空格,但不能不加双引号）",对应命令的执行函数名)
     * 注意最后一个不需要逗号，前面的都需要逗号
     */
    CMD_ADD("help"," Print all command and usage ",cmd_help_func),
    CMD_ADD("reboot"," 重启程序 ",cmd_reboot_func),
   // CMD_ADD("sensor","查看传感器状态 ",cmd_sensor_func),
    CMD_ADD("switch"," 查看触碰开关状态 ",cmd_switch_func),
    CMD_ADD("hello"," ",cmd_hello_func),
//    CMD_ADD("speed"," 设置底盘运动速度 ",cmd_speed_func),
//    CMD_ADD("launch"," 设置发射参数 ",cmd_launch_func),
    CMD_ADD("pos"," 设置全场定位的坐标 ",cmd_pos_func),
    CMD_ADD("action"," 使用命令执行各种动作 ",cmd_action_func),
    CMD_ADD("stop"," ",cmd_stop_func),
	CMD_ADD("param","系统参数",cmd_param_func),
	CMD_ADD("test", "haha!",cmd_test_func)
};

static char cmd_line[MAX_CMD_LINE_LENGTH + 1];
static char *cmd_argv[MAX_ARGC]; 

void cmd_init(void){
    int i;
#if CMD_PERIPH_INIT_EN == 1
    USART_InitTypeDef usart_init_stru;
    NVIC_InitTypeDef nvic_init_stru;
#if CMD_USARTn == 1
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);   
    GPIO_Configuration(GPIO_Pin_9,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOA);   //USART1 TX
    GPIO_Configuration(GPIO_Pin_10,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOA); //USART1 RX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);   
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);  
    nvic_init_stru.NVIC_IRQChannel = USART1_IRQn;
#elif CMD_USARTn == 2
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);
    GPIO_Configuration(GPIO_Pin_2,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOA);   //USART2 TX
    GPIO_Configuration(GPIO_Pin_3,GPIO_Mode_AF, GPIO_Speed_50MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOA); //USART2 RX
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);   
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);  
    nvic_init_stru.NVIC_IRQChannel = USART2_IRQn;
#elif CMD_USARTn == 3
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
    GPIO_Configuration(GPIO_Pin_10,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOB);   //USART3 TX
    GPIO_Configuration(GPIO_Pin_11,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOB); //USART3 RX
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_USART3);   
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_USART3); 
    nvic_init_stru.NVIC_IRQChannel = USART3_IRQn;
#elif CMD_USARTn == 5
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
    GPIO_Configuration(GPIO_Pin_12,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOC);   //USART5 TX
    GPIO_Configuration(GPIO_Pin_2,GPIO_Mode_AF, GPIO_Speed_100MHz,GPIO_OType_PP,GPIO_PuPd_NOPULL,GPIOD); //USART5 RX
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);   
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5); 
    nvic_init_stru.NVIC_IRQChannel = UART5_IRQn;
#endif
    usart_init_stru.USART_BaudRate = CMD_USART_BAUD;       //设置波特率
    usart_init_stru.USART_WordLength = USART_WordLength_8b;    //一个帧中传输的数据位数（字长为8位数据格式）
    usart_init_stru.USART_StopBits = USART_StopBits_1;     //一个停止位
    usart_init_stru.USART_Parity = USART_Parity_No;     //无奇偶校验
    usart_init_stru.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  //无硬件数据流控制
    usart_init_stru.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;  //发送和接受模式
    USART_Init(CMD_USARTx, &usart_init_stru);    //初始化串口    
    USART_Cmd(CMD_USARTx, ENABLE);    //使能串口
    USART_ITConfig(CMD_USARTx,USART_IT_RXNE, ENABLE);
    NVIC_PriorityGroupConfig(CMD_NVIC_GROUP);
    nvic_init_stru.NVIC_IRQChannelPreemptionPriority = CMD_NVIC_IRQPP;
    nvic_init_stru.NVIC_IRQChannelSubPriority = CMD_NVIC_IRQSP;
    nvic_init_stru.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&nvic_init_stru);
#endif
    for(i = 0;i < MAX_ARGC;i++){
        cmd_argv[i] = (char *)malloc(MAX_CMD_ARG_LENGTH + 1);
    }
}



int cmd_parse(char *cmd_line,int *argc,char *argv[]){
    char c_temp;
    int i = 0,arg_index = 0;
    int arg_cnt = 0;
    c_temp = cmd_line[i++];  
    while(c_temp != '\r'){
        if(c_temp == ' '){
            if(arg_index == 0){   //如果命令或者参数字符串第一个是空格，则忽略   
                c_temp = cmd_line[i++];
                continue;
            }
            //空格为参数或者命令的分隔符
            if(arg_cnt == MAX_ARGC){   //如果参数个数过多,则返回
                return -1;
            }
            argv[arg_cnt][arg_index] = 0;
            arg_cnt++;
            arg_index = 0;
            c_temp = cmd_line[i++];
            continue;
        }
        if(arg_index == MAX_CMD_ARG_LENGTH){   //如果参数长度过长，则报错返回
            return -2;
        }
        argv[arg_cnt][arg_index++] = c_temp;
        c_temp = cmd_line[i++];
    }
    if(arg_cnt == 0 && arg_index == 0){  //如果命令或者参数是空的，则返回
        return -3;
    }
    //最后一个参数的结束没有在上面的while循环中解析到
    argv[arg_cnt++][arg_index] = 0;
    *argc = arg_cnt;
    return 0;
}

int cmd_exec(int argc,char *argv[]){
    int cmd_index = 0;
    u32 cmd_num;
 
    cmd_num = sizeof(cmd_tbl)/sizeof(cmd_tbl[0]);

    if(argc == 0){  //如果参数是空的，则返回
        return -1;
    }
    for(cmd_index = 0;cmd_index < cmd_num;cmd_index++){   //查找命令
        if(strcmp((char *)(cmd_tbl[cmd_index].cmd_name),(char *)argv[0]) == 0){  //如果找到了命令，则执行命令相对应的函数
            cmd_tbl[cmd_index].cmd_func(argc,argv);
            return 0;
        }
    }
    return -2;
}


#if CMD_USARTn == 1
void USART1_IRQHandler(void){
#elif CMD_USARTn == 2
void USART2_IRQHandler(void){
#elif CMD_USARTn == 3
void USART3_IRQHandler(void){
#elif CMD_USARTn == 5
void UART5_IRQHandler(void){
#endif 
    static u32 cmd_line_length = 0;
    int cmd_argc,i;
    int erro_n;
    u8 c_recv;
    
	if(USART_GetITStatus(CMD_USARTx,USART_IT_RXNE) != RESET){      
        //USART_ClearITPendingBit(CMD_USARTx,USART_IT_RXNE);
        c_recv = USART_ReceiveData(CMD_USARTx);           
        if(c_recv == '\n'){  //接受完一次指令
            if(cmd_line_length == 0){
                return;
            }
            erro_n = cmd_parse(cmd_line,&cmd_argc,cmd_argv);  //解析命令
            if(erro_n < 0){
                //打印函数执行错误信息
                if(erro_n == -3){
                cmd_line_length = 0;
                memset(cmd_line,0,MAX_CMD_LINE_LENGTH + 1);
                return;
                }else if(erro_n == -2){
                    USART_SendString(CMD_USARTx,"msg: 命令参数长度过长\n");
                }else if(erro_n == -1){
                    USART_SendString(CMD_USARTx,"msg: 命令参数过多\n");
                }
                cmd_line_length = 0;
                memset(cmd_line,0,MAX_CMD_LINE_LENGTH + 1);
                return;
            }
            erro_n = cmd_exec(cmd_argc,cmd_argv);   //执行命令
            if(erro_n < 0){
                //打印函数执行错误信息
                if(erro_n == -2){
                      USART_SendString(CMD_USARTx,"msg: 未找到命令:%s\r\n",cmd_argv[0]);
                }
                cmd_line_length = 0;
                memset(cmd_line,0,MAX_CMD_LINE_LENGTH + 1);
                return;
            }
            cmd_line_length = 0;
            memset(cmd_line,0,MAX_CMD_LINE_LENGTH + 1);
        }else{
            if(cmd_line_length == MAX_CMD_LINE_LENGTH){
                //打印命令行太长的信息
                cmd_line_length = 0;
                return;
            }
            cmd_line[cmd_line_length++] = (char)c_recv;
        }           
    }
}

void cmd_help_func(int argc,char *argv[]){
    int i;
    u32 cmd_num;
    cmd_num = sizeof(cmd_tbl)/sizeof(cmd_tbl[0]);
    if(argc > 1){
        USART_SendString(CMD_USARTx,"msg: help命令参数过多\n");        
        return;         
    }
    for(i = 0;i < cmd_num;i++){  
        USART_SendString(CMD_USARTx,"msg: cmd:%s   usage:%s\n",cmd_tbl[i].cmd_name,cmd_tbl[i].cmd_usage);
    }
}
