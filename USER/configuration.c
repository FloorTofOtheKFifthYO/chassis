#include "configuration.h"

GPIO_InitTypeDef GPIO_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
EXTI_InitTypeDef exti_init_structure;



float direction_angle = 90;
//int WANTSPEED = 8000;
u8 OPEN_Hander = 0;



/*
 *函数名：void GPIO_Configuration(uint16_t GPIO_Pin,
                        GPIOMode_TypeDef  GPIO_Mode,
                        GPIOSpeed_TypeDef GPIO_Speed,
                        GPIOOType_TypeDef GPIO_OType,
                        GPIOPuPd_TypeDef GPIO_PuPd,
                        GPIO_TypeDef* GPIOx)
 * 功能：IO口初始化
 * 参数：uint16_t GPIO_Pin, 			管脚号
        GPIOMode_TypeDef  GPIO_Mode,	输出模式
        GPIOSpeed_TypeDef GPIO_Speed,	速度
        GPIOOType_TypeDef GPIO_OType,	407   GPIO_OType_PP 为推挽   GPIO_OType_OD 为开漏
        GPIOPuPd_TypeDef GPIO_PuPd,		GPIO_PuPd_DOWN GPIO_PuPd_UP GPIO_PuPd_NOPULL
        GPIO_TypeDef* GPIOx				管脚组别
 * 返回值：
 */
void GPIO_Configuration(uint16_t GPIO_Pin,GPIOMode_TypeDef  GPIO_Mode,GPIOOType_TypeDef GPIO_OType,
	GPIOSpeed_TypeDef GPIO_Speed,GPIOPuPd_TypeDef GPIO_PuPd,GPIO_TypeDef* GPIOx)
{
 
  	GPIO_InitStructure.GPIO_Pin = GPIO_Pin;                       //管脚号 
  	GPIO_InitStructure.GPIO_Mode = GPIO_Mode;                     //模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType;
  	GPIO_InitStructure.GPIO_Speed = GPIO_Speed;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd;                  
  	GPIO_Init(GPIOx, &GPIO_InitStructure);                        //管脚组别 
}


/*
 *函数名：void NVIC_Configuration(uint8_t NVIC_IRQChannel,
				uint8_t NVIC_IRQChannelPreemptionPriority,
				uint8_t NVIC_IRQChannelSubPriority,
                FunctionalState NVIC_IRQChannelCmd)
 * 功能：中断分组初始化
 * 参数：uint8_t NVIC_IRQChannel, 					 	使能按键所在的外部中断通道
   		uint8_t NVIC_IRQChannelPreemptionPriority,		抢占优先级
		uint8_t NVIC_IRQChannelSubPriority				子优先级
        FunctionalState NVIC_IRQChannelCmd              ENABLE DISABLE
 * 返回值：
 */
void NVIC_Configuration(uint8_t NVIC_IRQChannel,uint8_t NVIC_IRQChannelPreemptionPriority,
	uint8_t NVIC_IRQChannelSubPriority,FunctionalState NVIC_IRQChannelCmd)
{	
	NVIC_InitStructure.NVIC_IRQChannel = NVIC_IRQChannel;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=NVIC_IRQChannelPreemptionPriority;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =NVIC_IRQChannelSubPriority;		
	NVIC_InitStructure.NVIC_IRQChannelCmd = NVIC_IRQChannelCmd;			
	NVIC_Init(&NVIC_InitStructure);
}



void system_clk_set(void){ 
    ErrorStatus HSEStartUpStatus;
    RCC_DeInit();
 
    RCC_HSEConfig(RCC_HSE_ON );   //打开外部时钟
 
    HSEStartUpStatus = RCC_WaitForHSEStartUp();  //等待外部时钟打开至稳定
 
  if(HSEStartUpStatus == SUCCESS)     
  {
    FLASH_SetLatency(FLASH_Latency_5);   
    FLASH_PrefetchBufferCmd(ENABLE);       //flash时钟的相关配置  
    RCC_PLLCmd(DISABLE);  //配置PLL之前需要关闭PLL
    RCC_HCLKConfig(RCC_SYSCLK_Div1);   //HCLK分频
    RCC_PCLK2Config(RCC_HCLK_Div1);   //PCLK2分频
    RCC_PCLK1Config(RCC_HCLK_Div4);    //PCLK1分频
    RCC_PLLConfig(RCC_PLLSource_HSE, 8, 336, 2, 7);    //sysclk = 168MHZ  ,,计算公式参见数据手册
    RCC_PLLCmd(ENABLE); //使能PLL
 
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET){      //等待，，直到PLL使能完毕
    
    }
 
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);  //选择PLL时钟为系统时钟
 
    while(RCC_GetSYSCLKSource() != 0x08)       //等待，至PLL时钟设置成系统时钟完毕
       { 
       }
     }
    #if(__FPU_PRESENT == 1)&&(__FPU_USED == 1)
			SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));  //开启FPU
		#endif

}
 
 void rcc_config(){
	//RCC_DeInit();			//初始化为缺省值
	
	//system_clk_set();
	//RCC_GetClocksFreq(&RCC_ClockFreq);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
                                                            
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);	 //使能USART1时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); 
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);	//使能USART3时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);	//使能USART3时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);   //使能UART5时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);   //使能syscfg时钟，用于外部中断
	
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE); 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);
}

void gpio_config(void)
{
	
	//GPIO_PinRemapConfig(GPIO_Remap_USART1,ENABLE);   //?
	//GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);   //?
	//GPIO_PinRemapConfig(GPIO_Remap_USART3,ENABLE);   //?
	GPIO_Configuration(GPIO_Pin_9 | GPIO_Pin_10,GPIO_Mode_AF,
	                   GPIO_OType_PP,GPIO_Speed_50MHz,GPIO_PuPd_UP,GPIOA);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	GPIO_Configuration(GPIO_Pin_5 | GPIO_Pin_6,GPIO_Mode_AF,
	                   GPIO_OType_PP,GPIO_Speed_50MHz,GPIO_PuPd_UP,GPIOB);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2); 
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6,GPIO_AF_USART2); 
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5); //GPIOC12复用为USART5
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2,GPIO_AF_UART5); //GPIOD2复用为USART5
	GPIO_Configuration(GPIO_Pin_12,GPIO_Mode_AF,
					   GPIO_OType_PP,GPIO_Speed_50MHz,GPIO_PuPd_UP,GPIOC);

	GPIO_Configuration(GPIO_Pin_2 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_8 | GPIO_Pin_9,
					   GPIO_Mode_AF,GPIO_OType_PP,GPIO_Speed_50MHz,GPIO_PuPd_UP,GPIOD);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource8,GPIO_AF_USART3); //GPIOD8复用为USART3
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource9,GPIO_AF_USART3); //GPIOD9复用为USART3
	
	GPIO_Configuration(GPIO_Pin_10 | GPIO_Pin_11,GPIO_Mode_AF,
	                   GPIO_OType_PP,GPIO_Speed_50MHz,GPIO_PuPd_UP,GPIOC);
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOB6复用为USART4
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOB7复用为USART4
//----------------------------USART---------------------------------------------------------------------------------------------	
//	GPIO_Configuration(GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15,
//	                   GPIO_Mode_AF,GPIO_OType_PP,GPIO_Speed_100MHz,GPIO_PuPd_UP,GPIOB);
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_SPI2); 
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_SPI2); 
//	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_SPI2);
//----------------------------SPI2---------------------------------------------------------------------------------------------
	
	GPIO_Configuration(GPIO_Pin_12 | GPIO_Pin_13,GPIO_Mode_AF,GPIO_OType_PP,GPIO_Speed_50MHz,GPIO_PuPd_NOPULL,GPIOD);
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4); 	
	GPIO_PinAFConfig(GPIOD, GPIO_PinSource13, GPIO_AF_TIM4); 
	GPIO_Configuration(GPIO_Pin_6 | GPIO_Pin_7,GPIO_Mode_AF,GPIO_OType_PP,GPIO_Speed_50MHz,GPIO_PuPd_NOPULL,GPIOA);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_TIM3); 	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_TIM3); 
/*	GPIO_Configuration(GPIO_Pin_0 | GPIO_Pin_1,GPIO_Mode_AF,GPIO_OType_PP,GPIO_Speed_50MHz,GPIO_PuPd_NOPULL,GPIOA);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM5); 	
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM5); */
//----------------------------增量码盘---------------------------------------------------------------------------------------------
	
/*	GPIO_Configuration(GPIO_Pin_11,GPIO_Mode_IN,GPIO_OType_OD,GPIO_Speed_100MHz,GPIO_PuPd_UP,GPIOF);
	GPIO_Configuration(GPIO_Pin_14,GPIO_Mode_IN,GPIO_OType_OD,GPIO_Speed_100MHz,GPIO_PuPd_UP,GPIOE);*/
//----------------------------触碰开关---------------------------------------------------------------------------------------------
	
	GPIO_Configuration(GPIO_Pin_5, GPIO_Mode_AF,GPIO_OType_PP,GPIO_Speed_50MHz,GPIO_PuPd_UP,GPIOE);
	GPIO_PinAFConfig(GPIOE, GPIO_PinSource5, GPIO_AF_TIM9);
	GPIO_Configuration(GPIO_Pin_8|GPIO_Pin_9, GPIO_Mode_AF,GPIO_OType_PP,GPIO_Speed_50MHz,GPIO_PuPd_UP,GPIOF);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource8, GPIO_AF_TIM13);
	GPIO_PinAFConfig(GPIOF, GPIO_PinSource9, GPIO_AF_TIM14);
//----------------------------PWM---------------------------------------------------------------------------------------------

	GPIO_Configuration(GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13,GPIO_Mode_OUT,GPIO_OType_PP,GPIO_Speed_50MHz,GPIO_PuPd_DOWN,GPIOG);
//----------------------------继电器-----------------------------------------------------------------------------	

	GPIO_Configuration(GPIO_Pin_1|GPIO_Pin_4, GPIO_Mode_OUT,GPIO_OType_PP,GPIO_Speed_50MHz,GPIO_PuPd_UP,GPIOD);
//----------------------------LED-----------------------------------------------------------------------------	
}

/*
 * 函数名：void EXTI_Configuration(uint32_t EXTI_Line,
                EXTIMode_TypeDef EXTI_Mode,
                EXTITrigger_TypeDef EXTI_Trigger,
                FunctionalState EXTI_LineCmd)
 * 功能：外部中断初始化
 * 参数：uint32_t EXTI_Line,               中断线：EXTI_Line0~EXTI_Line15
        EXTIMode_TypeDef EXTI_Mode,         EXTI_Mode_Interrupt,EXTI_Mode_Event
        EXTITrigger_TypeDef EXTI_Trigger,   EXTI_Trigger_Rising,EXTI_Trigger_Falling, EXTI_Trigger_Rising_Falling
        FunctionalState EXTI_LineCmd        ENABLE DISABLE
 * 返回：
 */
void EXTI_Configuration(uint32_t EXTI_Line,
                EXTIMode_TypeDef EXTI_Mode,
                EXTITrigger_TypeDef EXTI_Trigger,
                FunctionalState EXTI_LineCmd)
{
    exti_init_structure.EXTI_Line = EXTI_Line;
    exti_init_structure.EXTI_Mode = EXTI_Mode;
    exti_init_structure.EXTI_Trigger = EXTI_Trigger;
    exti_init_structure.EXTI_LineCmd = EXTI_LineCmd;
    EXTI_Init(&exti_init_structure);
}

void EXTI_config()
{
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOF, EXTI_PinSource11);
	SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOE, EXTI_PinSource14);
	EXTI_Configuration(EXTI_Line11,EXTI_Mode_Interrupt,EXTI_Trigger_Falling,ENABLE);
	EXTI_Configuration(EXTI_Line14,EXTI_Mode_Interrupt,EXTI_Trigger_Falling,ENABLE);
	EXTI_ClearITPendingBit(EXTI_Line11); 
	EXTI_ClearITPendingBit(EXTI_Line14); 
}

void nvic_config()
{
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2 2:2   抢占：响应    共 3级:3级
	
	NVIC_Configuration(UART5_IRQn,2,3,ENABLE);
	NVIC_Configuration(USART1_IRQn,2,3,ENABLE);
	NVIC_Configuration(USART3_IRQn,2,3,ENABLE);
	NVIC_Configuration(TIM2_IRQn,0,2,ENABLE);
	NVIC_Configuration(TIM8_UP_TIM13_IRQn,1,1,ENABLE);
	NVIC_Configuration(TIM8_TRG_COM_TIM14_IRQn,1,1,ENABLE);
//	NVIC_Configuration(TIM5_IRQn,1,1,ENABLE);
//	NVIC_Configuration(EXTI15_10_IRQn, 0, 0, ENABLE);

	
}



