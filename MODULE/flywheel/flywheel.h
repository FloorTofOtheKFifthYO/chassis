#ifndef _FLYWHEEL_H_
#define _FLYWHEEL_H_
#include "sys.h"
#include <stdbool.h>
#include "configuration.h"
#include "motor_brushless.h"

/** 在各模块中的.h文件注释里指明每个部件应该分配的资源，如下：
  * 确定资源分配后，将下面的写在configuration.h
  * #ifndef BRUSHLESS
  *
  * #define BRUSHLESS
  * #define BRUSHLESS_TIM 		TIMx			//定时器
  * #define BRUSHLESS_OC 		1				//通道
  * #define BRUSHLESS_GPIOx 	GPIOx			//GPIO
  *	#define BRUSHLESS_PIN GPIO_	Pin_x			//管脚
  * #define BRUSHLESS_PINSOURCE GPIO_PinSourcex //复用管脚
  * #define BRUSHLESS_AF 		GPIO_AF_TIMx	//管脚复用功能
  * #endif
  *
  * 用一个宏定义将配置包起来，方便下面检查
  
  */
#ifndef BRUSHLESS
#error 未定义无刷
#endif


#endif
