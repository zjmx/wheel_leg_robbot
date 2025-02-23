#include "system.h"

//系统初始化
void system_init(void)
{
	HAL_TIM_Base_Start_IT(&htim4);//系统时钟初始化
	CAN_Init();//can初始化
	RC_init();//遥控器初始化
	chassis_init();
}


