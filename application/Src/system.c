#include "system.h"

//ϵͳ��ʼ��
void system_init(void)
{
	HAL_TIM_Base_Start_IT(&htim4);//ϵͳʱ�ӳ�ʼ��
	CAN_Init();//can��ʼ��
	RC_init();//ң������ʼ��
	chassis_init();
}


