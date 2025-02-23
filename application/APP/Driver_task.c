#include "Driver_task.h"

#define DRIVER_TIME 10 //控制周期为10ms

extern RC_ctrl_t rc_ctrl;										//遥控器数据指针
extern Chassis_t Chassis;										//车体全体数据结构体

void Driver_task(void)
{
	rc_data_process(&rc_ctrl,&Chassis,(float)DRIVER_TIME/1000.0f);
	osDelay(DRIVER_TIME);
}

void rc_data_process(RC_ctrl_t *rc_data,Chassis_t *chassis,float dt)
{
	chassis->target_v=rc_data->rc.ch[3]*0.0008f;
	slope_following(&chassis->target_v,&chassis->v_set,0.05f);
	
	chassis->x_set=chassis->x_set+chassis->v_set*dt;
	
	chassis->leg_right_set=chassis->leg_right_set+rc_data->rc.ch[1]*(0.0000050f);
	if(chassis->leg_right_set>0.15f){chassis->leg_right_set=0.15f;}
	else if(chassis->leg_right_set<0.10f){chassis->leg_right_set=0.10f;}
	
	chassis->leg_left_set=chassis->leg_left_set+rc_data->rc.ch[1]*(0.0000050f);
	if(chassis->leg_left_set>0.15f){chassis->leg_left_set=0.15f;}
	else if(chassis->leg_left_set<0.10f){chassis->leg_left_set=0.10f;}
	
	chassis->yaw_set=chassis->yaw_set+rc_data->rc.ch[0]*(-0.000050f);
	LimitMax(&chassis->yaw_set,pi);
}


