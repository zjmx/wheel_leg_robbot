#ifndef BALANCE_CHAIR_TASK_H
#define BALANCE_CHAIR_TASK_H

#include "main.h"
#include "usart_rc_dbus.h"
#include "can_receive.h"
#include "math.h"
#include "usart.h"
#include "string.h"
#include "cmsis_os.h"
#include "function.h"
#include "ins_task.h"
#include "bsp_dwt.h"
#include "BM_Ctrl.h"
#include "pid.h"
#include "VMC_calc.h"

//底盘数据结构体
typedef struct 
{
	BM_motor_measure_t BM_motor[2][3];		//BM电机数据返回结构体
	pid_type_def BM_motor_pid[2][3];			//BM电机PID数据结构体
	pid_type_def tp_err_pid;							//防劈叉补偿力矩PID
	pid_type_def leg_L_pid;								//腿长PID
	pid_type_def wheel_turn;							//偏航角补偿
	pid_type_def leg_roll_pid;						//偏航角补偿
	
	float Pitch_R;
	float d_Pitch_R;
	float Pitch_L;
	float d_Pitch_L;
	float Theta_error;								//双边腿夹角误差值
	
	float leg_tp;											//防劈叉补偿力矩
	float leg_L0_tp;									//腿长力矩
	float wheel_turn_tp;							//腿长力矩
	float leg_roll_tp;								//roll轴补偿力矩
	
	float v_filter;										//车体滤波后的速度
	float x_filter;										//车体滤波后的位移
	
	float v_set;											//目标速度
	float x_set;											//目标位移
	float target_v;
	
	float yaw_set;										//目标yaw轴角度
	
	float roll_set;										//目标roll轴角度
	
	float leg_set;										//目标腿长
	float leg_right_set;							//目标腿长
	float leg_left_set;							  //目标腿长
}Chassis_t;
extern Chassis_t Chassis;

void balance_chair_init(void);
void balance_chair_task(void);
void Balance_R_ctrl_loop(Chassis_t *Chassis_R,vmc_leg_t *vmc,float *LQR_K,INS_t *ins);
void Balance_L_ctrl_loop(Chassis_t *Chassis_L,vmc_leg_t *vmc,float *LQR_K,INS_t *ins);
void chassisR_feedback_update(Chassis_t *Chassis_R,vmc_leg_t *vmcR,INS_t *ins);//返回数据更新
void chassisL_feedback_update(Chassis_t *Chassis_L,vmc_leg_t *vmcL,INS_t *ins);

#endif



