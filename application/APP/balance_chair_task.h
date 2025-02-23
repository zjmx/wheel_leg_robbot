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

//�������ݽṹ��
typedef struct 
{
	BM_motor_measure_t BM_motor[2][3];		//BM������ݷ��ؽṹ��
	pid_type_def BM_motor_pid[2][3];			//BM���PID���ݽṹ��
	pid_type_def tp_err_pid;							//�����油������PID
	pid_type_def leg_L_pid;								//�ȳ�PID
	pid_type_def wheel_turn;							//ƫ���ǲ���
	pid_type_def leg_roll_pid;						//ƫ���ǲ���
	
	float Pitch_R;
	float d_Pitch_R;
	float Pitch_L;
	float d_Pitch_L;
	float Theta_error;								//˫���ȼн����ֵ
	
	float leg_tp;											//�����油������
	float leg_L0_tp;									//�ȳ�����
	float wheel_turn_tp;							//�ȳ�����
	float leg_roll_tp;								//roll�Ჹ������
	
	float v_filter;										//�����˲�����ٶ�
	float x_filter;										//�����˲����λ��
	
	float v_set;											//Ŀ���ٶ�
	float x_set;											//Ŀ��λ��
	float target_v;
	
	float yaw_set;										//Ŀ��yaw��Ƕ�
	
	float roll_set;										//Ŀ��roll��Ƕ�
	
	float leg_set;										//Ŀ���ȳ�
	float leg_right_set;							//Ŀ���ȳ�
	float leg_left_set;							  //Ŀ���ȳ�
}Chassis_t;
extern Chassis_t Chassis;

void balance_chair_init(void);
void balance_chair_task(void);
void Balance_R_ctrl_loop(Chassis_t *Chassis_R,vmc_leg_t *vmc,float *LQR_K,INS_t *ins);
void Balance_L_ctrl_loop(Chassis_t *Chassis_L,vmc_leg_t *vmc,float *LQR_K,INS_t *ins);
void chassisR_feedback_update(Chassis_t *Chassis_R,vmc_leg_t *vmcR,INS_t *ins);//�������ݸ���
void chassisL_feedback_update(Chassis_t *Chassis_L,vmc_leg_t *vmcL,INS_t *ins);

#endif



