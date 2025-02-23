#include "balance_chair_task.h"

Chassis_t Chassis;													//车体全体数据结构体
vmc_leg_t vmc_right;												//右腿vmc
vmc_leg_t vmc_left;													//左腿vmc
extern RC_ctrl_t rc_ctrl;										//遥控器数据指针
extern INS_t INS;														//mcu数据结构体

//PID系数
float BM_right_pid[3][3]={{500.0f,120.0f,10.0f},{200.0f,0,100.0f},{200.0f,0,100.0f}};
float BM_left_pid[3][3]={{500.0f,120.0f,10.0f},{200.0f,0,100.0f},{200.0f,0,100.0f}};
float BM_tp_err_pid[3]={300.0f,0,100.0f};
float BM_leg_L_pid[3]={50.0f,0,15.0f};
float BM_wheel_turn_pid[3]={80.0f,0,15.0f};
float BM_leg_roll_pid[3]={100.0f,0,15.0f};
//LQR增益矩阵K初始化     
float LQR_K_R[12]={-18.8448,  -1.9941,   -1.6472,  -5.8039,  14.6346,  1.5244,//轮毂力矩  phi0 dphi0 x dx pitch dpitch
									 13.9567,   0.8796,    1.0032,   3.4700,   48.1959,  1.0041};//关节力矩 phi0 dphi0 x dx pitch dpitch
float LQR_K_L[12]={-18.8448,  -1.9941,   -1.6472,  -5.8039,  14.6346,  1.5244,//轮毂力矩  phi0 dphi0 x dx pitch dpitch
									 13.9567,   0.8796,    1.0032,   3.4700,   48.1959,  1.0041};//关节力矩 phi0 dphi0 x dx pitch dpitch
//三次函数线性化LQR增益矩阵
float Poly_Coefficient[12][4]={ {-248.989064502438,   279.214362868555,  -152.437333999771,  -1.40691899227407},
															  {-1.01788424377763,   2.09117693229754,  -14.1370976983001,  0.0826944572282850},
															  {-7.25924339956220,   6.36536748358899,  -0.214288113418487, -0.133700986931148},
																{-23.9778377756431,   21.3578607955453,  -7.24638407895371,  -5.11389110657298},
																{-234.078571291078,   243.160872107869,  -102.006435467889,  25.2703713668271},
																{-3.35633821298145,   4.31955054767682,  -2.32191206811098,  1.78682919633411},
																
																{227.000548927736,    -203.182861611737, 64.7028096919680,   8.03250630253053},
																{18.1617991896724,    -16.0012863874266, 4.45854346311373,   0.507753621828622},
																{-43.5682883994163,   42.3133747072427,  -15.4385567341941,  2.51747009466697},
																{-149.078691415823,   144.634581581944,  -52.7458352600435,  8.64278172349427},
																{575.613543040908,    -528.173260624629, 176.505306109859,   31.6030947534232},
																{37.2098506860765,    -35.7990400543555, 12.9094161337950,   -0.255475342848792}};
									 
//系统采样时间
#define SYS_delay 1
																
//底盘参数初始化
void balance_chair_init(void)
{
	//PID初始化
	PID_init(&Chassis.BM_motor_pid[0][0],BM_right_pid[0],16384,16384);
	PID_init(&Chassis.BM_motor_pid[0][1],BM_right_pid[1],7500,7500);
	PID_init(&Chassis.BM_motor_pid[0][2],BM_right_pid[2],7500,7500);
	PID_init(&Chassis.BM_motor_pid[1][0],BM_left_pid[0],16384,16384);
	PID_init(&Chassis.BM_motor_pid[1][1],BM_left_pid[1],7500,7500);
	PID_init(&Chassis.BM_motor_pid[1][2],BM_left_pid[2],7500,7500);
	
	PID_init(&Chassis.tp_err_pid,BM_tp_err_pid,7500,7500);//防劈叉pid
	PID_init(&Chassis.leg_L_pid,BM_leg_L_pid,7500,7500);//腿长pid
	PID_init(&Chassis.wheel_turn,BM_wheel_turn_pid,16384,16384);//转向pid
	PID_init(&Chassis.leg_roll_pid,BM_leg_roll_pid,7500,7500);//roll轴补偿pid
	
	//VMC初始化
	VMC_init(&vmc_right);
	VMC_init(&vmc_left);
	
	//目标yaw轴角度初始化
	Chassis.yaw_set=0.0f;
	//目标roll轴角度初始化
	Chassis.roll_set=0.0f;
	
	//腿长初始化
	Chassis.leg_right_set=0.15f;
	Chassis.leg_left_set=0.15f;
	
	//关节电机使能
	uint8_t ctrl_mode[8];
	for(int i=0;i<8;i++){ctrl_mode[i]=BM_enable_mode_joint;}
	BM_ctrl_mode_joint_set(&hcan1,ctrl_mode);
	BM_ctrl_mode_joint_set(&hcan2,ctrl_mode);
}

//底盘运动函数
void balance_chair_task(void)
{
	while(INS.ins_flag==0)
	{//等待accel收敛
		osDelay(SYS_delay);
	}
	//初始化
	balance_chair_init();
	while(1)
	{
		//获取系统时间
		
		//更新系统状态向量
		chassisR_feedback_update(&Chassis,&vmc_right,&INS);
		chassisL_feedback_update(&Chassis,&vmc_left,&INS);
		
		//计算双边腿夹角误差
		Chassis.Theta_error=(vmc_left.theta+vmc_right.theta);		
		
		//底盘右腿输出控制环
		Balance_R_ctrl_loop(&Chassis,&vmc_right,LQR_K_R,&INS);
		Balance_L_ctrl_loop(&Chassis,&vmc_left,LQR_K_L,&INS);
		
		//计算电机PID输出
		PID_calc(&Chassis.BM_motor_pid[0][0],Chassis.BM_motor[0][0].torque,Chassis.BM_motor[0][0].set_torque);
		PID_calc(&Chassis.BM_motor_pid[0][1],Chassis.BM_motor[0][1].torque,vmc_right.torque_set[0]);
		PID_calc(&Chassis.BM_motor_pid[0][2],Chassis.BM_motor[0][2].torque,vmc_right.torque_set[1]);
		PID_calc(&Chassis.BM_motor_pid[1][0],Chassis.BM_motor[1][0].torque,Chassis.BM_motor[1][0].set_torque);
		PID_calc(&Chassis.BM_motor_pid[1][1],Chassis.BM_motor[1][1].torque,vmc_left.torque_set[0]);
		PID_calc(&Chassis.BM_motor_pid[1][2],Chassis.BM_motor[1][2].torque,vmc_left.torque_set[1]);
		
		if(rc_ctrl.rc.s[1]==1)
		{
			//传入电机期望力矩值

			BM_motor_ctrl(&hcan1,BM_ID_range1,Chassis.BM_motor_pid[0][0].out,Chassis.BM_motor_pid[0][1].out,Chassis.BM_motor_pid[0][2].out,0);
			BM_motor_ctrl(&hcan2,BM_ID_range2,Chassis.BM_motor_pid[1][0].out,Chassis.BM_motor_pid[1][1].out,Chassis.BM_motor_pid[1][2].out,0);
			
//			BM_motor_ctrl(&hcan1,BM_ID_range1,0,Chassis.BM_motor_pid[0][1].out,Chassis.BM_motor_pid[0][2].out,0);
//			BM_motor_ctrl(&hcan2,BM_ID_range2,0,Chassis.BM_motor_pid[1][1].out,Chassis.BM_motor_pid[1][2].out,0);
			
//			BM_motor_ctrl(&hcan1,BM_ID_range1,Chassis.BM_motor_pid[0][0].out,0,0,0);
//			BM_motor_ctrl(&hcan2,BM_ID_range2,Chassis.BM_motor_pid[1][0].out,0,0,0);
		}
		else
		{
			BM_motor_ctrl(&hcan1,BM_ID_range1,0,0,0,0);
			BM_motor_ctrl(&hcan2,BM_ID_range2,0,0,0,0);
		}
		osDelay(SYS_delay);
	}
}

//底盘右腿输出控制环
void Balance_R_ctrl_loop(Chassis_t *Chassis_R,vmc_leg_t *vmc_R,float *LQR_K,INS_t *ins)
{
	//VMC计算L0长度和角度phi0,d_phi0,该任务控制周期是3*0.001秒
	VMC_calc_1_right(vmc_R,ins,SYS_delay*3.0f/1000.0f);
	
	//根据对应的线性化方程系数计算LQR_K
//	for(int i=0;i<12;i++)
//	{
//		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],Chassis_R->leg_right_set);
//	}
	
	//防劈叉补偿
	Chassis_R->leg_tp=PID_calc(&Chassis_R->tp_err_pid,Chassis_R->Theta_error,0.0f);
	//腿长补偿
	Chassis_R->leg_L0_tp=PID_calc(&Chassis_R->leg_L_pid,vmc_R->L0,Chassis_R->leg_right_set);
	//偏航角补偿
	Chassis_R->wheel_turn_tp=BM_wheel_turn_pid[0]*(Chassis_R->yaw_set-ins->Yaw)-BM_wheel_turn_pid[2]*ins->Gyro[2];
	//roll轴补偿力矩
	Chassis_R->leg_roll_tp=PID_calc(&Chassis_R->leg_roll_pid,ins->Roll,Chassis_R->roll_set);
	
	//电机输出期望赋值
	Chassis_R->BM_motor[0][0].set_torque=LQR_K[0]*vmc_R->theta   														//phi0     
																			+LQR_K[1]*vmc_R->d_theta														//dphi0
																			+LQR_K[2]*(Chassis_R->x_filter-Chassis_R->x_set)		//x
																			+LQR_K[3]*(Chassis_R->v_filter-Chassis_R->v_set)		//dx
																			+LQR_K[4]*Chassis_R->Pitch_R												//pitch
																			+LQR_K[5]*Chassis_R->d_Pitch_R;											//dpitch
	vmc_R->Tp=LQR_K[6]*vmc_R->theta  															//phi0     
					 +LQR_K[7]*vmc_R->d_theta															//dphi0
					 +LQR_K[8]*(Chassis_R->x_filter-Chassis_R->x_set)			//x
					 +LQR_K[9]*(Chassis_R->v_filter-Chassis_R->v_set)			//dx
					 +LQR_K[10]*ins->Pitch																//pitch
					 +LQR_K[11]*ins->Gyro[0];															//dpitch
	//偏航角补偿
	Chassis_R->BM_motor[0][0].set_torque=Chassis_R->BM_motor[0][0].set_torque-Chassis_R->wheel_turn_tp;											
												
	//防劈叉力矩补偿											
	vmc_R->Tp=vmc_R->Tp-Chassis_R->leg_tp;

	vmc_R->F0=(15.0f*0.50f*9.80f)/arm_cos_f32(vmc_R->theta)+Chassis_R->leg_L0_tp-Chassis_R->leg_roll_tp;//前馈+pd
//	LimitMax(&Chassis_R->VMC_data.F0,172.3f);
												
	//计算期望的关节输出力矩
	VMC_calc_2(vmc_R);
	
//	//轮毂电机力矩限幅,限制量为电机额定力矩
//	LimitMax(&Chassis->BM_motor[0].set_torque,3.0f);
	
	//关节输出力矩限幅,限制量为电机额定力矩
	LimitMax(&vmc_R->torque_set[0],12.0f);
	LimitMax(&vmc_R->torque_set[1],12.0f);
}

//底盘右腿输出控制环
void Balance_L_ctrl_loop(Chassis_t *Chassis_L,vmc_leg_t *vmcL,float *LQR_K,INS_t *ins)
{
	//VMC计算L0长度和角度phi0,d_phi0,该任务控制周期是3*0.001秒
	VMC_calc_1_left(vmcL,ins,SYS_delay*3.0f/1000.0f);
	
	//根据对应的线性化方程系数计算LQR_K
//	for(int i=0;i<12;i++)
//	{
//		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],Chassis_L->leg_right_set);
//	}
	
	//防劈叉补偿
	Chassis_L->leg_tp=PID_calc(&Chassis_L->tp_err_pid,Chassis_L->Theta_error,0.0f);
	//腿长补偿
	Chassis_L->leg_L0_tp=PID_calc(&Chassis_L->leg_L_pid,vmcL->L0,Chassis_L->leg_left_set);
	//偏航角补偿
	Chassis_L->wheel_turn_tp=BM_wheel_turn_pid[0]*(Chassis_L->yaw_set-ins->Yaw)-BM_wheel_turn_pid[2]*ins->Gyro[2];
	//roll轴补偿力矩
	Chassis_L->leg_roll_tp=PID_calc(&Chassis_L->leg_roll_pid,ins->Roll,Chassis_L->roll_set);
	
	//电机输出期望赋值
	Chassis_L->BM_motor[1][0].set_torque=LQR_K[0]*vmcL->theta   														//phi0     
																		  +LQR_K[1]*vmcL->d_theta															//dphi0
																		  +LQR_K[2]*(Chassis_L->x_filter-Chassis_L->x_set)		//x
																		  +LQR_K[3]*(Chassis_L->v_filter-Chassis_L->v_set)		//dx
																		  +LQR_K[4]*ins->Pitch																//pitch
																		  +LQR_K[5]*ins->Gyro[0];															//dpitch
	vmcL->Tp=LQR_K[6]*vmcL->theta  																//phi0     
					+LQR_K[7]*vmcL->d_theta																//dphi0
					+LQR_K[8]*(Chassis_L->x_set-Chassis_L->x_filter)			//x
					+LQR_K[9]*(Chassis_L->v_set-Chassis_L->v_filter)			//dx
					+LQR_K[10]*ins->Pitch																	//pitch
					+LQR_K[11]*ins->Gyro[0];															//dpitch
	//偏航角补偿
	Chassis_L->BM_motor[1][0].set_torque=-Chassis_L->BM_motor[1][0].set_torque-Chassis_L->wheel_turn_tp;
												
	//防劈叉力矩补偿											
	vmcL->Tp=vmcL->Tp-Chassis_L->leg_tp;	

	vmcL->F0=-((15.0f*0.50f*9.80f)/arm_cos_f32(vmcL->theta)+Chassis_L->leg_L0_tp+Chassis_L->leg_roll_tp);//前馈+pd
//	LimitMax(&Chassis_L->VMC_data.F0,172.3f);
	
	//计算期望的关节输出力矩
	VMC_calc_2(vmcL);
	
//	//轮毂电机力矩限幅,限制量为电机额定力矩
//	LimitMax(&Chassis_L->BM_motor[0].set_torque,3.0f);
	
	//关节输出力矩限幅,限制量为电机额定力矩
	LimitMax(&vmcL->torque_set[0],12.0f);
	LimitMax(&vmcL->torque_set[1],12.0f);
}

//返回数据更新
void chassisR_feedback_update(Chassis_t *Chassis_R,vmc_leg_t *vmcR,INS_t *ins)
{
  vmcR->phi1=Chassis_R->BM_motor[0][1].ecd_angle;
	vmcR->phi4=Chassis_R->BM_motor[0][2].ecd_angle;
		
	Chassis_R->Pitch_R=ins->Pitch;
	Chassis_R->d_Pitch_R=ins->Gyro[0];
}

void chassisL_feedback_update(Chassis_t *Chassis_L,vmc_leg_t *vmcL,INS_t *ins)
{
  vmcL->phi1=Chassis_L->BM_motor[1][2].ecd_angle;
	vmcL->phi4=Chassis_L->BM_motor[1][1].ecd_angle;
		
	Chassis_L->Pitch_L=0.0f-ins->Pitch;
	Chassis_L->d_Pitch_L=0.0f-ins->Gyro[0];
}



