#ifndef __PID_H
#define __PID_H

#include "struct_typedef.h"
#include "main.h"
#include "function.h"

#define gimbal_balance 0//��̨��еƽ���

//����PID�ṹ��
typedef struct
{
	//PID���Ʋ���
	fp32 kp;
	fp32 ki;
	fp32 kd;
	
	//���
	fp32 dbuf[3];  //΢���� 0���� 1��һ�� 2���ϴ�
  fp32 error[3]; //����� 0���� 1��һ�� 2���ϴ�
	
	//PID�޷�
	fp32 max_iout;
	fp32 max_out;
	
	//Ŀ�����뵱ǰ��
	fp32 set;
	fp32 fdb;
	
	//PID���
	fp32 out;
	fp32 pout;
	fp32 iout;
	fp32 dout;

	//˫��PID����
	float angle_ref;
	float angle_fdb;
	float speed_ref;
	float speed_fdb;

	//ֱ��������
	fp32 Angle_pitch;
	fp32 gyro_y;

}pid_type_def;

void LimitMax(fp32 *input,fp32 max);
fp32 rad_format(fp32 rad);
void PID_init(pid_type_def *pid,const fp32 PID[3],fp32 max_iout,fp32 max_out);
fp32 PID_calc(pid_type_def *pid,fp32 ref,fp32 set);
fp32 PID_calc_rad_format(pid_type_def *pid,fp32 fdb,fp32 set);//��ǰֵ��Ŀ��ֵ
fp32 PID_calc_application(pid_type_def *pid,fp32 ref,fp32 set,fp32 speed);//��ǰֵ��Ŀ��ֵ


fp32 PI_postion(pid_type_def *pid,fp32 pitch,fp32 gyro_y);//PI�ǶȻ�

#endif
