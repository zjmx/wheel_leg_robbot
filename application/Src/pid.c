#include "pid.h"

//�޷�����
void LimitMax(fp32 *input,fp32 max)
{
	if(*input>max)
	{
		*input=max;
	}
	else if(*input<-max)
	{
		*input=-max;
	}
}

//����6020,1:1���ٱȵĹ��㴦��
fp32 rad_format(fp32 rad)
{
	if(rad>180)
	{
		while(rad>180)
		{
			rad=rad-2*180;
		}
	}
	else if(rad<-180)
	{
		while(rad<-180)
		{
			rad=rad+2*180;
		}
	}
	return rad;
}

//pid��ʼ��
void PID_init(pid_type_def *pid,const fp32 PID[3],fp32 max_iout,fp32 max_out)
{
	pid->kp=PID[0];
	pid->ki=PID[1];
	pid->kd=PID[2];
	pid->max_iout=max_iout;
	pid->max_out=max_out;
	pid->dbuf[0] = pid->dbuf[1] = pid->dbuf[2] = 0.0f;
  pid->error[0] = pid->error[1] = pid->error[2] = pid->pout = pid->iout = pid->dout = pid->out = 0.0f;
}


//pid�ٶȻ�
fp32 PID_calc(pid_type_def *pid,fp32 fdb,fp32 set)//��ǰֵ��Ŀ��ֵ
{
	pid->fdb=fdb;//��ǰֵ
	pid->set=set;//Ŀ��ֵ
	pid->error[1]=pid->error[0];
	pid->error[0]=(set-fdb);//�������
	
	pid->pout=pid->kp*pid->error[0];
	pid->iout+=pid->ki*pid->error[0];
	
	LimitMax(&pid->iout,pid->max_iout);//pid�����޷�
	
	pid->dout=pid->kd*(pid->error[0]-pid->error[1]);
	
	pid->out=pid->pout+pid->iout+pid->dout;
	
	LimitMax(&pid->out,pid->max_out);//pid΢���޷�
	
	return pid->out;
}

//pid�ǶȻ���  
fp32 PID_calc_angle(pid_type_def *pid,fp32 fdb,fp32 set)//��ǰֵ��Ŀ��ֵ
{
	pid->fdb=fdb;//��ǰֵ
	pid->set=set;//Ŀ��ֵ
	pid->error[1]=pid->error[0];
	pid->error[0]=rad_format(set-fdb);//�������
	
	pid->pout=pid->kp*pid->error[0];
	pid->iout+=pid->ki*pid->error[0];
	
	LimitMax(&pid->iout,pid->max_iout);//pid�����޷�
	
	pid->dout=pid->kd*(pid->error[0]-pid->error[1]);
	
	pid->out=pid->pout+pid->iout+pid->dout;
	
	LimitMax(&pid->out,pid->max_out);//pid΢���޷�
	
	return pid->out;
}

//PI
//ƽ�⳵ѧ���Ŀ��Ʒ���
//����Ƕ�ֵ�ͽ��ٶ�ֵ
fp32 PI_postion(pid_type_def *pid,fp32 pitch,fp32 gyro_y)
{
	pid->fdb=pitch;
	pid->set=gimbal_balance;//��е��ֵ
	pid->error[1]=pid->error[0];
	pid->error[0]=pid->set-pid->fdb;
	
	pid->pout=pid->kp*pid->error[0];
	pid->dout=pid->kd*gyro_y;

	//Ŀ��ֵ��ȥ��е��ֵ����һ��Ϊ0��
	pid->out=pid->pout+pid->dout;

	return pid->out;
}


