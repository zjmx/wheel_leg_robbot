	#include "DM_Ctrl.h"

DM_Motor_t DM_Motor[3];//DM������ݷ��ؽṹ��

/**
************************************************************************
* @brief:      	enable_motor_mode: ���õ��ģʽ����
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   mode_id:  ģʽID��ָ��Ҫ������ģʽ
* @retval:     	void
* @details:    	ͨ��CAN�������ض�������������ض�ģʽ������
************************************************************************
**/
void enable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	for(int i=0;i<7;i++)
	{
		data[i]=0xFF;
	}
	data[7] = 0xFC;
	
	CAN_SendData_uint8_t(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	disable_motor_mode: ���õ��ģʽ����
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   mode_id:  ģʽID��ָ��Ҫ���õ�ģʽ
* @retval:     	void
* @details:    	ͨ��CAN�������ض�������ͽ����ض�ģʽ������
************************************************************************
**/
void disable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	for(int i=0;i<7;i++)
	{
		data[i]=0xFF;
	}
	data[7] = 0xFD;
	
	CAN_SendData_uint8_t(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	save_pos_zero: ����λ����㺯��
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   mode_id:  ģʽID��ָ��Ҫ����λ������ģʽ
* @retval:     	void
* @details:    	ͨ��CAN�������ض�������ͱ���λ����������
************************************************************************
**/
void save_pos_zero(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	for(int i=0;i<7;i++)
	{
		data[i]=0xFF;
	}
	data[7] = 0xFE;
	
	CAN_SendData_uint8_t(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	clear_err: ������������
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   mode_id:  ģʽID��ָ��Ҫ��������ģʽ
* @retval:     	void
* @details:    	ͨ��CAN�������ض�������������������
************************************************************************
**/
void clear_err(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id)
{
	uint8_t data[8];
	uint16_t id = motor_id + mode_id;
	
	for(int i=0;i<7;i++)
	{
		data[i]=0xFF;
	}
	data[7] = 0xFB;
	
	CAN_SendData_uint8_t(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	mit_ctrl: MITģʽ�µĵ�����ƺ���
* @param[in]:   hcan:			ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id:	���ID��ָ��Ŀ����
* @param[in]:   pos:			λ�ø���ֵ
* @param[in]:   vel:			�ٶȸ���ֵ
* @param[in]:   kp:				λ�ñ���ϵ��
* @param[in]:   kd:				λ��΢��ϵ��
* @param[in]:   torq:			ת�ظ���ֵ
* @retval:     	void
* @details:    	ͨ��CAN������������MITģʽ�µĿ���֡��
************************************************************************
**/
void mit_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq)
{
	uint8_t data_t[8];
	uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;
	uint16_t id = motor_id + MIT_MODE;

	pos_tmp = float_to_uint(pos,  P_MIN,  P_MAX,  16);
	vel_tmp = float_to_uint(vel,  V_MIN,  V_MAX,  12);
	kp_tmp  = float_to_uint(kp,   KP_MIN, KP_MAX, 12);
	kd_tmp  = float_to_uint(kd,   KD_MIN, KD_MAX, 12);
	tor_tmp = float_to_uint(torq, T_MIN,  T_MAX,  12);

	data_t[0] = (pos_tmp >> 8);
	data_t[1] = pos_tmp;
	data_t[2] = (vel_tmp >> 4);
	data_t[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);
	data_t[4] = kp_tmp;
	data_t[5] = (kd_tmp >> 4);
	data_t[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);
	data_t[7] = tor_tmp;
	
	CAN_SendData_uint8_t(hcan,id,data_t,8);
}
/**
************************************************************************
* @brief:      	pos_speed_ctrl: λ���ٶȿ��ƺ���
* @param[in]:   hcan:			ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id:	���ID��ָ��Ŀ����
* @param[in]:   vel:			�ٶȸ���ֵ
* @retval:     	void
* @details:    	ͨ��CAN������������λ���ٶȿ�������
************************************************************************
**/
void pos_speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel)
{
	uint16_t id;
	uint8_t *pbuf, *vbuf;
	uint8_t data[8];
	
	id = motor_id + POS_MODE;
	pbuf=(uint8_t*)&pos;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *pbuf;
	data[1] = *(pbuf+1);
	data[2] = *(pbuf+2);
	data[3] = *(pbuf+3);

	data[4] = *vbuf;
	data[5] = *(vbuf+1);
	data[6] = *(vbuf+2);
	data[7] = *(vbuf+3);
	
	CAN_SendData_uint8_t(hcan,id,data,8);
}
/**
************************************************************************
* @brief:      	speed_ctrl: �ٶȿ��ƺ���
* @param[in]:   hcan: 		ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @param[in]:   motor_id: ���ID��ָ��Ŀ����
* @param[in]:   vel: 			�ٶȸ���ֵ
* @retval:     	void
* @details:    	ͨ��CAN�������������ٶȿ�������
************************************************************************
**/
void speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float vel)
{
	uint16_t id;
	uint8_t *vbuf;
	uint8_t data[4];
	
	id = motor_id + SPEED_MODE;
	vbuf=(uint8_t*)&vel;
	
	data[0] = *vbuf;
	data[1] = *(vbuf+1);
	data[2] = *(vbuf+2);
	data[3] = *(vbuf+3);
	
	CAN_SendData_uint8_t(hcan,id,data,4);
}

//DM����������ݽ���
void CAN_rx_DM_Data(DM_Motor_t *motor_data,uint8_t *data)                                
{
	motor_data->para.id=(data[0])&0x0F;
	motor_data->para.state=(data[0])>>4;
	motor_data->para.p_int=(data[1]<<8)|data[2];
	motor_data->para.v_int=(data[3]<<4)|(data[4]>>4);
	motor_data->para.t_int=((data[4]&0xF)<<8)|data[5];
	motor_data->para.pos=uint_to_float(motor_data->para.p_int,P_MIN,P_MAX,16); // (-12.5,12.5)
	motor_data->para.vel=uint_to_float(motor_data->para.v_int,V_MIN,V_MAX,12); // (-30.0,30.0)
	motor_data->para.tor=uint_to_float(motor_data->para.t_int,T_MIN,T_MAX,12); // (-10.0,10.0)
	motor_data->para.Tmos=(float)(data[6]);
	motor_data->para.Tcoil=(float)(data[7]);                            
}


	