#include "BM_Ctrl.h"

/**
************************************************************************
* @brief:      	CAN_chassis_send: CAN3508_2006�����������
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor1~4: ��ָ��ID������͵���ֵ
* @retval:     	uint8_t
* @details:    	ͨ��CAN�������ض�������������ض�ģʽ������
************************************************************************
**/

//��ѹ��Vq�����������ٶȸ���
//�������ָ���16383��16383��
//���������ָ���16383��16383��Iq ����ֵ=����ֵ*55/32767����λ�� A
//����ٶȻ�ָ���5000��5000����500RPM��500RPM
uint8_t BM_motor_ctrl(CAN_HandleTypeDef *hcan,int16_t ctrl_ID_range,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = ctrl_ID_range;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	// �ȷ���8λ���ݣ��ٷ���8λ����
	txFrame.data[0] = (uint8_t)(motor1 >> 8);
	txFrame.data[1] = (uint8_t)((motor1 << 8) >> 8);
	txFrame.data[2] = (uint8_t)(motor2 >> 8);
	txFrame.data[3] = (uint8_t)((motor2 << 8) >> 8);
	txFrame.data[4] = (uint8_t)(motor3 >> 8);
	txFrame.data[5] = (uint8_t)((motor3 << 8) >> 8);
	txFrame.data[6] = (uint8_t)(motor4 >> 8);
	txFrame.data[7] = (uint8_t)((motor4 << 8) >> 8); 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//���У׼
uint8_t BM_motor_adjust(CAN_HandleTypeDef *hcan)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_motor_adjust_ID;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	for(int i=0;i<8;i++)
	{
		txFrame.data[i] = 0x000;
	} 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//���ÿ���ģʽ
uint8_t BM_ctrl_mode_set(CAN_HandleTypeDef *hcan,uint8_t ctrl_mode[8])
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_mode_set;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	for(int i=0;i<8;i++)
	{
		txFrame.data[i]=ctrl_mode[i];
	}	

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//���÷���ģʽ (�� 7 λΪ�����ϱ���ʽ�µ��ϱ�Ƶ�ʣ���λ ms ����ΧΪ1~127ms)
uint8_t BM_motor_feedback_mode_set(CAN_HandleTypeDef *hcan,uint8_t motor_feedback_mode[8])
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_feedback_mode_set_ID;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	for(int i=0;i<8;i++)
	{
		txFrame.data[i] = motor_feedback_mode[i];	
	}

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//�������ݲ�ѯѡ��
uint8_t BM_motor_feedback_data_choose(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t data_choose[3],uint8_t user_data)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_feedback_data_choose_ID;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//Ŀ����ID
	txFrame.data[1] = (uint8_t)(data_choose[0]);//���ز���ѡ��
	txFrame.data[2] = (uint8_t)(data_choose[1]);
	txFrame.data[3] = (uint8_t)(data_choose[2]);
	txFrame.data[4] = 0x0AA;
	txFrame.data[5] = 0x000; 
	txFrame.data[6] = 0x000; 
	txFrame.data[7] = 0x000; 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//���ID����
uint8_t BM_ID_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_ID_set_ID;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//Ŀ����ID
	for(int i=1;i<8;i++)
	{
		txFrame.data[i] = 0x000;
	}	

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//��� CAN �ն˵���ѡͨ����(Ĭ�϶Ͽ�)
uint8_t BM_can_terminal_R_ctrl_set(CAN_HandleTypeDef *hcan,uint8_t on_off[8])
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_can_terminal_R_ctrl_ID;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	for(int i=0;i<8;i++)
	{
		txFrame.data[i]=on_off[i];
	}	

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//����̼��汾��ѯ
uint8_t BM_version_query_set(CAN_HandleTypeDef *hcan)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_version_query_ID;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	for(int i=0;i<8;i++)
	{
		txFrame.data[i] = 0x000;
	} 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//ͨѶ��ʱ��д��������
uint8_t BM_commun_timeout_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t timeout_sign,uint8_t R_W_sign,int16_t timeout_data)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_commun_timeout_set_ID;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//Ŀ����ID
	txFrame.data[1] = (uint8_t)(timeout_sign);//��ʱ��־
	txFrame.data[2] = (uint8_t)(R_W_sign);//��д��־
	txFrame.data[3] = (uint8_t)(timeout_data>>8);
	txFrame.data[4] = (uint8_t)((timeout_data<<8)>>8);
	txFrame.data[5] = 0x000; 
	txFrame.data[6] = 0x000; 
	txFrame.data[7] = 0x000; 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//��� PI ��������
uint8_t BM_motor_PI_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t adjust_mode,int16_t MAX_duty_or_band_width)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_motor_PI_adjust_ID;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//Ŀ����ID
	txFrame.data[1] = (uint8_t)(adjust_mode);//����Ŀ��
	txFrame.data[2] = (uint8_t)(MAX_duty_or_band_width>>8);
	txFrame.data[3] = (uint8_t)((MAX_duty_or_band_width<<8)>>8);
	txFrame.data[4] = 0x000;
	txFrame.data[5] = 0x000; 
	txFrame.data[6] = 0x000; 
	txFrame.data[7] = 0x000; 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//��������
uint8_t BM_motor_data_save_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_motor_data_save_ID;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//Ŀ����ID
	txFrame.data[1] = 0x0FE;
	for(int i=2;i<8;i++)
	{
		txFrame.data[i]=0x000;
	} 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//���¡�������������ѡͨ����
uint8_t BM_over_protect_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t switch_sign,uint8_t R_W_sign,uint8_t switch_data)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_end_byte_set_ID;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//Ŀ����ID
	txFrame.data[1] = (uint8_t)(switch_sign);//����ֽڱ�־
	txFrame.data[2] = (uint8_t)(R_W_sign);//��д��־
	txFrame.data[3] = (uint8_t)(switch_data);//����ѡ��
	txFrame.data[4] = 0x000;
	txFrame.data[5] = 0x000; 
	txFrame.data[6] = 0x000; 
	txFrame.data[7] = 0x000; 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//CAN ��������
uint8_t BM_can_commun_speed_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t speed_set_sign,uint8_t R_W_sign,uint8_t commun_speed)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_can_commun_speed_set_ID;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//Ŀ����ID
	txFrame.data[1] = (uint8_t)(speed_set_sign);//�������ñ�־
	txFrame.data[2] = (uint8_t)(R_W_sign);//��д��־
	txFrame.data[3] = (uint8_t)(commun_speed);//ͨѶ����ѡ��
	txFrame.data[4] = 0x000;
	txFrame.data[5] = 0x000; 
	txFrame.data[6] = 0x000; 
	txFrame.data[7] = 0x000; 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//�ؽڷ�����ʽ����
uint8_t BM_feedback_mode_joint_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t feedback_mode,uint8_t feedback_time,uint8_t send_data[4])
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_feedback_mode_joint_ID;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//Ŀ����ID
	txFrame.data[1] = (uint8_t)(feedback_mode);//����ģʽ
	txFrame.data[2] = (uint8_t)(feedback_time);//����ʱ��
	txFrame.data[3] = (uint8_t)(send_data[0]);//�ϱ�����[4]
	txFrame.data[4] = (uint8_t)(send_data[1]);
	txFrame.data[5] = (uint8_t)(send_data[2]); 
	txFrame.data[6] = (uint8_t)(send_data[3]); 
	txFrame.data[7] = 0x000; 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//�ؽڵ���������ݲ�ѯ
uint8_t BM_data_passive_mode_set(CAN_HandleTypeDef *hcan,uint8_t motor_data[4])
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_data_passive_mode_ID;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_data[0]);//��ѯ����
	txFrame.data[1] = (uint8_t)(motor_data[1]);
	txFrame.data[2] = (uint8_t)(motor_data[2]);
	txFrame.data[3] = (uint8_t)(motor_data[3]);
	txFrame.data[4] = 0x000;
	txFrame.data[5] = 0x000;
	txFrame.data[6] = 0x000; 
	txFrame.data[7] = 0x000; 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//�ؽڵ����������
uint8_t BM_data_write_mode_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t data_sign,int32_t passive_data)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_data_write_mode_ID;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//Ŀ����ID
	txFrame.data[1] = (uint8_t)(data_sign);//�������
	txFrame.data[2] = (uint8_t)(passive_data>>24);//��ѯ����
	txFrame.data[3] = (uint8_t)(passive_data>>16);
	txFrame.data[4] = (uint8_t)(passive_data>>8);
	txFrame.data[5] = (uint8_t)(passive_data); 
	txFrame.data[6] = 0x000; 
	txFrame.data[7] = 0x000; 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//�ؽڵ��������ȡ
uint8_t BM_data_read_mode_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t data_sign)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_data_write_mode_ID;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//Ŀ����ID
	txFrame.data[1] = (uint8_t)(data_sign);//�������
	txFrame.data[2] = 0x000;
	txFrame.data[3] = 0x000;
	txFrame.data[4] = 0x000;
	txFrame.data[5] = 0x000; 
	txFrame.data[6] = 0x000; 
	txFrame.data[7] = 0x000; 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//���ÿ���ģʽ
uint8_t BM_ctrl_mode_joint_set(CAN_HandleTypeDef *hcan,uint8_t ctrl_mode[8])
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_ctrl_mode_joint_ID;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	for(int i=0;i<8;i++)
	{
		txFrame.data[i]=ctrl_mode[i];
	}	

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//��������
uint8_t BM_data_save_joint_set(CAN_HandleTypeDef *hcan,uint8_t reserve_mode,uint8_t abs_zero_point_set_switch)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_save_mode_joint_ID;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(reserve_mode);//ѡ���Ƿ����flash����
	txFrame.data[1] = (uint8_t)(abs_zero_point_set_switch);//����ǰλ�ñ���Ϊ�������
	for(int i=2;i<8;i++)
	{
		txFrame.data[i]=0x000;
	}	

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//�����λ
uint8_t BM_software_reset_set(CAN_HandleTypeDef *hcan,uint8_t software_reset_switch)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_software_reset_ID;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(software_reset_switch);//ѡ�񱣴�ģʽ
	for(int i=1;i<8;i++)
	{
		txFrame.data[i]=0x000;
	}	

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//BM����������ݽ���
void CAN_rx_BM_wheel_Data(BM_motor_measure_t *motor_data,uint8_t *data)                                
{
	motor_data->last_ecd=motor_data->ecd;
	motor_data->last_ecd_angle=(float)(motor_data->last_ecd*ecd_ratio_wheel);//���ǰ�Ƕ�(0~360��)
	
	motor_data->speed=(int16_t)((data)[0]<<8|(data)[1]);    
	motor_data->speed_rpm=(float)(motor_data->speed*speed_ratio_wheel);//���ת��(rpm/s)
	motor_data->speed_rad=(float)(motor_data->speed_rpm*rpm_to_rad);//������ٶ�(rad/s)
	
	motor_data->current=(int16_t)((data)[2]<<8|(data)[3]);
	motor_data->current_A=(float)(motor_data->current*current_ratio_wheel);//ת�ص���(A)
	motor_data->torque=(float)((motor_data->current_A)*torque_constant_wheel);//Ť��(Nm)
	
	motor_data->ecd=(uint16_t)((data)[4]<<8|(data)[5]); 
	motor_data->ecd_angle=(float)msp(motor_data->ecd,0,32768,-pi,pi);//����Ƕ�(0~2pi)
	
	motor_data->error=(data)[6];  
	motor_data->mode_now=(data)[7];
}

void CAN_rx_BM_joint_Data(BM_motor_measure_t *motor_data,uint8_t *data)                                
{
	motor_data->last_ecd=motor_data->ecd;
	motor_data->last_ecd_angle=(float)(motor_data->last_ecd*ecd_ratio_joint);//���ǰ�Ƕ�(0~360��)
	
	motor_data->speed=(int16_t)((data)[0]<<8|(data)[1]);    
	motor_data->speed_rpm=(float)(motor_data->speed*speed_ratio_joint);//���ת��(rpm/s)
	motor_data->speed_rad=(float)(motor_data->speed_rpm*rpm_to_rad);//������ٶ�(rad/s)
	
	motor_data->current=(int16_t)((data)[2]<<8|(data)[3]);
	motor_data->current_A=(float)(motor_data->current*current_ratio_joint);//ת�ص���(A)
	motor_data->torque=(float)((motor_data->current_A/1.414f)*torque_constant_joint);//Ť��(Nm)
	
	motor_data->ecd=(uint16_t)((data)[4]<<8|(data)[5]); 
	motor_data->ecd_angle=(float)(float)msp(motor_data->ecd,0,32768,-pi,pi);//����Ƕ�(0~2pi)
	
	motor_data->voltage=(int16_t)((data)[6]<<8|(data)[7]);  
	motor_data->voltage_V=(float)(motor_data->voltage*voltage_ratio_joint);//ĸ�ߵ�ѹ(V)
}

	