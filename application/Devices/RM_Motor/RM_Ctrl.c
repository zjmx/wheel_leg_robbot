#include "RM_Ctrl.h"

motor_measure_t motor_chassis[7];//rm������ݷ��ؽṹ��

/**
************************************************************************
* @brief:      	CAN_chassis_send: CAN3508_2006�����������
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   motor1~4: ��ָ��ID������͵���ֵ
* @retval:     	uint8_t
* @details:    	ͨ��CAN�������ض�������������ض�ģʽ������
************************************************************************
**/
uint8_t CAN_chassis_send(CAN_HandleTypeDef *hcan,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = CAN_all_chassis_ID;//���͵�ַ
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

/**
************************************************************************
* @brief:      	CAN_gimbal_send: CAN6020�����������
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   rudder1~4: ��ָ��ID������͵���ֵ
* @retval:     	uint8_t
* @details:    	ͨ��CAN�������ض�������������ض�ģʽ������
************************************************************************
**/
uint8_t CAN_gimbal_send(CAN_HandleTypeDef *hcan,int16_t rudder1, int16_t rudder2, int16_t rudder3, int16_t rudder4)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = CAN_all_gimbal_ID;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	// �ȷ���8λ���ݣ��ٷ���8λ����
	txFrame.data[0] = (uint8_t)(rudder1 >> 8);
	txFrame.data[1] = (uint8_t)((rudder1 << 8) >> 8);
	txFrame.data[2] = (uint8_t)(rudder2 >> 8);
	txFrame.data[3] = (uint8_t)((rudder2 << 8) >> 8);
	txFrame.data[4] = (uint8_t)(rudder3 >> 8);
	txFrame.data[5] = (uint8_t)((rudder3 << 8) >> 8);
	txFrame.data[6] = (uint8_t)(rudder4 >> 8);
	txFrame.data[7] = (uint8_t)((rudder4 << 8) >> 8);		

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//rm����������ݽ���
void CAN_rx_rm_Data(motor_measure_t *motor_data,uint8_t *data)                                
{
	//��ID�͵�ַ���ؼ���ֵ
	motor_data->last_ecd=motor_data->ecd;                               
	motor_data->ecd=(uint16_t)((data)[0]<<8|(data)[1]);       
	motor_data->speed_rpm=(uint16_t)((data)[2]<<8|(data)[3]);    
	motor_data->given_current=(uint16_t)((data)[4]<<8|(data)[5]);
	motor_data->temperate=(data)[6];                            
}

	