#include "can_receive.h"

CAN_FilterTypeDef CAN1_FilterConfig;//can1�ṹ��
CAN_RxFrameTypeDef hcanRxFrame;//can���սṹ��
CAN_TxFrameTypeDef hcanTxFrame;//can���ͽṹ
motor_measure_t motor_chassis[4];//rm������ݷ��ؽṹ��
motor_measure_t motor_gimbal[4];//rm������ݷ��ؽṹ��

//can�˲�����ʼ��
void CAN_Filter_Init(CAN_FilterTypeDef *sFilterConfig)
{
	sFilterConfig->FilterIdHigh = 0;						
	sFilterConfig->FilterIdLow = 0;							
	sFilterConfig->FilterMaskIdHigh = 0;					// ���˲�
	sFilterConfig->FilterMaskIdLow = 0;						// ���˲�
	sFilterConfig->FilterFIFOAssignment = CAN_FILTER_FIFO0;	// �˲���������FIFO0
	sFilterConfig->FilterBank = 0;							// �����˲���0   ��canΪ0��13��˫canΪ14��28
	sFilterConfig->FilterMode = CAN_FILTERMODE_IDMASK;		// ��ʶ������ģʽ
	sFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT;		// 32λ��
	sFilterConfig->FilterActivation = ENABLE;				// �����˲���
	sFilterConfig->SlaveStartFilterBank = 0;
	HAL_CAN_ConfigFilter(&hcan1, sFilterConfig);
	HAL_CAN_Start(&hcan1);

}

//can��ʼ��
void CAN_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	
	// ����CAN��ʶ���˲���
	CAN_Filter_Init(&sFilterConfig);
	
	// ʹ�ܽ����ж�
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

//can1���ͺ���
uint8_t CAN_SendData(CAN_HandleTypeDef *hcan,uint32_t stdId, int16_t *dat)
{
	uint32_t *txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = stdId;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	// �ȷ���8λ���ݣ��ٷ���8λ����
	txFrame.data[0] = (uint8_t)(dat[0] >> 8);
	txFrame.data[1] = (uint8_t)((dat[0] << 8) >> 8);
	txFrame.data[2] = (uint8_t)(dat[1] >> 8);
	txFrame.data[3] = (uint8_t)((dat[1] << 8) >> 8);
	txFrame.data[4] = (uint8_t)(dat[2] >> 8);
	txFrame.data[5] = (uint8_t)((dat[2] << 8) >> 8);
	txFrame.data[6] = (uint8_t)(dat[3] >> 8);
	txFrame.data[7] = (uint8_t)((dat[3] << 8) >> 8);		

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//can1��챵����������
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

//can1��������������
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
void CAN_rx_rm_Data(motor_measure_t *motor_data,uint8_t data[8])                                
{
	motor_data->last_ecd=motor_data->ecd;                               
	motor_data->ecd=(uint16_t)((data)[0]<<8|(data)[1]);       
	motor_data->speed_rpm=(uint16_t)((data)[2]<<8|(data)[3]);    
	motor_data->given_current=(uint16_t)((data)[4]<<8|(data)[5]);
	motor_data->temperate=(data)[6];                            
}

//can�����ж�
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//can�������ݴ洢
	HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&hcanRxFrame.header,hcanRxFrame.data);
	
	//can���ݽ���
	switch(hcanRxFrame.header.StdId)
	{
		case CAN_3508_M1_ID:
		case CAN_3508_M2_ID:
		case CAN_3508_M3_ID:
		case CAN_3508_M4_ID:
		{
			static uint8_t i = 0;
			//��ȡ���ID
			i=hcanRxFrame.header.StdId-CAN_3508_M1_ID;
			CAN_rx_rm_Data(&motor_chassis[i],hcanRxFrame.data);
			break;
		}
		case CAN_rudder1_ID:
		case CAN_rudder2_ID:
		case CAN_rudder3_ID:
		case CAN_rudder4_ID:
		{
			static uint8_t i = 0;
			//��ȡ���ID
			i=hcanRxFrame.header.StdId-CAN_rudder1_ID;
			CAN_rx_rm_Data(&motor_gimbal[i],hcanRxFrame.data);
			break;
		}
		default:
		{
				break;
		}
	}
}

	