#include "can_receive.h"

CAN_FilterTypeDef CAN1_FilterConfig;//can1�ṹ��
CAN_RxFrameTypeDef hcan1RxFrame;//can1���սṹ��
CAN_TxFrameTypeDef hcan1TxFrame;//can1���ͽṹ
CAN_FilterTypeDef CAN2_FilterConfig;//can2�ṹ��
CAN_RxFrameTypeDef hcan2RxFrame;//can2���սṹ��
CAN_TxFrameTypeDef hcan2TxFrame;//can2���ͽṹ

/**
************************************************************************
* @brief:      	CAN_Filter_Init: CAN�˲�����ʼ������
* @param[in]:   sFilterConfig: 	 ָ��CAN_HandleTypeDef�ṹ��ָ�룬����ָ��CAN����
* @retval:     	void
* @details:    	CAN�˲�����ʼ��
************************************************************************
**/
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
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);//FIFO0
	HAL_CAN_Start(&hcan1);
	
//	sFilterConfig->FilterIdHigh = 0;						
//	sFilterConfig->FilterIdLow = 0;							
//	sFilterConfig->FilterMaskIdHigh = 0;					// ���˲�
//	sFilterConfig->FilterMaskIdLow = 0;						// ���˲�
	sFilterConfig->FilterFIFOAssignment = CAN_FILTER_FIFO1;	// �˲���������FIFO0
	sFilterConfig->FilterBank = 14;							// �����˲���0   ��canΪ0��13��˫canΪ14��28
	sFilterConfig->FilterMode = CAN_FILTERMODE_IDMASK;		// ��ʶ������ģʽ
	sFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT;		// 32λ��
	sFilterConfig->FilterActivation = ENABLE;				// �����˲���
	sFilterConfig->SlaveStartFilterBank = 14;
	
	HAL_CAN_ConfigFilter(&hcan2, sFilterConfig);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);//FIFO1
	HAL_CAN_Start(&hcan2);
}

/**
************************************************************************
* @brief:      	CAN_Init: CAN��ʼ��
* @param[in]:   void
* @retval:     	void
* @details:    	CAN��ʼ��
************************************************************************
**/
void CAN_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	
	// ����CAN��ʶ���˲���
	CAN_Filter_Init(&sFilterConfig);
	
	// ʹ�ܽ����ж�
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	// ʹ�ܽ����ж�
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
************************************************************************
* @brief:      	CAN_SendData_int16_t: CAN���ͺ���,����int16_t����
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   stdId:    ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   data:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @retval:     	uint8_t
* @details:    	ͨ��CAN����int16_t����
************************************************************************
**/
uint8_t CAN_SendData_int16_t(CAN_HandleTypeDef *hcan,uint32_t stdId,int16_t *data)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = stdId;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	// �ȷ���8λ���ݣ��ٷ���8λ����
	txFrame.data[0] = (uint8_t)(data[0] >> 8);
	txFrame.data[1] = (uint8_t)((data[0] << 8) >> 8);
	txFrame.data[2] = (uint8_t)(data[1] >> 8);
	txFrame.data[3] = (uint8_t)((data[1] << 8) >> 8);
	txFrame.data[4] = (uint8_t)(data[2] >> 8);
	txFrame.data[5] = (uint8_t)((data[2] << 8) >> 8);
	txFrame.data[6] = (uint8_t)(data[3] >> 8);
	txFrame.data[7] = (uint8_t)((data[3] << 8) >> 8);		

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

/**
************************************************************************
* @brief:      	CAN_SendData_uint8_t: CAN���ͺ���,����uint8_t����
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @param[in]:   stdId:    CANid��ַ
* @param[in]:   data:     ��������
* @param[in]:   len:      ���͵����ݳ���
* @retval:     	uint8_t
* @details:    	ͨ��CAN����uint8_t����
************************************************************************
**/
uint8_t CAN_SendData_uint8_t(CAN_HandleTypeDef *hcan,uint32_t stdId,uint8_t *data,uint32_t len)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = stdId;//���͵�ַ
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	
	//�жϲ���ֵ���ݳ���
	if(len<=8)	
	{
	  txFrame.header.DLC=8;     // ���ͳ��ȣ�8byte
	}
	
	// �ȷ���8λ���ݣ��ٷ���8λ����
	for(int i=0;i<8;i++)
	{
		txFrame.data[i] = data[i];
	}		

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

/**
************************************************************************
* @brief:      	HAL_CAN_RxFifo0MsgPendingCallback: CAN1�����жϺ���
* @brief:      	HAL_CAN_RxFifo1MsgPendingCallback: CAN2�����жϺ���
* @param[in]:   hcan:     ָ��CAN_HandleTypeDef�ṹ��ָ��
* @retval:     	void
* @details:    	ͨ��CAN���߽��շ�������
************************************************************************
**/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance==CAN1)
  {
		//can�������ݴ洢
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&hcan1RxFrame.header,hcan1RxFrame.data);
		switch(hcan1RxFrame.header.StdId)
		{
			//BM����챵��
			case BM_wheel_right_ID:{CAN_rx_BM_wheel_Data(&Chassis.BM_motor[0][0],hcan1RxFrame.data);break;}
			//BM�ؽڵ��
			case BM_joint_right1_ID:{CAN_rx_BM_joint_Data(&Chassis.BM_motor[0][1],hcan1RxFrame.data);break;}
			case BM_joint_right2_ID:{CAN_rx_BM_joint_Data(&Chassis.BM_motor[0][2],hcan1RxFrame.data);break;}
			default:break;		
		}
	}
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance==CAN2)
  {
		//can�������ݴ洢
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&hcan2RxFrame.header,hcan2RxFrame.data);
		switch(hcan2RxFrame.header.StdId)
		{
			//BM����챵��
			case BM_wheel_left_ID:{CAN_rx_BM_wheel_Data(&Chassis.BM_motor[1][0],hcan2RxFrame.data);break;}
			//BM�ؽڵ��
			case BM_joint_left1_ID:{CAN_rx_BM_joint_Data(&Chassis.BM_motor[1][1],hcan2RxFrame.data);break;}
			case BM_joint_left2_ID:{CAN_rx_BM_joint_Data(&Chassis.BM_motor[1][2],hcan2RxFrame.data);break;}
			default:break;		
		}
	}
}
	