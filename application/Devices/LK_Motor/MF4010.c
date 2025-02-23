#include "MF4010.h"
#include "can.h"
#include "main.h"

//CAN_RxFrameTypeDef hcanRxFrame;//can���սṹ��
//CAN_TxFrameTypeDef hcanTxFrame;//can���ͽṹ��
MF4010_measure_t MF4010_motor[2];//������ݷ��ؽṹ��
View_measure_t MF4010_View;//�Ӿ����ݷ��ؽṹ��
/**
 * @brief  ������ݶ�ȡ����,PID���������ٶȣ�����������Ȧ�Ƕȣ���Ȧ�Ƕȣ����״̬1/2/3
 * @param  hcan   CAN�ľ��
 * @param  ID     ����֡��ID      
 * @param  data   ����֡
 */
uint8_t MF4010_Read_data(CAN_HandleTypeDef* hcan,uint16_t ID, int16_t modle) 
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = ID;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = modle;
	txFrame.data[1] = 0x00;
	txFrame.data[2] = 0x00;
	txFrame.data[3] = 0x00;
	txFrame.data[4] = 0x00;
	txFrame.data[5] = 0x00;
	txFrame.data[6] = 0x00;
	txFrame.data[7] = 0x00;
	
	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, txFrame.data, &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

/**
 * @brief  д��PID������RAM/ROM���ϵ��д�����ʧЧ/�ϵ���Ȼ��Ч
 * @param  hcan   CAN�ľ��
 * @param  ID     ����֡��ID
 * @param  modle  ����ģʽ
 * @param  anglePI   �ǶȻ�PI����
 * @param  speedPI   �ٶȻ�PI����
 * @param  iqPI   	 ת�ػ�PI����
 */
uint8_t MF4010_Write_PID(CAN_HandleTypeDef* hcan,uint16_t ID,uint16_t modle,float anglePI[2],float speedPI[2],float iqPI[2])
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = ID;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = modle;
	txFrame.data[1] = 0x00;
	txFrame.data[2] = anglePI[0];
	txFrame.data[3] = anglePI[1];
	txFrame.data[4] = speedPI[0];
	txFrame.data[5] = speedPI[1];
	txFrame.data[6] = iqPI[0];
	txFrame.data[7] = iqPI[1];
	
	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, txFrame.data, &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

/**
 * @brief  д��Accel������RAM���ϵ��д�����ʧЧ
 * @param  hcan   CAN�ľ��
 * @param  ID     ����֡��ID
 * @param  modle  ����ģʽ
 * @param  Accel  ���ٶȸ���
 */
uint8_t MF4010_Write_Accel(CAN_HandleTypeDef* hcan,uint16_t ID,uint16_t modle,int32_t Accel)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = ID;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = modle;
	txFrame.data[1] = 0x00;
	txFrame.data[2] = 0x00;
	txFrame.data[3] = 0x00;
	txFrame.data[4] = *(uint8_t*)(&Accel);
	txFrame.data[5] = *((uint8_t*)(&Accel)+1);
	txFrame.data[6] = *((uint8_t*)(&Accel)+2);
	txFrame.data[7] = *((uint8_t*)(&Accel)+3);
	
	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, txFrame.data, &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

/**
 * @brief  д�������ֵ/��ǰλ�ò�����ROM����Ϊ����������ϵ��д�����ʧЧ
 * @param  hcan   CAN�ľ��
 * @param  ID     ����֡��ID
 * @param  modle  ����ģʽ
 * @param  encoderOffset  ���ٶȸ���
 */
uint8_t MF4010_Write_Encoder(CAN_HandleTypeDef* hcan,uint16_t ID,uint16_t modle,int32_t encoderOffset)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = ID;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = modle;
	txFrame.data[1] = 0x00;
	txFrame.data[2] = 0x00;
	txFrame.data[3] = 0x00;
	txFrame.data[4] = 0x00;
	txFrame.data[5] = 0x00;
	txFrame.data[6] = *((uint8_t*)(&encoderOffset));
	txFrame.data[7] = *((uint8_t*)(&encoderOffset)+1);
	
	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, txFrame.data, &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

/**
 * @brief  λ�û�ģʽ���¿���֡
 * @param  hcan   CAN�ľ��
 * @param  ID     ����֡��ID
 * @param  modle  λ�û�����ģʽ
 * @param  angleControl   λ�ø���
 */
uint8_t MF4010_Write_aggle(CAN_HandleTypeDef* hcan,uint16_t ID,uint16_t modle,int32_t angleControl)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = ID;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = modle;
	txFrame.data[1] = 0x00;
	txFrame.data[2] = 0x00;
	txFrame.data[3] = 0x00;
	txFrame.data[4] = *((uint8_t *)(&angleControl));
	txFrame.data[5] = *((uint8_t *)(&angleControl)+1);
	txFrame.data[6] = *((uint8_t *)(&angleControl)+2);
	txFrame.data[7] = *((uint8_t *)(&angleControl)+3);
	
	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, txFrame.data, &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

/**
 * @brief  �ٶȱջ�ģʽ���¿���֡
 * @param  hcan   CAN�ľ��
 * @param  ID     ����֡��ID
 * @param  modle  λ�û�����ģʽ
 * @param  speedControl  �ٶȸ���
 */
uint8_t MF4010_Write_speed_close(CAN_HandleTypeDef* hcan,uint16_t ID,uint16_t modle,int32_t speedControl)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = ID;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = modle;
	txFrame.data[1] = 0x00;
	txFrame.data[2] = 0x00;
	txFrame.data[3] = 0x00;
	txFrame.data[4] = *(uint8_t*)(&speedControl);
	txFrame.data[5] = *((uint8_t *)(&speedControl)+1);
	txFrame.data[6] = *((uint8_t *)(&speedControl)+2);
	txFrame.data[7] = *((uint8_t *)(&speedControl)+3);
	
	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, txFrame.data, &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

/**
 * @brief  λ�ñջ�ģʽ���¿���֡
 * @param  hcan   CAN�ľ��
 * @param  ID     ����֡��ID
 * @param  modle  λ�û�����ģʽ
 * @param  spinDirection  ת������
 * @param  maxSpeed   		���ת��
 * @param  angleControl   �Ƕȸ���
 */
uint8_t MF4010_Write_aggle_close(CAN_HandleTypeDef* hcan,uint16_t ID,uint16_t modle,uint8_t spinDirection,uint16_t maxSpeed,int32_t angleControl)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = ID;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = modle;
	txFrame.data[1] = 0x00;
	txFrame.data[2] = *(uint8_t*)(&maxSpeed);
	txFrame.data[3] = *((uint8_t*)(&maxSpeed)+1);
	txFrame.data[4] = *((uint8_t *)(&angleControl));
	txFrame.data[5] = *((uint8_t *)(&angleControl)+1);
	txFrame.data[6] = *((uint8_t *)(&angleControl)+2);
	txFrame.data[7] = *((uint8_t *)(&angleControl)+3);
	
	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, txFrame.data, &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

/**
 * @brief  ����ģʽ���¿���֡
 * @param  hcan   CAN�ľ��
 * @param  ID     ����֡��ID
 * @param  iqControl_1   ����ֵ1
 * @param  iqControl_2   ����ֵ2
 * @param  iqControl_3   ����ֵ3
 * @param  iqControl_4   ����ֵ4
 */
uint8_t MF4010_Write_multi_motor(CAN_HandleTypeDef* hcan,uint16_t ID,int16_t iqControl_1,int16_t iqControl_2,int16_t iqControl_3,int16_t iqControl_4)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = ID;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = *(uint8_t*)(&iqControl_1);
	txFrame.data[1] = *((uint8_t*)(&iqControl_1)+1);
	txFrame.data[2] = *(uint8_t*)(&iqControl_2);
	txFrame.data[3] = *((uint8_t*)(&iqControl_2)+1);
	txFrame.data[4] = *(uint8_t*)(&iqControl_3);
	txFrame.data[5] = *((uint8_t*)(&iqControl_3)+1);
	txFrame.data[6] = *(uint8_t*)(&iqControl_4);
	txFrame.data[7] = *((uint8_t*)(&iqControl_4)+1);
	
	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, txFrame.data, &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}


//can���ݷ��ؽ���
void MF4010_Rx_analysis(MF4010_measure_t *motor_rx_data,uint8_t data[8])
{
	switch(data[0])
	{
		case MF4010_Read_PID:
		case MF4010_Write_PID_RAM:
		case MF4010_Write_PID_ROM:
		{
			data[1]=data[1];
			motor_rx_data->anglePidKp=data[2];
			motor_rx_data->anglePidKi=data[3];
			motor_rx_data->speedPidKp=data[4];
			motor_rx_data->speedPidKi=data[5];
			motor_rx_data->iqPidKp=data[6];
			motor_rx_data->iqPidKi=data[7];
			break;
		}
		//���ٶ�
		case MF4010_Read_Accle:
		case MF4010_Write_Accle_RAM:
		{
			motor_rx_data->Accle|=data[4];
			motor_rx_data->Accle|=data[5]<<8;
			motor_rx_data->Accle|=(data[6]<<8)<<8;
			motor_rx_data->Accle|=((data[7]<<8)<<8)<<8;
			break;
		}
		//������ֵ
		case MF4010_Read_enc:
		{
			motor_rx_data->encoder=(uint16_t)((data[2]|(data[3]<<8)));
			motor_rx_data->encoderRaw=(uint16_t)((data[4]|(data[5]<<8)));
			motor_rx_data->encoderOffset=(uint16_t)((data[6]|(data[7]<<8)));
			break;
		}
		case MF4010_Write_enc_ROM:
		case MF4010_Write_nenc_ROM:
		{
			motor_rx_data->encoderOffset=(uint16_t)((data[6]|(data[7]<<8)));
		}
		//ת�رջ�*�ٶȱջ�*λ�ñջ�*��������
		case MF4010_torque_close://(and��������)
		case MF4010_speed_modle:
		case MF4010_postion_modle1:
		case MF4010_postion_modle2:
		case MF4010_postion_modle3:
		case MF4010_postion_modle4:
		case MF4010_postion_modle5:
		case MF4010_postion_modle6:
		{
			motor_rx_data->temperature=data[1];
			motor_rx_data->iq=(data[2]|(data[3]<<8));
			motor_rx_data->speed=(data[4]|(data[5]<<8));
			motor_rx_data->encoder=(data[6]|(data[7]<<8));
		}
		default:break;
	}
}



