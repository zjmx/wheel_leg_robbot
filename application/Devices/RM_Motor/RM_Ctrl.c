#include "RM_Ctrl.h"

motor_measure_t motor_chassis[7];//rm电机数据返回结构体

/**
************************************************************************
* @brief:      	CAN_chassis_send: CAN3508_2006电机驱动函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor1~4: 向指定ID电机发送电流值
* @retval:     	uint8_t
* @details:    	通过CAN总线向特定电机发送启用特定模式的命令
************************************************************************
**/
uint8_t CAN_chassis_send(CAN_HandleTypeDef *hcan,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = CAN_all_chassis_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	// 先发高8位数据，再发低8位数据
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
* @brief:      	CAN_gimbal_send: CAN6020电机驱动函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   rudder1~4: 向指定ID电机发送电流值
* @retval:     	uint8_t
* @details:    	通过CAN总线向特定电机发送启用特定模式的命令
************************************************************************
**/
uint8_t CAN_gimbal_send(CAN_HandleTypeDef *hcan,int16_t rudder1, int16_t rudder2, int16_t rudder3, int16_t rudder4)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = CAN_all_gimbal_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	// 先发高8位数据，再发低8位数据
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

//rm电机返回数据解算
void CAN_rx_rm_Data(motor_measure_t *motor_data,uint8_t *data)                                
{
	//差ID和地址返回及赋值
	motor_data->last_ecd=motor_data->ecd;                               
	motor_data->ecd=(uint16_t)((data)[0]<<8|(data)[1]);       
	motor_data->speed_rpm=(uint16_t)((data)[2]<<8|(data)[3]);    
	motor_data->given_current=(uint16_t)((data)[4]<<8|(data)[5]);
	motor_data->temperate=(data)[6];                            
}

	