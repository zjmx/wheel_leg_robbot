#include "MF4010.h"
#include "can.h"
#include "main.h"

//CAN_RxFrameTypeDef hcanRxFrame;//can接收结构体
//CAN_TxFrameTypeDef hcanTxFrame;//can发送结构体
MF4010_measure_t MF4010_motor[2];//电机数据返回结构体
View_measure_t MF4010_View;//视觉数据返回结构体
/**
 * @brief  电机数据读取命令,PID参数，加速度，编码器，多圈角度，单圈角度，电机状态1/2/3
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID      
 * @param  data   命令帧
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
 * @brief  写入PID参数到RAM/ROM，断电后写入参数失效/断电仍然有效
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  modle  控制模式
 * @param  anglePI   角度环PI给定
 * @param  speedPI   速度环PI给定
 * @param  iqPI   	 转矩环PI给定
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
 * @brief  写入Accel参数到RAM，断电后写入参数失效
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  modle  控制模式
 * @param  Accel  加速度给定
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
 * @brief  写入编码器值/当前位置参数到ROM，作为电机零点命令，断电后写入参数失效
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  modle  控制模式
 * @param  encoderOffset  加速度给定
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
 * @brief  位置环模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  modle  位置环控制模式
 * @param  angleControl   位置给定
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
 * @brief  速度闭环模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  modle  位置环控制模式
 * @param  speedControl  速度给定
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
 * @brief  位置闭环模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  modle  位置环控制模式
 * @param  spinDirection  转动方向
 * @param  maxSpeed   		最大转速
 * @param  angleControl   角度给定
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
 * @brief  多电机模式控下控制帧
 * @param  hcan   CAN的句柄
 * @param  ID     数据帧的ID
 * @param  iqControl_1   电流值1
 * @param  iqControl_2   电流值2
 * @param  iqControl_3   电流值3
 * @param  iqControl_4   电流值4
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


//can数据返回解析
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
		//加速度
		case MF4010_Read_Accle:
		case MF4010_Write_Accle_RAM:
		{
			motor_rx_data->Accle|=data[4];
			motor_rx_data->Accle|=data[5]<<8;
			motor_rx_data->Accle|=(data[6]<<8)<<8;
			motor_rx_data->Accle|=((data[7]<<8)<<8)<<8;
			break;
		}
		//编码器值
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
		//转矩闭环*速度闭环*位置闭环*多电机控制
		case MF4010_torque_close://(and多电机控制)
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



