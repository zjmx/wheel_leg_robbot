#include "can_receive.h"

CAN_FilterTypeDef CAN1_FilterConfig;//can1结构体
CAN_RxFrameTypeDef hcan1RxFrame;//can1接收结构体
CAN_TxFrameTypeDef hcan1TxFrame;//can1发送结构
CAN_FilterTypeDef CAN2_FilterConfig;//can2结构体
CAN_RxFrameTypeDef hcan2RxFrame;//can2接收结构体
CAN_TxFrameTypeDef hcan2TxFrame;//can2发送结构

/**
************************************************************************
* @brief:      	CAN_Filter_Init: CAN滤波器初始化函数
* @param[in]:   sFilterConfig: 	 指向CAN_HandleTypeDef结构的指针，用于指定CAN总线
* @retval:     	void
* @details:    	CAN滤波器初始化
************************************************************************
**/
void CAN_Filter_Init(CAN_FilterTypeDef *sFilterConfig)
{
	sFilterConfig->FilterIdHigh = 0;						
	sFilterConfig->FilterIdLow = 0;							
	sFilterConfig->FilterMaskIdHigh = 0;					// 不滤波
	sFilterConfig->FilterMaskIdLow = 0;						// 不滤波
	
	sFilterConfig->FilterFIFOAssignment = CAN_FILTER_FIFO0;	// 滤波器关联到FIFO0
	sFilterConfig->FilterBank = 0;							// 设置滤波器0   单can为0到13，双can为14到28
	sFilterConfig->FilterMode = CAN_FILTERMODE_IDMASK;		// 标识符屏蔽模式
	sFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT;		// 32位宽
	sFilterConfig->FilterActivation = ENABLE;				// 激活滤波器
	sFilterConfig->SlaveStartFilterBank = 0;
	
	HAL_CAN_ConfigFilter(&hcan1, sFilterConfig);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);//FIFO0
	HAL_CAN_Start(&hcan1);
	
//	sFilterConfig->FilterIdHigh = 0;						
//	sFilterConfig->FilterIdLow = 0;							
//	sFilterConfig->FilterMaskIdHigh = 0;					// 不滤波
//	sFilterConfig->FilterMaskIdLow = 0;						// 不滤波
	sFilterConfig->FilterFIFOAssignment = CAN_FILTER_FIFO1;	// 滤波器关联到FIFO0
	sFilterConfig->FilterBank = 14;							// 设置滤波器0   单can为0到13，双can为14到28
	sFilterConfig->FilterMode = CAN_FILTERMODE_IDMASK;		// 标识符屏蔽模式
	sFilterConfig->FilterScale = CAN_FILTERSCALE_32BIT;		// 32位宽
	sFilterConfig->FilterActivation = ENABLE;				// 激活滤波器
	sFilterConfig->SlaveStartFilterBank = 14;
	
	HAL_CAN_ConfigFilter(&hcan2, sFilterConfig);
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO1_MSG_PENDING);//FIFO1
	HAL_CAN_Start(&hcan2);
}

/**
************************************************************************
* @brief:      	CAN_Init: CAN初始化
* @param[in]:   void
* @retval:     	void
* @details:    	CAN初始化
************************************************************************
**/
void CAN_Init(void)
{
	CAN_FilterTypeDef sFilterConfig;
	
	// 配置CAN标识符滤波器
	CAN_Filter_Init(&sFilterConfig);
	
	// 使能接收中断
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	
	// 使能接收中断
	HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
************************************************************************
* @brief:      	CAN_SendData_int16_t: CAN发送函数,发送int16_t数据
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   stdId:    指向CAN_HandleTypeDef结构的指针
* @param[in]:   data:     指向CAN_HandleTypeDef结构的指针
* @retval:     	uint8_t
* @details:    	通过CAN总线int16_t命令
************************************************************************
**/
uint8_t CAN_SendData_int16_t(CAN_HandleTypeDef *hcan,uint32_t stdId,int16_t *data)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = stdId;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	// 先发高8位数据，再发低8位数据
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
* @brief:      	CAN_SendData_uint8_t: CAN发送函数,发送uint8_t数据
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   stdId:    CANid地址
* @param[in]:   data:     发送数据
* @param[in]:   len:      发送的数据长度
* @retval:     	uint8_t
* @details:    	通过CAN总线uint8_t命令
************************************************************************
**/
uint8_t CAN_SendData_uint8_t(CAN_HandleTypeDef *hcan,uint32_t stdId,uint8_t *data,uint32_t len)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = stdId;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	
	//判断并赋值数据长度
	if(len<=8)	
	{
	  txFrame.header.DLC=8;     // 发送长度：8byte
	}
	
	// 先发高8位数据，再发低8位数据
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
* @brief:      	HAL_CAN_RxFifo0MsgPendingCallback: CAN1接收中断函数
* @brief:      	HAL_CAN_RxFifo1MsgPendingCallback: CAN2接收中断函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @retval:     	void
* @details:    	通过CAN总线接收返回数据
************************************************************************
**/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance==CAN1)
  {
		//can接收数据存储
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&hcan1RxFrame.header,hcan1RxFrame.data);
		switch(hcan1RxFrame.header.StdId)
		{
			//BM右轮毂电机
			case BM_wheel_right_ID:{CAN_rx_BM_wheel_Data(&Chassis.BM_motor[0][0],hcan1RxFrame.data);break;}
			//BM关节电机
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
		//can接收数据存储
		HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&hcan2RxFrame.header,hcan2RxFrame.data);
		switch(hcan2RxFrame.header.StdId)
		{
			//BM左轮毂电机
			case BM_wheel_left_ID:{CAN_rx_BM_wheel_Data(&Chassis.BM_motor[1][0],hcan2RxFrame.data);break;}
			//BM关节电机
			case BM_joint_left1_ID:{CAN_rx_BM_joint_Data(&Chassis.BM_motor[1][1],hcan2RxFrame.data);break;}
			case BM_joint_left2_ID:{CAN_rx_BM_joint_Data(&Chassis.BM_motor[1][2],hcan2RxFrame.data);break;}
			default:break;		
		}
	}
}
	