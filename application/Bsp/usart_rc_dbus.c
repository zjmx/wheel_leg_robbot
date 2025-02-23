#include "usart_rc_dbus.h"

RC_ctrl_t rc_ctrl; 
uint8_t SBUS_MultiRx_Buf[2][RC_FRAME_LENGTH];//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界

/**
************************************************************************
* @brief:      	RC_init: 根据手册对遥控器接收数据进行解算
* @param[in]:   void
* @retval:     	void
* @details:    	接收串口接收返回数据
************************************************************************
**/
void RC_init(void)
{
	USART_RxDMA_DoubleBuffer_Init(&huart6,SBUS_MultiRx_Buf[0],SBUS_MultiRx_Buf[1],SBUS_RX_BUF_NUM);
}
/**
************************************************************************
* @brief:      	sbus_to_rc: 根据手册对遥控器接收数据进行解算
* @param[in]:   sbus_buf:		接收数据存储的数组变量
* @param[in]:   rc_ctrl: 		指向RC_ctrl_t结构的指针
* @retval:     	void
* @details:    	接收串口接收返回数据
************************************************************************
**/
void sbus_to_rc(uint8_t *sbus_buf, RC_ctrl_t *rc_data)	
{
	rc_data->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
	rc_data->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
	rc_data->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
											 (sbus_buf[4] << 10)) &0x07ff;
	rc_data->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
	rc_data->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
	rc_data->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
	rc_data->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
	rc_data->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
	rc_data->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
	rc_data->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
	rc_data->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
	rc_data->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
//	rc_data->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

	rc_data->rc.ch[0] -= RC_CH_VALUE_OFFSET;
	rc_data->rc.ch[1] -= RC_CH_VALUE_OFFSET;
	rc_data->rc.ch[2] -= RC_CH_VALUE_OFFSET;
	rc_data->rc.ch[3] -= RC_CH_VALUE_OFFSET;
//	rc_data->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}
/**
************************************************************************
* @brief:      	USART_DMAEx_MultiBuffer_Init: 双缓冲区模式配置初始化
* @param[in]:   huart:    						指向UART_HandleTypeDef结构的指针
* @param[in]:   DstAddress:     			缓冲区1地址
* @param[in]:   SecondMemAddress:     缓冲区2地址
* @param[in]:   DataLength:     			缓冲区1+2的数据长度
* @retval:     	void
* @details:    	DMA双缓冲区模式配置初始化
************************************************************************
**/
static void USART_DMAEx_MultiBuffer_Init(UART_HandleTypeDef *huart, uint8_t *DstAddress, uint8_t *SecondMemAddress, uint32_t DataLength)
{
	//数据接收模式
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	//数据接收事件类型
	huart->RxEventType = HAL_UART_RXEVENT_IDLE;
  //数据接收长度
	huart->RxXferSize  = DataLength;
	//将串口的控制寄存器3（CR3）的DMAR位置1，使能该串口的DMA
	SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);
	//使能空闲中断
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); 
	//在配置DMA的传输起点和终点的地址前需要先关闭DMA数据传输，以免发生传输意外 
	do
	{//当CR寄存器中的EN位置0，退出循环
		__HAL_DMA_DISABLE(huart->hdmarx);
	}while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);
	//设置DMA传输终点、起点
	((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->DR;
	((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;
	((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;
	((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;

	SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

	__HAL_DMA_ENABLE(huart->hdmarx);	
}
/**
************************************************************************
* @brief:      	USART_RxDMA_DoubleBuffer_Init: 双缓冲区模式配置初始化
* @param[in]:   huart:    						指向UART_HandleTypeDef结构的指针
* @param[in]:   DstAddress:     			缓冲区1地址
* @param[in]:   SecondMemAddress:     缓冲区2地址
* @param[in]:   DataLength:     			缓冲区1+2的数据长度
* @retval:     	void
* @details:    	DMA双缓冲区模式配置初始化
************************************************************************
**/
/**注意：此函数与USART_DMAEx_MultiBuffer_Init作用相同，但配置更为简洁**/
static void USART_RxDMA_DoubleBuffer_Init(UART_HandleTypeDef *huart, uint8_t *DstAddress, uint8_t *SecondMemAddress, uint32_t DataLength)
{ 
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE; 

	huart->RxEventType = HAL_UART_RXEVENT_IDLE; 

	huart->RxXferSize  = DataLength; 

	SET_BIT(huart->Instance->CR3,USART_CR3_DMAR); 

	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);  

	HAL_DMAEx_MultiBufferStart(huart->hdmarx,(uint32_t)&huart->Instance->DR,(uint32_t)DstAddress,(uint32_t)SecondMemAddress,DataLength); 
}
/**
************************************************************************
* @brief:      	USER_USART6_RxHandler: 双缓冲区数据处理函数
* @param[in]:   huart:    指向UART_HandleTypeDef结构的指针
* @param[in]:   Size:     接收的数据长度
* @retval:     	void
* @details:    	在各个缓冲区的指定情况下分配数据
************************************************************************
**/
static void USER_USART6_RxHandler(UART_HandleTypeDef *huart,uint16_t Size)
{ 
	if(((((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR) & DMA_SxCR_CT ) == RESET) 
	{ 
		__HAL_DMA_DISABLE(huart->hdmarx); 

		((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR |= DMA_SxCR_CT; 

		__HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM); 

		if(Size == RC_FRAME_LENGTH) 
		{ 
			sbus_to_rc(SBUS_MultiRx_Buf[0],&rc_ctrl); 
		}
	}
	else
	{ 
	  __HAL_DMA_DISABLE(huart->hdmarx); 

		((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR &= ~(DMA_SxCR_CT); 

		__HAL_DMA_SET_COUNTER(huart->hdmarx,SBUS_RX_BUF_NUM); 

		if(Size == RC_FRAME_LENGTH) 
		{ 
			sbus_to_rc(SBUS_MultiRx_Buf[1],&rc_ctrl); 
		}			 
	} 
	__HAL_DMA_ENABLE(huart->hdmarx);				 
}
/**
************************************************************************
* @brief:      	HAL_UARTEx_RxEventCallback: 串口接收中断函数（同时获取接收数据长度）
* @param[in]:   huart:    指向UART_HandleTypeDef结构的指针
* @param[in]:   Size:     接收的数据长度
* @retval:     	void
* @details:    	接收串口接收返回数据
************************************************************************
**/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size) 
{ 
	 if(huart == &huart6)
	 { 
		 USER_USART6_RxHandler(huart,Size); 
	 }  
}