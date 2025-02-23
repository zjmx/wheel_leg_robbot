#include "usart_rc_dbus.h"

RC_ctrl_t rc_ctrl; 
uint8_t SBUS_MultiRx_Buf[2][RC_FRAME_LENGTH];//����ԭʼ���ݣ�Ϊ18���ֽڣ�����36���ֽڳ��ȣ���ֹDMA����Խ��

/**
************************************************************************
* @brief:      	RC_init: �����ֲ��ң�����������ݽ��н���
* @param[in]:   void
* @retval:     	void
* @details:    	���մ��ڽ��շ�������
************************************************************************
**/
void RC_init(void)
{
	USART_RxDMA_DoubleBuffer_Init(&huart6,SBUS_MultiRx_Buf[0],SBUS_MultiRx_Buf[1],SBUS_RX_BUF_NUM);
}
/**
************************************************************************
* @brief:      	sbus_to_rc: �����ֲ��ң�����������ݽ��н���
* @param[in]:   sbus_buf:		�������ݴ洢���������
* @param[in]:   rc_ctrl: 		ָ��RC_ctrl_t�ṹ��ָ��
* @retval:     	void
* @details:    	���մ��ڽ��շ�������
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
* @brief:      	USART_DMAEx_MultiBuffer_Init: ˫������ģʽ���ó�ʼ��
* @param[in]:   huart:    						ָ��UART_HandleTypeDef�ṹ��ָ��
* @param[in]:   DstAddress:     			������1��ַ
* @param[in]:   SecondMemAddress:     ������2��ַ
* @param[in]:   DataLength:     			������1+2�����ݳ���
* @retval:     	void
* @details:    	DMA˫������ģʽ���ó�ʼ��
************************************************************************
**/
static void USART_DMAEx_MultiBuffer_Init(UART_HandleTypeDef *huart, uint8_t *DstAddress, uint8_t *SecondMemAddress, uint32_t DataLength)
{
	//���ݽ���ģʽ
	huart->ReceptionType = HAL_UART_RECEPTION_TOIDLE;
	//���ݽ����¼�����
	huart->RxEventType = HAL_UART_RXEVENT_IDLE;
  //���ݽ��ճ���
	huart->RxXferSize  = DataLength;
	//�����ڵĿ��ƼĴ���3��CR3����DMARλ��1��ʹ�ܸô��ڵ�DMA
	SET_BIT(huart->Instance->CR3,USART_CR3_DMAR);
	//ʹ�ܿ����ж�
	__HAL_UART_ENABLE_IT(huart, UART_IT_IDLE); 
	//������DMA�Ĵ��������յ�ĵ�ַǰ��Ҫ�ȹر�DMA���ݴ��䣬���ⷢ���������� 
	do
	{//��CR�Ĵ����е�ENλ��0���˳�ѭ��
		__HAL_DMA_DISABLE(huart->hdmarx);
	}while(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR & DMA_SxCR_EN);
	//����DMA�����յ㡢���
	((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->PAR = (uint32_t)&huart->Instance->DR;
	((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M0AR = (uint32_t)DstAddress;
	((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->M1AR = (uint32_t)SecondMemAddress;
	((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->NDTR = DataLength;

	SET_BIT(((DMA_Stream_TypeDef  *)huart->hdmarx->Instance)->CR, DMA_SxCR_DBM);

	__HAL_DMA_ENABLE(huart->hdmarx);	
}
/**
************************************************************************
* @brief:      	USART_RxDMA_DoubleBuffer_Init: ˫������ģʽ���ó�ʼ��
* @param[in]:   huart:    						ָ��UART_HandleTypeDef�ṹ��ָ��
* @param[in]:   DstAddress:     			������1��ַ
* @param[in]:   SecondMemAddress:     ������2��ַ
* @param[in]:   DataLength:     			������1+2�����ݳ���
* @retval:     	void
* @details:    	DMA˫������ģʽ���ó�ʼ��
************************************************************************
**/
/**ע�⣺�˺�����USART_DMAEx_MultiBuffer_Init������ͬ�������ø�Ϊ���**/
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
* @brief:      	USER_USART6_RxHandler: ˫���������ݴ�����
* @param[in]:   huart:    ָ��UART_HandleTypeDef�ṹ��ָ��
* @param[in]:   Size:     ���յ����ݳ���
* @retval:     	void
* @details:    	�ڸ�����������ָ������·�������
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
* @brief:      	HAL_UARTEx_RxEventCallback: ���ڽ����жϺ�����ͬʱ��ȡ�������ݳ��ȣ�
* @param[in]:   huart:    ָ��UART_HandleTypeDef�ṹ��ָ��
* @param[in]:   Size:     ���յ����ݳ���
* @retval:     	void
* @details:    	���մ��ڽ��շ�������
************************************************************************
**/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size) 
{ 
	 if(huart == &huart6)
	 { 
		 USER_USART6_RxHandler(huart,Size); 
	 }  
}