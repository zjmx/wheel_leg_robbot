//#include "usart_rc_dbus.h"

//RC_ctrl_t rc_ctrl; 
//uint8_t sbus_rx_buf[18];//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界

////遥控器串口DMA接收初始化
//void RC_init(void)
//{
//	HAL_UART_Receive_DMA(&huart3,sbus_rx_buf,sizeof(sbus_rx_buf));
////	HAL_UART_Receive_DMA(&huart5,sbus_rx_buf,sizeof(sbus_rx_buf));
//}

////遥控器数据解算，将DMA接收到的数据存入结构体成员
//void sbus_to_rc(uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
//{
//	rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
//	rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
//	rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
//											 (sbus_buf[4] << 10)) &0x07ff;
//	rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
//	rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                  //!< Switch left
//	rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                       //!< Switch right
//	rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
//	rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
//	rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
//	rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
//	rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
//	rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
////	rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

//	rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET;
//	rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
//	rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
//	rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
////	rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
//}

////串口中断回调函数
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance==USART3)
//	{
//		sbus_to_rc(sbus_rx_buf,&rc_ctrl);
//		HAL_UART_Receive_DMA(&huart3,sbus_rx_buf,sizeof(sbus_rx_buf));//重新开启DMA接收
//	}
//}


