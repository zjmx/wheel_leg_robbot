#include "BM_Ctrl.h"

/**
************************************************************************
* @brief:      	CAN_chassis_send: CAN3508_2006电机驱动函数
* @param[in]:   hcan:     指向CAN_HandleTypeDef结构的指针
* @param[in]:   motor1~4: 向指定ID电机发送电流值
* @retval:     	uint8_t
* @details:    	通过CAN总线向特定电机发送启用特定模式的命令
************************************************************************
**/

//电压（Vq）、电流、速度给定
//电机开环指令（－16383～16383）
//电机电流环指令（－16383～16383）Iq 电流值=设置值*55/32767，单位是 A
//电机速度环指令（－5000～5000）－500RPM～500RPM
uint8_t BM_motor_ctrl(CAN_HandleTypeDef *hcan,int16_t ctrl_ID_range,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = ctrl_ID_range;//发送地址
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

//电机校准
uint8_t BM_motor_adjust(CAN_HandleTypeDef *hcan)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_motor_adjust_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	for(int i=0;i<8;i++)
	{
		txFrame.data[i] = 0x000;
	} 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//设置控制模式
uint8_t BM_ctrl_mode_set(CAN_HandleTypeDef *hcan,uint8_t ctrl_mode[8])
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_mode_set;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	for(int i=0;i<8;i++)
	{
		txFrame.data[i]=ctrl_mode[i];
	}	

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//设置反馈模式 (低 7 位为主动上报方式下的上报频率，单位 ms ，范围为1~127ms)
uint8_t BM_motor_feedback_mode_set(CAN_HandleTypeDef *hcan,uint8_t motor_feedback_mode[8])
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_feedback_mode_set_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	for(int i=0;i<8;i++)
	{
		txFrame.data[i] = motor_feedback_mode[i];	
	}

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//返回数据查询选择
uint8_t BM_motor_feedback_data_choose(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t data_choose[3],uint8_t user_data)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_feedback_data_choose_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//目标电机ID
	txFrame.data[1] = (uint8_t)(data_choose[0]);//返回参数选择
	txFrame.data[2] = (uint8_t)(data_choose[1]);
	txFrame.data[3] = (uint8_t)(data_choose[2]);
	txFrame.data[4] = 0x0AA;
	txFrame.data[5] = 0x000; 
	txFrame.data[6] = 0x000; 
	txFrame.data[7] = 0x000; 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//电机ID设置
uint8_t BM_ID_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_ID_set_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//目标电机ID
	for(int i=1;i<8;i++)
	{
		txFrame.data[i] = 0x000;
	}	

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//电机 CAN 终端电阻选通设置(默认断开)
uint8_t BM_can_terminal_R_ctrl_set(CAN_HandleTypeDef *hcan,uint8_t on_off[8])
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_can_terminal_R_ctrl_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	for(int i=0;i<8;i++)
	{
		txFrame.data[i]=on_off[i];
	}	

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//电机固件版本查询
uint8_t BM_version_query_set(CAN_HandleTypeDef *hcan)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_version_query_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	for(int i=0;i<8;i++)
	{
		txFrame.data[i] = 0x000;
	} 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//通讯超时读写操作设置
uint8_t BM_commun_timeout_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t timeout_sign,uint8_t R_W_sign,int16_t timeout_data)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_commun_timeout_set_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//目标电机ID
	txFrame.data[1] = (uint8_t)(timeout_sign);//超时标志
	txFrame.data[2] = (uint8_t)(R_W_sign);//读写标志
	txFrame.data[3] = (uint8_t)(timeout_data>>8);
	txFrame.data[4] = (uint8_t)((timeout_data<<8)>>8);
	txFrame.data[5] = 0x000; 
	txFrame.data[6] = 0x000; 
	txFrame.data[7] = 0x000; 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//电机 PI 参数调节
uint8_t BM_motor_PI_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t adjust_mode,int16_t MAX_duty_or_band_width)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_motor_PI_adjust_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//目标电机ID
	txFrame.data[1] = (uint8_t)(adjust_mode);//调节目标
	txFrame.data[2] = (uint8_t)(MAX_duty_or_band_width>>8);
	txFrame.data[3] = (uint8_t)((MAX_duty_or_band_width<<8)>>8);
	txFrame.data[4] = 0x000;
	txFrame.data[5] = 0x000; 
	txFrame.data[6] = 0x000; 
	txFrame.data[7] = 0x000; 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//参数保存
uint8_t BM_motor_data_save_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_motor_data_save_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//目标电机ID
	txFrame.data[1] = 0x0FE;
	for(int i=2;i<8;i++)
	{
		txFrame.data[i]=0x000;
	} 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//过温、过流保护开关选通设置
uint8_t BM_over_protect_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t switch_sign,uint8_t R_W_sign,uint8_t switch_data)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_end_byte_set_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//目标电机ID
	txFrame.data[1] = (uint8_t)(switch_sign);//最后字节标志
	txFrame.data[2] = (uint8_t)(R_W_sign);//读写标志
	txFrame.data[3] = (uint8_t)(switch_data);//开关选择
	txFrame.data[4] = 0x000;
	txFrame.data[5] = 0x000; 
	txFrame.data[6] = 0x000; 
	txFrame.data[7] = 0x000; 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//CAN 速率设置
uint8_t BM_can_commun_speed_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t speed_set_sign,uint8_t R_W_sign,uint8_t commun_speed)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_can_commun_speed_set_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//目标电机ID
	txFrame.data[1] = (uint8_t)(speed_set_sign);//速率设置标志
	txFrame.data[2] = (uint8_t)(R_W_sign);//读写标志
	txFrame.data[3] = (uint8_t)(commun_speed);//通讯速率选择
	txFrame.data[4] = 0x000;
	txFrame.data[5] = 0x000; 
	txFrame.data[6] = 0x000; 
	txFrame.data[7] = 0x000; 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//关节反馈方式设置
uint8_t BM_feedback_mode_joint_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t feedback_mode,uint8_t feedback_time,uint8_t send_data[4])
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_feedback_mode_joint_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//目标电机ID
	txFrame.data[1] = (uint8_t)(feedback_mode);//反馈模式
	txFrame.data[2] = (uint8_t)(feedback_time);//反馈时间
	txFrame.data[3] = (uint8_t)(send_data[0]);//上报数据[4]
	txFrame.data[4] = (uint8_t)(send_data[1]);
	txFrame.data[5] = (uint8_t)(send_data[2]); 
	txFrame.data[6] = (uint8_t)(send_data[3]); 
	txFrame.data[7] = 0x000; 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//关节电机主动数据查询
uint8_t BM_data_passive_mode_set(CAN_HandleTypeDef *hcan,uint8_t motor_data[4])
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_data_passive_mode_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_data[0]);//查询数据
	txFrame.data[1] = (uint8_t)(motor_data[1]);
	txFrame.data[2] = (uint8_t)(motor_data[2]);
	txFrame.data[3] = (uint8_t)(motor_data[3]);
	txFrame.data[4] = 0x000;
	txFrame.data[5] = 0x000;
	txFrame.data[6] = 0x000; 
	txFrame.data[7] = 0x000; 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//关节电机参数设置
uint8_t BM_data_write_mode_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t data_sign,int32_t passive_data)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_data_write_mode_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//目标电机ID
	txFrame.data[1] = (uint8_t)(data_sign);//参数序号
	txFrame.data[2] = (uint8_t)(passive_data>>24);//查询数据
	txFrame.data[3] = (uint8_t)(passive_data>>16);
	txFrame.data[4] = (uint8_t)(passive_data>>8);
	txFrame.data[5] = (uint8_t)(passive_data); 
	txFrame.data[6] = 0x000; 
	txFrame.data[7] = 0x000; 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//关节电机参数读取
uint8_t BM_data_read_mode_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t data_sign)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_data_write_mode_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(motor_ID);//目标电机ID
	txFrame.data[1] = (uint8_t)(data_sign);//参数序号
	txFrame.data[2] = 0x000;
	txFrame.data[3] = 0x000;
	txFrame.data[4] = 0x000;
	txFrame.data[5] = 0x000; 
	txFrame.data[6] = 0x000; 
	txFrame.data[7] = 0x000; 

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//设置控制模式
uint8_t BM_ctrl_mode_joint_set(CAN_HandleTypeDef *hcan,uint8_t ctrl_mode[8])
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_ctrl_mode_joint_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	for(int i=0;i<8;i++)
	{
		txFrame.data[i]=ctrl_mode[i];
	}	

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//参数保存
uint8_t BM_data_save_joint_set(CAN_HandleTypeDef *hcan,uint8_t reserve_mode,uint8_t abs_zero_point_set_switch)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_save_mode_joint_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(reserve_mode);//选择是否进行flash保存
	txFrame.data[1] = (uint8_t)(abs_zero_point_set_switch);//将当前位置保存为绝对零点
	for(int i=2;i<8;i++)
	{
		txFrame.data[i]=0x000;
	}	

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//软件复位
uint8_t BM_software_reset_set(CAN_HandleTypeDef *hcan,uint8_t software_reset_switch)
{
	uint32_t txMailBox;
	CAN_TxFrameTypeDef txFrame;
	
	txFrame.header.StdId = BM_software_reset_ID;//发送地址
	txFrame.header.IDE = CAN_ID_STD;
	txFrame.header.RTR = CAN_RTR_DATA;
	txFrame.header.DLC = 8;
	
	txFrame.data[0] = (uint8_t)(software_reset_switch);//选择保存模式
	for(int i=1;i<8;i++)
	{
		txFrame.data[i]=0x000;
	}	

	if(HAL_CAN_AddTxMessage(hcan, &txFrame.header, &txFrame.data[0], &txMailBox) != HAL_OK)
	{
		return HAL_ERROR;
	}
	return HAL_OK;
}

//BM电机返回数据解算
void CAN_rx_BM_wheel_Data(BM_motor_measure_t *motor_data,uint8_t *data)                                
{
	motor_data->last_ecd=motor_data->ecd;
	motor_data->last_ecd_angle=(float)(motor_data->last_ecd*ecd_ratio_wheel);//电机前角度(0~360°)
	
	motor_data->speed=(int16_t)((data)[0]<<8|(data)[1]);    
	motor_data->speed_rpm=(float)(motor_data->speed*speed_ratio_wheel);//电机转速(rpm/s)
	motor_data->speed_rad=(float)(motor_data->speed_rpm*rpm_to_rad);//电机角速度(rad/s)
	
	motor_data->current=(int16_t)((data)[2]<<8|(data)[3]);
	motor_data->current_A=(float)(motor_data->current*current_ratio_wheel);//转矩电流(A)
	motor_data->torque=(float)((motor_data->current_A)*torque_constant_wheel);//扭矩(Nm)
	
	motor_data->ecd=(uint16_t)((data)[4]<<8|(data)[5]); 
	motor_data->ecd_angle=(float)msp(motor_data->ecd,0,32768,-pi,pi);//电机角度(0~2pi)
	
	motor_data->error=(data)[6];  
	motor_data->mode_now=(data)[7];
}

void CAN_rx_BM_joint_Data(BM_motor_measure_t *motor_data,uint8_t *data)                                
{
	motor_data->last_ecd=motor_data->ecd;
	motor_data->last_ecd_angle=(float)(motor_data->last_ecd*ecd_ratio_joint);//电机前角度(0~360°)
	
	motor_data->speed=(int16_t)((data)[0]<<8|(data)[1]);    
	motor_data->speed_rpm=(float)(motor_data->speed*speed_ratio_joint);//电机转速(rpm/s)
	motor_data->speed_rad=(float)(motor_data->speed_rpm*rpm_to_rad);//电机角速度(rad/s)
	
	motor_data->current=(int16_t)((data)[2]<<8|(data)[3]);
	motor_data->current_A=(float)(motor_data->current*current_ratio_joint);//转矩电流(A)
	motor_data->torque=(float)((motor_data->current_A/1.414f)*torque_constant_joint);//扭矩(Nm)
	
	motor_data->ecd=(uint16_t)((data)[4]<<8|(data)[5]); 
	motor_data->ecd_angle=(float)(float)msp(motor_data->ecd,0,32768,-pi,pi);//电机角度(0~2pi)
	
	motor_data->voltage=(int16_t)((data)[6]<<8|(data)[7]);  
	motor_data->voltage_V=(float)(motor_data->voltage*voltage_ratio_joint);//母线电压(V)
}

	