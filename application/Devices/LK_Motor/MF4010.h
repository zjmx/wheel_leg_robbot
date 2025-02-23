#ifndef MF4010_H
#define MF4010_H

#include "main.h"

#define MF4010_STD_ID						(0x140)
#define MF4010_first_ID					(0x141)
#define MF4010_second_ID				(0x142)
#define MF4010_thrid_ID					(0x143)
#define MF4010_fourth_ID				(0x144)
#define MF4010_fifth_ID					(0x145)


#define MF4010_Read_PID					(0x30)//读取 PID参数命令
#define MF4010_Write_PID_RAM		(0x31)//写入 PID 参数到 RAM 命令
#define MF4010_Write_PID_ROM		(0x32)//写入 PID 参数到ROM命令
#define MF4010_Read_Accle				(0x33)//读取加速度命令
#define MF4010_Write_Accle_RAM	(0x34)//写入加速度到 RAM 命令
#define MF4010_Read_enc					(0x90)//读取编码器命令
#define MF4010_Write_enc_ROM		(0x91)//写入编码器值到ROM 作为电机零点命令
#define MF4010_Write_nenc_ROM		(0x19)//写入当前位置到ROM 作为电机零点命令
#define MF4010_Read_aggle_m			(0x92)//读取多圈角度命令
#define MF4010_Read_aggle_one		(0x94)//读取单圈角度命令
#define MF4010_Clen_aggle				(0x95)//清除电机角度命令(设置电机初始位置)
#define MF4010_Read_modle1_err	(0x9A)//读取电机状态1和错误标志命令
#define MF4010_Clen_err 				(0x9B)//读取电机状态1和错误标志命令
#define MF4010_Read_modle2			(0x9C)//读取电机状态2命令
#define MF4010_Read_modle3			(0x9D)//读取电机状态3命令
#define MF4010_close						(0x80)//电机关闭命令
#define MF4010_stop							(0x81)//电机停止命令
#define MF4010_Enable						(0x88)//电机运行命令
#define MF4010_torque_open  		(0xA0)//转矩开环控制命令
#define MF4010_torque_close 		(0xA1)//转矩闭环控制命令(与转矩闭环控制命令相同)
#define MF4010_speed_modle			(0xA2)//速度闭环控制命令
#define MF4010_postion_modle1		(0xA3)//位置闭环控制命令1
#define MF4010_postion_modle2		(0xA4)//位置闭环控制命令2
#define MF4010_postion_modle3		(0xA5)//位置闭环控制命令3
#define MF4010_postion_modle4		(0xA6)//位置闭环控制命令4
#define MF4010_postion_modle5		(0xA7)//位置闭环控制命令5
#define MF4010_postion_modle6		(0xA8)//位置闭环控制命令6
#define MF4010_iq_TAG						(0x280)//多电机控制命令

typedef struct
{
	int16_t anglePidKp;//角度环Kp
	int16_t anglePidKi;//角度环Ki
	int16_t speedPidKp;//速度环Kp
	int16_t speedPidKi;//速度环Kifloat
	int16_t iqPidKp;//转矩环Kp
	int16_t iqPidKi;//转矩环Ki
	
	int32_t Accle;//加速度 单位 1dps/s
	uint16_t encoderRaw;//编码器原始位置 0~16383
	uint16_t encoderOffset;//编码器零偏 0~16383
	
	int8_t temperature;//温度 单位 1℃/LSB
	int16_t iq;//转矩电流值 范围-2048~2048，对应实际转矩电流范围-33A~33A
	int16_t speed;//转速 单位 1dps/LSB
	uint16_t encoder;//编码器位置 0~16383
	
}MF4010_measure_t;
extern MF4010_measure_t MF4010_motor[2];//电机数据返回结构体

typedef struct
{
	//接收视觉
	uint8_t frame_rx_header1;//帧头1
	uint8_t frame_rx_header2;//帧头2
	uint16_t yaw_rx_view;//接收视觉的yaw轴目标角度
	uint16_t pitch_rx_view;//接收视觉的pitch轴目标角度
	uint8_t frame_rx_end;//帧尾
	
	//视觉发送
	uint8_t frame_tx_header1;//帧头1
	uint8_t frame_tx_header2;//帧头2
	uint16_t yaw_tx_view;//接收视觉的yaw轴目标角度
	uint16_t pitch_tx_view;//接收视觉的pitch轴目标角度
	uint8_t frame_tx_end;//帧尾
}View_measure_t;
extern View_measure_t MF4010_View;

uint8_t MF4010_Read_data(CAN_HandleTypeDef* hcan,uint16_t ID, int16_t enable);
uint8_t MF4010_Write_PID(CAN_HandleTypeDef* hcan,uint16_t ID,uint16_t modle,float anglePI[2],float speedPI[2],float iqPI[2]);
uint8_t MF4010_Write_Accel(CAN_HandleTypeDef* hcan,uint16_t ID,uint16_t modle,int32_t Accel);
uint8_t MF4010_Write_aggle(CAN_HandleTypeDef* hcan,uint16_t ID, uint16_t modle,int32_t angleControl);
uint8_t MF4010_Write_speed_close(CAN_HandleTypeDef* hcan,uint16_t ID,uint16_t modle,int32_t speedControl);
uint8_t MF4010_Write_aggle_close(CAN_HandleTypeDef* hcan,uint16_t ID,uint16_t modle,uint8_t spinDirection,uint16_t maxSpeed,int32_t angleControl);
uint8_t MF4010_Write_multi_motor(CAN_HandleTypeDef* hcan,uint16_t ID,int16_t iqControl_1,int16_t iqControl_2,int16_t iqControl_3,int16_t iqControl_4);
//void view_Rx_analysis(View_measure_t *view_rx_data,uint8_t data[8]);
void MF4010_Rx_analysis(MF4010_measure_t *motor_rx_data,uint8_t data[8]);
#endif



