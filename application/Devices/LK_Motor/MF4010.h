#ifndef MF4010_H
#define MF4010_H

#include "main.h"

#define MF4010_STD_ID						(0x140)
#define MF4010_first_ID					(0x141)
#define MF4010_second_ID				(0x142)
#define MF4010_thrid_ID					(0x143)
#define MF4010_fourth_ID				(0x144)
#define MF4010_fifth_ID					(0x145)


#define MF4010_Read_PID					(0x30)//��ȡ PID��������
#define MF4010_Write_PID_RAM		(0x31)//д�� PID ������ RAM ����
#define MF4010_Write_PID_ROM		(0x32)//д�� PID ������ROM����
#define MF4010_Read_Accle				(0x33)//��ȡ���ٶ�����
#define MF4010_Write_Accle_RAM	(0x34)//д����ٶȵ� RAM ����
#define MF4010_Read_enc					(0x90)//��ȡ����������
#define MF4010_Write_enc_ROM		(0x91)//д�������ֵ��ROM ��Ϊ����������
#define MF4010_Write_nenc_ROM		(0x19)//д�뵱ǰλ�õ�ROM ��Ϊ����������
#define MF4010_Read_aggle_m			(0x92)//��ȡ��Ȧ�Ƕ�����
#define MF4010_Read_aggle_one		(0x94)//��ȡ��Ȧ�Ƕ�����
#define MF4010_Clen_aggle				(0x95)//�������Ƕ�����(���õ����ʼλ��)
#define MF4010_Read_modle1_err	(0x9A)//��ȡ���״̬1�ʹ����־����
#define MF4010_Clen_err 				(0x9B)//��ȡ���״̬1�ʹ����־����
#define MF4010_Read_modle2			(0x9C)//��ȡ���״̬2����
#define MF4010_Read_modle3			(0x9D)//��ȡ���״̬3����
#define MF4010_close						(0x80)//����ر�����
#define MF4010_stop							(0x81)//���ֹͣ����
#define MF4010_Enable						(0x88)//�����������
#define MF4010_torque_open  		(0xA0)//ת�ؿ�����������
#define MF4010_torque_close 		(0xA1)//ת�رջ���������(��ת�رջ�����������ͬ)
#define MF4010_speed_modle			(0xA2)//�ٶȱջ���������
#define MF4010_postion_modle1		(0xA3)//λ�ñջ���������1
#define MF4010_postion_modle2		(0xA4)//λ�ñջ���������2
#define MF4010_postion_modle3		(0xA5)//λ�ñջ���������3
#define MF4010_postion_modle4		(0xA6)//λ�ñջ���������4
#define MF4010_postion_modle5		(0xA7)//λ�ñջ���������5
#define MF4010_postion_modle6		(0xA8)//λ�ñջ���������6
#define MF4010_iq_TAG						(0x280)//������������

typedef struct
{
	int16_t anglePidKp;//�ǶȻ�Kp
	int16_t anglePidKi;//�ǶȻ�Ki
	int16_t speedPidKp;//�ٶȻ�Kp
	int16_t speedPidKi;//�ٶȻ�Kifloat
	int16_t iqPidKp;//ת�ػ�Kp
	int16_t iqPidKi;//ת�ػ�Ki
	
	int32_t Accle;//���ٶ� ��λ 1dps/s
	uint16_t encoderRaw;//������ԭʼλ�� 0~16383
	uint16_t encoderOffset;//��������ƫ 0~16383
	
	int8_t temperature;//�¶� ��λ 1��/LSB
	int16_t iq;//ת�ص���ֵ ��Χ-2048~2048����Ӧʵ��ת�ص�����Χ-33A~33A
	int16_t speed;//ת�� ��λ 1dps/LSB
	uint16_t encoder;//������λ�� 0~16383
	
}MF4010_measure_t;
extern MF4010_measure_t MF4010_motor[2];//������ݷ��ؽṹ��

typedef struct
{
	//�����Ӿ�
	uint8_t frame_rx_header1;//֡ͷ1
	uint8_t frame_rx_header2;//֡ͷ2
	uint16_t yaw_rx_view;//�����Ӿ���yaw��Ŀ��Ƕ�
	uint16_t pitch_rx_view;//�����Ӿ���pitch��Ŀ��Ƕ�
	uint8_t frame_rx_end;//֡β
	
	//�Ӿ�����
	uint8_t frame_tx_header1;//֡ͷ1
	uint8_t frame_tx_header2;//֡ͷ2
	uint16_t yaw_tx_view;//�����Ӿ���yaw��Ŀ��Ƕ�
	uint16_t pitch_tx_view;//�����Ӿ���pitch��Ŀ��Ƕ�
	uint8_t frame_tx_end;//֡β
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



