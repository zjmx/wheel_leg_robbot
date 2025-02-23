#ifndef __CAN_RECEIVE_H
#define __CAN_RECEIVE_H

#include "main.h"
#include "can.h"
#include "pid.h"

extern CAN_HandleTypeDef hcan1;

typedef struct{
	CAN_RxHeaderTypeDef header;
	uint8_t 			data[8];
}CAN_RxFrameTypeDef;

typedef struct{
	CAN_TxHeaderTypeDef header;
	uint8_t				data[8];
}CAN_TxFrameTypeDef;
typedef struct{
	uint16_t ecd;//����Ƕ�
	int16_t speed_rpm;//���ת��
	int16_t given_current;//ת�ص���
	uint8_t temperate;//����¶�
	int16_t last_ecd;//���ǰ�Ƕ�
	
	pid_type_def hub_wheel_pid;//��챵���ٶȻ�PID����ָ��
}motor_measure_t;
extern motor_measure_t motor_chassis[4];//rm������ݷ��ؽṹ��
extern motor_measure_t motor_gimbal[4];//rm������ݷ��ؽṹ��

typedef enum
{
	//����3508���
	CAN_all_chassis_ID=0x200,
	CAN_3508_M1_ID=0x201,
	CAN_3508_M2_ID=0x202,
	CAN_3508_M3_ID=0x203,
	CAN_3508_M4_ID=0x204,
	//��̨6020���
	CAN_all_gimbal_ID=0x1FF,
	CAN_rudder1_ID=0x205,
	CAN_rudder2_ID=0x206,
	CAN_rudder3_ID=0x207,
	CAN_rudder4_ID=0x208,
	//���̵��
} can_id;//canID


void CAN_Filter_Init(CAN_FilterTypeDef *sFilterConfig);
void CAN_Init(void);
uint8_t CAN_SendData(CAN_HandleTypeDef *hcan,uint32_t stdId, int16_t *dat);
//can1��챵����������
uint8_t CAN_chassis_send(CAN_HandleTypeDef *hcan,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);//can1�����������
uint8_t CAN_gimbal_send(CAN_HandleTypeDef *hcan,int16_t rudder1, int16_t rudder2, int16_t rudder3, int16_t rudder4);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif
