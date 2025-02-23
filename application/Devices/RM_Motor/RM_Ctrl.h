#ifndef RM_CTRL_H
#define RM_CTRL_H

#include "main.h"
#include "can.h"
#include "can_receive.h"

//������ID��ַ
typedef enum
{
	//RM���ID
	CAN_all_chassis_ID=0x200,
	CAN_all_ID=0x200,
	CAN_first_ID=0x201,
	CAN_second_ID=0x202,
	CAN_third_ID=0x203,
	CAN_forth_ID=0x204,
	
	CAN_all_gimbal_ID=0x1FF,
	CAN_gimbal_fir=0x205,
	CAN_gimbal_sec=0x206,
} RM_can_id;//canID

typedef struct{
	uint16_t id;//���ID
	uint16_t state;//�����ַ
	uint16_t ecd;//����Ƕ�
	int16_t speed_rpm;//���ת��
	int16_t given_current;//ת�ص���
	uint8_t temperate;//����¶�
	int16_t last_ecd;//���ǰ�Ƕ�
}motor_measure_t;
extern motor_measure_t motor_chassis[7];//rm������ݷ��ؽṹ��

uint8_t CAN_chassis_send(CAN_HandleTypeDef *hcan,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
uint8_t CAN_gimbal_send(CAN_HandleTypeDef *hcan,int16_t rudder1, int16_t rudder2, int16_t rudder3, int16_t rudder4);
void CAN_rx_rm_Data(motor_measure_t *motor_data,uint8_t *data);

#endif
