#ifndef __CAN_RECEIVE_H
#define __CAN_RECEIVE_H

#include "main.h"
#include "can.h"
#include "balance_chair_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;



void CAN_Filter_Init(CAN_FilterTypeDef *sFilterConfig);
void CAN_Init(void);
uint8_t CAN_SendData_int16_t(CAN_HandleTypeDef *hcan,uint32_t stdId,int16_t *data);
uint8_t CAN_SendData_uint8_t(CAN_HandleTypeDef *hcan,uint32_t stdId,uint8_t *data,uint32_t len);

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif
