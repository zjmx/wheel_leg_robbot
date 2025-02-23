#ifndef DM_CTRL_H
#define DM_CTRL_H

#include "main.h"
#include "can.h"
#include "can_receive.h"
#include "function.h"

//DM模式及PID参数
#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		0x200

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

#define P_MIN2 -12.0f
#define P_MAX2 12.0f
#define V_MIN2 -45.0f
#define V_MAX2 45.0f
#define KP_MIN2 0.0f
#define KP_MAX2 500.0f
#define KD_MIN2 0.0f
#define KD_MAX2 5.0f
#define T_MIN2 -18.0f
#define T_MAX2 18.0f

//定义DM电机ID
typedef enum
{
	//MIT模式下,DM电机ID
	DM_all_Master_ID=0x000,
	DM_first_Master_ID=0x001,
	DM_second_Master_ID=0x002,
	DM_third_Master_ID=0x003,
} DM_can_id;//canID

typedef struct{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;//电机位置
	float vel;//电机速度
	float tor;//电机扭矩
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
} motor_DM_measure_t;

typedef struct
{
	uint16_t mode;
	motor_DM_measure_t para;
} DM_Motor_t;
extern DM_Motor_t DM_Motor[3];//DM电机数据返回结构体

void enable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
void disable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
void save_pos_zero(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
void clear_err(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id);
void mit_ctrl(CAN_HandleTypeDef* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
void pos_speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float pos, float vel);
void speed_ctrl(CAN_HandleTypeDef* hcan,uint16_t motor_id, float vel);
void CAN_rx_DM_Data(DM_Motor_t *motor_data,uint8_t *data);

#endif
