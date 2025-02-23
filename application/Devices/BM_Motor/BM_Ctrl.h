#ifndef BM_CTRL_H
#define BM_CTRL_H

#include "main.h"
#include "can.h"
#include "function.h"

//控制指令目标选择
#define	BM_ID_range1 				 0x032//控制ID1~ID4
#define	BM_ID_range2 				 0x033//控制ID5~ID8

//电机校准
#define BM_motor_adjust_ID           0x104//地址

//设置控制模式
#define BM_mode_set									 0x105//模式设置
#define BM_openloop_mode						 0x000//开环模式
#define	BM_current_mode  			 			 0x001//电流环模式
#define	BM_speed_mode    			       0x002//速度环模式
#define	BM_disable_mode  			 		 	 0x009//失能模式
#define	BM_enable_mode    			     0x00A//使能模式(默认使能)

//设置指令反馈模式
#define	BM_feedback_mode_set_ID 	   0x106//地址
#define	BM_passive_mode				       0x080//查询模式
#define	BM_auto_report_01ms_mode 	   0x001//01ms主动上报模式
#define	BM_auto_report_10ms_mode     0x00A//10ms主动上报模式

//返回数据查询选择(只在查询模式下有效)
#define	BM_feedback_data_choose_ID   0x107//地址
#define	BM_motor_speed 							 0x001//速度
#define	BM_motor_current						 0x001//母线电流
#define	BM_motor_temp 							 0x003//绕组温度
#define	BM_motor_ecd 							 	 0x004//位置值
#define	BM_motor_error 							 0x005//故障值
#define	BM_motor_now_mode						 0x006//当前模式
#define	BM_motor_user 							 0x0AA//0~255，写自定义值，用于区分返回帧

//电机ID设置(一个一个标)
#define	BM_ID_set_ID 								 0x108//地址

//电机 CAN 终端电阻选通设置(默认断开)
#define	BM_can_terminal_R_ctrl_ID    0x109//地址
#define BM_can_terminal_R_OFF        0x000//断开
#define BM_can_terminal_R_ON       	 0x001//接通

//电机固件版本查询
#define	BM_version_query_ID    			 0x10A//地址

//通讯超时读写操作设置
#define	BM_commun_timeout_set_ID     0x10B//地址
#define BM_commun_timeout_be_set		 0x010//设置
#define BM_commun_timeout_be_reset	 0x011//复位
#define BM_sign_write					 			 0x001//写
#define BM_sign_read						 		 0x000//读

//电机 PI 参数调节
//输出占空比最大值调节(8500 ~ 9600),实际占空比 = 占空比输出值 / 100
//带宽调节(环带宽范围： 10 ~ 2000)
#define BM_motor_PI_adjust_ID				 0x10C//地址
#define BM_motor_MAX_duty_adjuct     0x011//修改电流环输出最大占空比
#define BM_motor_band_width_adjuct   0x021//修改电流环带宽值
#define BM_motor_data_reset   			 0x0FF//所有电机参数复位

//参数保存
#define BM_motor_data_save_ID				 0x10C//地址

//反馈数据最后一字节设置
#define BM_end_byte_set_ID 					 0x10E//地址
#define BM_end_byte_be_set					 0x030//设置
#define BM_end_byte_be_reset				 0x031//复位
#define BM_sign_write					 			 0x001//写
#define BM_sign_read						 		 0x000//读
#define BM_feedback_ctrl_mode				 0x000//反馈电机控制模式(默认反馈控制模式)
#define BM_feedback_temp						 0x001//反馈电机绕组温度

//过温、过流保护开关选通设置
#define BM_over_protect_ID					 0x110//地址
#define BM_over_protect_be_set			 0x050//设置
#define BM_over_protect_be_reset		 0x051//复位 
#define BM_sign_write					 			 0x001//写
#define BM_sign_read						 		 0x000//读
#define BM_over_protect_ON					 0x000//保护打开
#define BM_over_protect_OFF					 0x001//保护关闭
#define BM_over_protect_error				 0x002//设置失败

//CAN 速率设置
#define BM_can_commun_speed_set_ID	 0x111//地址
#define BM_can_commun_speed_be_set	 0x060//设置
#define BM_can_commun_speed_be_reset 0x061//复位 
#define BM_sign_write					 			 0x001//写
#define BM_sign_read						 		 0x000//读

//关节电机反馈方式设置
#define BM_feedback_mode_joint_ID    0x034//地址
#define BM_passive_mode_joint			   0x000//查询模式,此模式下,data[2]~data[6]无效
#define BM_auto_report_mode_joint    0x001//主动上报模式

//关节电机主动数据查询(所有电机有效)
#define BM_data_passive_mode_ID      0x035//地址

//参数设置,使用时建议设置为位置环,0x028,0x004
#define BM_data_write_mode_ID        0x036//地址

//参数读取
#define BM_data_read_mode_ID 				 0x037//地址

//关节电机控制状态设置
#define BM_ctrl_mode_joint_ID        0x038//地址
#define BM_reserve_mode_joint 			 0x000//保留
#define BM_disable_mode_joint				 0x001//失能
#define BM_enable_mode_joint				 0x002//使能

//参数保存
#define BM_save_mode_joint_ID		 		 0x039//地址
#define BM_save_by_flash      		   0x001//进行Flash保存
#define BM_save_abs_zero_point       0x001//将当前位置设置为绝对零点，并保存

//软件复位
#define BM_software_reset_ID				 0x040//地址
#define BM_software_reset_all				 0x001//软件复位所有电机

//轮毂电机返回值参数范围
#define	speed_ratio_wheel 					 1.0f/10.0f//转速比例系数
#define rpm_to_rad									 2*pi/60//转速转换角速度比例系数
#define	current_ratio_wheel 			   55.0f/32767.0f//电流比例系数
#define	ecd_ratio_wheel 				     pi/32767.0f//角度比例系数
#define torque_constant_wheel        0.80f//Nm/A

//关节电机返回值参数范围
#define	speed_ratio_joint 					 1.0f/10.0f//转速比例系数
#define	current_ratio_joint 			   1.0f/100.0f//电流比例系数
#define	voltage_ratio_joint 			   1.0f/10.0f//电流比例系数
#define	ecd_ratio_joint 				 		 pi/32767.0f//角度比例系数
#define torque_constant_joint   		 1.20f//Nm/A

//定义电机ID
typedef enum
{
	//can1,can2各控制一边轮腿,即一个轮毂,两个关节
	//轮毂电机
	BM_wheel_all_ID=0x096,
	BM_wheel_right_ID=0x097,//ID1
	BM_wheel_left_ID=0x09B,//ID5
	//关节电机ID
	BM_joint_all_ID=0x050,
	BM_joint_right1_ID=0x052,//ID2
	BM_joint_right2_ID=0x053,//ID3
	BM_joint_left1_ID=0x056,//ID6
	BM_joint_left2_ID=0x057,//ID7
}BM_can_id;//motorID

typedef struct
{
//	uint16_t id;//电机ID
	int16_t speed;//电机转速	
	int16_t current;//转矩电流
	uint16_t ecd;//电机角度
	int16_t last_ecd;//电机前角度
	
	//实际数据
	float speed_rpm;//电机转速(rpm/s)
	float speed_rad;//电机角速度(rad/s)
	float current_A;//转矩电流(A)
	float voltage_V;//电压值(V)
	float torque;//扭矩(Nm)
	float ecd_angle;//电机角度(0~2pi)
	float last_ecd_angle;//电机前角度(0~2pi)
	//轮毂
	uint8_t error;//故障值
	uint8_t mode_now;//当前模式
	//关节
	int16_t voltage;//电压值
	
	//电机输出期望力矩
	float set_torque;
}BM_motor_measure_t;

//轮毂电机控制函数
uint8_t BM_motor_ctrl(CAN_HandleTypeDef *hcan,int16_t ctrl_ID_range,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
uint8_t BM_motor_adjust(CAN_HandleTypeDef *hcan);
//电机先失能，再进行控制模式修改
uint8_t BM_ctrl_mode_set(CAN_HandleTypeDef *hcan,uint8_t ctrl_mode[8]);

//关节电机控制模式
uint8_t BM_ctrl_mode_joint_set(CAN_HandleTypeDef *hcan,uint8_t ctrl_mode[8]);
//电机先失能，再进行控制模式修改
uint8_t BM_data_write_mode_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t data_sign,int32_t passive_data);

//轮毂电机设置函数
uint8_t BM_feedback_mode_set(CAN_HandleTypeDef *hcan,uint8_t motor_feedback_mode[8]);
uint8_t BM_feedback_data_choose(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t data_choose[3],uint8_t user_data);
uint8_t BM_ID_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID);
uint8_t BM_can_terminal_R_ctrl_set(CAN_HandleTypeDef *hcan,uint8_t on_off[8]);
uint8_t BM_version_query_set(CAN_HandleTypeDef *hcan);
uint8_t BM_commun_timeout_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t timeout_sign,uint8_t R_W_sign,int16_t timeout_data);
uint8_t BM_PI_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t adjust_mode,int16_t MAX_duty_or_band_width);
uint8_t BM_data_save_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID);
uint8_t BM_over_protect_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t switch_sign,uint8_t R_W_sign,uint8_t switch_data);
uint8_t BM_can_commun_speed_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t speed_set_sign,uint8_t R_W_sign,uint8_t commun_speed);
//关节电机设置函数
uint8_t BM_feedback_mode_joint_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t feedback_mode,uint8_t feedback_time,uint8_t send_data[4]);
uint8_t BM_data_passive_mode_set(CAN_HandleTypeDef *hcan,uint8_t motor_data[4]);
uint8_t BM_data_read_mode_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t data_sign);
uint8_t BM_data_save_joint_set(CAN_HandleTypeDef *hcan,uint8_t reserve_mode,uint8_t abs_zero_point_set_switch);
uint8_t BM_software_reset_set(CAN_HandleTypeDef *hcan,uint8_t software_reset_switch);
void CAN_rx_BM_wheel_Data(BM_motor_measure_t *motor_data,uint8_t *data);
void CAN_rx_BM_joint_Data(BM_motor_measure_t *motor_data,uint8_t *data);

#endif
