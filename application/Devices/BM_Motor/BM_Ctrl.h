#ifndef BM_CTRL_H
#define BM_CTRL_H

#include "main.h"
#include "can.h"
#include "function.h"

//����ָ��Ŀ��ѡ��
#define	BM_ID_range1 				 0x032//����ID1~ID4
#define	BM_ID_range2 				 0x033//����ID5~ID8

//���У׼
#define BM_motor_adjust_ID           0x104//��ַ

//���ÿ���ģʽ
#define BM_mode_set									 0x105//ģʽ����
#define BM_openloop_mode						 0x000//����ģʽ
#define	BM_current_mode  			 			 0x001//������ģʽ
#define	BM_speed_mode    			       0x002//�ٶȻ�ģʽ
#define	BM_disable_mode  			 		 	 0x009//ʧ��ģʽ
#define	BM_enable_mode    			     0x00A//ʹ��ģʽ(Ĭ��ʹ��)

//����ָ���ģʽ
#define	BM_feedback_mode_set_ID 	   0x106//��ַ
#define	BM_passive_mode				       0x080//��ѯģʽ
#define	BM_auto_report_01ms_mode 	   0x001//01ms�����ϱ�ģʽ
#define	BM_auto_report_10ms_mode     0x00A//10ms�����ϱ�ģʽ

//�������ݲ�ѯѡ��(ֻ�ڲ�ѯģʽ����Ч)
#define	BM_feedback_data_choose_ID   0x107//��ַ
#define	BM_motor_speed 							 0x001//�ٶ�
#define	BM_motor_current						 0x001//ĸ�ߵ���
#define	BM_motor_temp 							 0x003//�����¶�
#define	BM_motor_ecd 							 	 0x004//λ��ֵ
#define	BM_motor_error 							 0x005//����ֵ
#define	BM_motor_now_mode						 0x006//��ǰģʽ
#define	BM_motor_user 							 0x0AA//0~255��д�Զ���ֵ���������ַ���֡

//���ID����(һ��һ����)
#define	BM_ID_set_ID 								 0x108//��ַ

//��� CAN �ն˵���ѡͨ����(Ĭ�϶Ͽ�)
#define	BM_can_terminal_R_ctrl_ID    0x109//��ַ
#define BM_can_terminal_R_OFF        0x000//�Ͽ�
#define BM_can_terminal_R_ON       	 0x001//��ͨ

//����̼��汾��ѯ
#define	BM_version_query_ID    			 0x10A//��ַ

//ͨѶ��ʱ��д��������
#define	BM_commun_timeout_set_ID     0x10B//��ַ
#define BM_commun_timeout_be_set		 0x010//����
#define BM_commun_timeout_be_reset	 0x011//��λ
#define BM_sign_write					 			 0x001//д
#define BM_sign_read						 		 0x000//��

//��� PI ��������
//���ռ�ձ����ֵ����(8500 ~ 9600),ʵ��ռ�ձ� = ռ�ձ����ֵ / 100
//�������(������Χ�� 10 ~ 2000)
#define BM_motor_PI_adjust_ID				 0x10C//��ַ
#define BM_motor_MAX_duty_adjuct     0x011//�޸ĵ�����������ռ�ձ�
#define BM_motor_band_width_adjuct   0x021//�޸ĵ���������ֵ
#define BM_motor_data_reset   			 0x0FF//���е��������λ

//��������
#define BM_motor_data_save_ID				 0x10C//��ַ

//�����������һ�ֽ�����
#define BM_end_byte_set_ID 					 0x10E//��ַ
#define BM_end_byte_be_set					 0x030//����
#define BM_end_byte_be_reset				 0x031//��λ
#define BM_sign_write					 			 0x001//д
#define BM_sign_read						 		 0x000//��
#define BM_feedback_ctrl_mode				 0x000//�����������ģʽ(Ĭ�Ϸ�������ģʽ)
#define BM_feedback_temp						 0x001//������������¶�

//���¡�������������ѡͨ����
#define BM_over_protect_ID					 0x110//��ַ
#define BM_over_protect_be_set			 0x050//����
#define BM_over_protect_be_reset		 0x051//��λ 
#define BM_sign_write					 			 0x001//д
#define BM_sign_read						 		 0x000//��
#define BM_over_protect_ON					 0x000//������
#define BM_over_protect_OFF					 0x001//�����ر�
#define BM_over_protect_error				 0x002//����ʧ��

//CAN ��������
#define BM_can_commun_speed_set_ID	 0x111//��ַ
#define BM_can_commun_speed_be_set	 0x060//����
#define BM_can_commun_speed_be_reset 0x061//��λ 
#define BM_sign_write					 			 0x001//д
#define BM_sign_read						 		 0x000//��

//�ؽڵ��������ʽ����
#define BM_feedback_mode_joint_ID    0x034//��ַ
#define BM_passive_mode_joint			   0x000//��ѯģʽ,��ģʽ��,data[2]~data[6]��Ч
#define BM_auto_report_mode_joint    0x001//�����ϱ�ģʽ

//�ؽڵ���������ݲ�ѯ(���е����Ч)
#define BM_data_passive_mode_ID      0x035//��ַ

//��������,ʹ��ʱ��������Ϊλ�û�,0x028,0x004
#define BM_data_write_mode_ID        0x036//��ַ

//������ȡ
#define BM_data_read_mode_ID 				 0x037//��ַ

//�ؽڵ������״̬����
#define BM_ctrl_mode_joint_ID        0x038//��ַ
#define BM_reserve_mode_joint 			 0x000//����
#define BM_disable_mode_joint				 0x001//ʧ��
#define BM_enable_mode_joint				 0x002//ʹ��

//��������
#define BM_save_mode_joint_ID		 		 0x039//��ַ
#define BM_save_by_flash      		   0x001//����Flash����
#define BM_save_abs_zero_point       0x001//����ǰλ������Ϊ������㣬������

//�����λ
#define BM_software_reset_ID				 0x040//��ַ
#define BM_software_reset_all				 0x001//�����λ���е��

//��챵������ֵ������Χ
#define	speed_ratio_wheel 					 1.0f/10.0f//ת�ٱ���ϵ��
#define rpm_to_rad									 2*pi/60//ת��ת�����ٶȱ���ϵ��
#define	current_ratio_wheel 			   55.0f/32767.0f//��������ϵ��
#define	ecd_ratio_wheel 				     pi/32767.0f//�Ƕȱ���ϵ��
#define torque_constant_wheel        0.80f//Nm/A

//�ؽڵ������ֵ������Χ
#define	speed_ratio_joint 					 1.0f/10.0f//ת�ٱ���ϵ��
#define	current_ratio_joint 			   1.0f/100.0f//��������ϵ��
#define	voltage_ratio_joint 			   1.0f/10.0f//��������ϵ��
#define	ecd_ratio_joint 				 		 pi/32767.0f//�Ƕȱ���ϵ��
#define torque_constant_joint   		 1.20f//Nm/A

//������ID
typedef enum
{
	//can1,can2������һ������,��һ�����,�����ؽ�
	//��챵��
	BM_wheel_all_ID=0x096,
	BM_wheel_right_ID=0x097,//ID1
	BM_wheel_left_ID=0x09B,//ID5
	//�ؽڵ��ID
	BM_joint_all_ID=0x050,
	BM_joint_right1_ID=0x052,//ID2
	BM_joint_right2_ID=0x053,//ID3
	BM_joint_left1_ID=0x056,//ID6
	BM_joint_left2_ID=0x057,//ID7
}BM_can_id;//motorID

typedef struct
{
//	uint16_t id;//���ID
	int16_t speed;//���ת��	
	int16_t current;//ת�ص���
	uint16_t ecd;//����Ƕ�
	int16_t last_ecd;//���ǰ�Ƕ�
	
	//ʵ������
	float speed_rpm;//���ת��(rpm/s)
	float speed_rad;//������ٶ�(rad/s)
	float current_A;//ת�ص���(A)
	float voltage_V;//��ѹֵ(V)
	float torque;//Ť��(Nm)
	float ecd_angle;//����Ƕ�(0~2pi)
	float last_ecd_angle;//���ǰ�Ƕ�(0~2pi)
	//���
	uint8_t error;//����ֵ
	uint8_t mode_now;//��ǰģʽ
	//�ؽ�
	int16_t voltage;//��ѹֵ
	
	//��������������
	float set_torque;
}BM_motor_measure_t;

//��챵�����ƺ���
uint8_t BM_motor_ctrl(CAN_HandleTypeDef *hcan,int16_t ctrl_ID_range,int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
uint8_t BM_motor_adjust(CAN_HandleTypeDef *hcan);
//�����ʧ�ܣ��ٽ��п���ģʽ�޸�
uint8_t BM_ctrl_mode_set(CAN_HandleTypeDef *hcan,uint8_t ctrl_mode[8]);

//�ؽڵ������ģʽ
uint8_t BM_ctrl_mode_joint_set(CAN_HandleTypeDef *hcan,uint8_t ctrl_mode[8]);
//�����ʧ�ܣ��ٽ��п���ģʽ�޸�
uint8_t BM_data_write_mode_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t data_sign,int32_t passive_data);

//��챵�����ú���
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
//�ؽڵ�����ú���
uint8_t BM_feedback_mode_joint_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t feedback_mode,uint8_t feedback_time,uint8_t send_data[4]);
uint8_t BM_data_passive_mode_set(CAN_HandleTypeDef *hcan,uint8_t motor_data[4]);
uint8_t BM_data_read_mode_set(CAN_HandleTypeDef *hcan,uint8_t motor_ID,uint8_t data_sign);
uint8_t BM_data_save_joint_set(CAN_HandleTypeDef *hcan,uint8_t reserve_mode,uint8_t abs_zero_point_set_switch);
uint8_t BM_software_reset_set(CAN_HandleTypeDef *hcan,uint8_t software_reset_switch);
void CAN_rx_BM_wheel_Data(BM_motor_measure_t *motor_data,uint8_t *data);
void CAN_rx_BM_joint_Data(BM_motor_measure_t *motor_data,uint8_t *data);

#endif
