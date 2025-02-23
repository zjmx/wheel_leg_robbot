/**
 ******************************************************************************
 * @file    ins_task.h
 * @author  Wang Hongxi
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#ifndef __INS_TASK_H
#define __INS_TASK_H

#include "stdint.h"
#include "cmsis_os.h"
#include "bsp_dwt.h"
#include "BMI088driver.h"
//#include "QuaternionEKF.h"

//#define x 0
//#define y 1
//#define z 2

#define INS_TASK_PERIOD 1

extern float ins_dt;

typedef struct
{
    float q[4]; // ��Ԫ������ֵ

    float Gyro[3];  // ���ٶ�
    float Accel[3]; // ���ٶ�
    float MotionAccel_b[3]; // ����������ٶ�
    float MotionAccel_n[3]; // ����ϵ���ٶ�

    float AccelLPF; // ���ٶȵ�ͨ�˲�ϵ��

    // ���ٶ��ھ���ϵ��������ʾ
    float xn[3];
    float yn[3];
    float zn[3];

    float atanxz;
    float atanyz;

    // λ��
    float Roll;
    float Pitch;
    float Yaw;
    float YawTotalAngle;
		float YawAngleLast;
		float YawRoundCount;
		
		float v_n;//����ϵ����ˮƽ�˶�������ٶ�
		float x_n;//����ϵ����ˮƽ�˶������λ��
		
		uint8_t ins_flag;
} INS_t;


/**
 * @brief ����������װ���Ĳ���,demo�п�����
 * 
 */
typedef struct
{
    uint8_t flag;

    float scale[3];

    float Yaw;
    float Pitch;
    float Roll;
} IMU_Param_t;

extern void INS_Init(void);
extern void INS_task(void);

void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);

#endif


