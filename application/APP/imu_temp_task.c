#include "imu_temp_task.h"

#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm∏¯∂®

#define TEMPERATURE_PID_KP 0.0f //kp of temperature control PID 
#define TEMPERATURE_PID_KI 0.0f    //ki of temperature control PID 
#define TEMPERATURE_PID_KD 0.0f    //kd of temperature control PID 

#define TEMPERATURE_PID_MAX_OUT 4500.0f  //max out of temperature control PID 
#define TEMPERATURE_PID_MAX_IOUT 4400.0f //max iout of temperature control PID 

fp32 gyro[3], accel[3], temp;

const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};

pid_type_def imu_temp_pid;
extern IMU_Data_t BMI088;
extern SPI_HandleTypeDef hspi2;

void imu_pwm_set(uint16_t pwm)
{
    TIM2->CCR2 = (pwm);
}

uint16_t pwm;
void imu_temp_task()
{
	PID_init(&imu_temp_pid, imu_temp_PID, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	while(1)
	{
		BMI088_Read(&BMI088);
		uint16_t tempPWM;
		//pid calculate. PIDº∆À„
		PID_calc(&imu_temp_pid, BMI088.Temperature, 40.0f);
		if (imu_temp_pid.out < 0.0f)
		{
			imu_temp_pid.out = 0.0f;
		}
		tempPWM = (uint16_t)imu_temp_pid.out;
		pwm=tempPWM;
		IMU_temp_PWM(tempPWM);
	}
}

