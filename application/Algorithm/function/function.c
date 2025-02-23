#include "function.h"

extern UART_HandleTypeDef huart2;

//�������ݰ���ӡ����
void MyPrintf(UART_HandleTypeDef *huart,const char *__format, ...)
{
	#define TX_BUF_LEN  256     /* ���ͻ�����������������Ҫ���е��� */
	uint8_t TxBuf[TX_BUF_LEN];  /* ���ͻ�����                       */
	
  va_list ap;
  va_start(ap, __format);
  
  /* ��շ��ͻ����� */
  memset(TxBuf, 0x00, TX_BUF_LEN);
  
  /* ��䷢�ͻ����� */
  vsnprintf((char*)TxBuf, TX_BUF_LEN, (const char *)__format, ap);
  va_end(ap);
  int len = strlen((const char*)TxBuf);
  
  /* �����ڷ������� */
  HAL_UART_Transmit(huart, (uint8_t*)&TxBuf, len, 0xFFFF);
}

//ӳ�亯��������������ֵ��0~8191��ת��Ϊ�����ƵĽǶ�ֵ��-PI~PI��
double msp(double x, double in_min, double in_max, double out_min, double out_max)
{
	return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

//�޷�����
void LimitMax(fp32 *input,fp32 max)
{
	if(*input>max)
	{
		*input=max;
	}
	else if(*input<-max)
	{
		*input=-max;
	}
}

/**
************************************************************************
* @brief:      	float_to_uint: ������ת��Ϊ�޷�����������
* @param[in]:   x_float:	��ת���ĸ�����
* @param[in]:   x_min:		��Χ��Сֵ
* @param[in]:   x_max:		��Χ���ֵ
* @param[in]:   bits: 		Ŀ���޷���������λ��
* @retval:     	�޷����������
* @details:    	�������ĸ����� x ��ָ����Χ [x_min, x_max] �ڽ�������ӳ�䣬ӳ����Ϊһ��ָ��λ�����޷�������
************************************************************************
**/
int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
	/* Converts a float to an unsigned int, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return (int) ((x_float-offset)*((float)((1<<bits)-1))/span);
}
/**
************************************************************************
* @brief:      	uint_to_float: �޷�������ת��Ϊ����������
* @param[in]:   x_int: ��ת�����޷�������
* @param[in]:   x_min: ��Χ��Сֵ
* @param[in]:   x_max: ��Χ���ֵ
* @param[in]:   bits:  �޷���������λ��
* @retval:     	���������
* @details:    	���������޷������� x_int ��ָ����Χ [x_min, x_max] �ڽ�������ӳ�䣬ӳ����Ϊһ��������
************************************************************************
**/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	/* converts unsigned int to float, given range and number of bits */
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

//�¶ȸ���
void slope_following(float *target,float *set,float acc)
{
	if(*target > *set)
	{
		*set = *set + acc;
		if(*set >= *target)
		*set = *target;
	}
	else if(*target < *set)
	{
		*set = *set - acc;
		if(*set <= *target)
		*set = *target;
	}
}

