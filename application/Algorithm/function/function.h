#ifndef FUNCTION_H
#define FUNCTION_H

#include "main.h"
#include "usart.h"
#include "stdarg.h"
#include "string.h"
#include "stdio.h"
#include "struct_typedef.h"

#define pi 3.1415926358979f

void MyPrintf(UART_HandleTypeDef *huart,const char *__format, ...);
double msp(double x, double in_min, double in_max, double out_min, double out_max);
void LimitMax(fp32 *input,fp32 max);
int float_to_uint(float x_float, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void slope_following(float *target,float *set,float acc);

#endif
