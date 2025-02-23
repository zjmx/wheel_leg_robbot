#ifndef DRIVER_H
#define DRIVER_H

#include "main.h"
#include "cmsis_os.h"
#include "usart_rc_dbus.h"
#include "balance_chair_task.h"
#include "function.h"

void Driver_task(void);
void rc_data_process(RC_ctrl_t *rc_data,Chassis_t *chassis,float dt);

#endif



