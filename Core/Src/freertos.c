/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ins_task.h"
#include "balance_chair_task.h"
#include "observe_task.h"
#include "Driver_task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId INS_TASKHandle;
osThreadId LEG_RIGHTHandle;
osThreadId LEG_LEFTHandle;
osThreadId BALAENCE_CHAIRHandle;
osThreadId OBSERVE_TASKHandle;
osThreadId DRIVER_TASKHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void INS_Task(void const * argument);
void LEG_Right(void const * argument);
void LEG_Left(void const * argument);
void BALANCE_Chair(void const * argument);
void Observe_Task(void const * argument);
void DRIVER_Task(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of INS_TASK */
  osThreadDef(INS_TASK, INS_Task, osPriorityRealtime, 0, 512);
  INS_TASKHandle = osThreadCreate(osThread(INS_TASK), NULL);

  /* definition and creation of LEG_RIGHT */
  osThreadDef(LEG_RIGHT, LEG_Right, osPriorityAboveNormal, 0, 512);
  LEG_RIGHTHandle = osThreadCreate(osThread(LEG_RIGHT), NULL);

  /* definition and creation of LEG_LEFT */
  osThreadDef(LEG_LEFT, LEG_Left, osPriorityAboveNormal, 0, 512);
  LEG_LEFTHandle = osThreadCreate(osThread(LEG_LEFT), NULL);

  /* definition and creation of BALAENCE_CHAIR */
  osThreadDef(BALAENCE_CHAIR, BALANCE_Chair, osPriorityAboveNormal, 0, 512);
  BALAENCE_CHAIRHandle = osThreadCreate(osThread(BALAENCE_CHAIR), NULL);

  /* definition and creation of OBSERVE_TASK */
  osThreadDef(OBSERVE_TASK, Observe_Task, osPriorityHigh, 0, 512);
  OBSERVE_TASKHandle = osThreadCreate(osThread(OBSERVE_TASK), NULL);

  /* definition and creation of DRIVER_TASK */
  osThreadDef(DRIVER_TASK, DRIVER_Task, osPriorityAboveNormal, 0, 128);
  DRIVER_TASKHandle = osThreadCreate(osThread(DRIVER_TASK), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_INS_Task */
/**
* @brief Function implementing the INS_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_INS_Task */
void INS_Task(void const * argument)
{
  /* USER CODE BEGIN INS_Task */
  /* Infinite loop */
  for(;;)
  {
    INS_task();
		osDelay(1);
  }
  /* USER CODE END INS_Task */
}

/* USER CODE BEGIN Header_LEG_Right */
/**
* @brief Function implementing the LEG_RIGHT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LEG_Right */
void LEG_Right(void const * argument)
{
  /* USER CODE BEGIN LEG_Right */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LEG_Right */
}

/* USER CODE BEGIN Header_LEG_Left */
/**
* @brief Function implementing the LEG_LEFT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LEG_Left */
void LEG_Left(void const * argument)
{
  /* USER CODE BEGIN LEG_Left */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END LEG_Left */
}

/* USER CODE BEGIN Header_BALANCE_Chair */
/**
* @brief Function implementing the BALAENCE_CHAIR thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BALANCE_Chair */
void BALANCE_Chair(void const * argument)
{
  /* USER CODE BEGIN BALANCE_Chair */
  /* Infinite loop */
  for(;;)
  {
		balance_chair_task();
    osDelay(1);
  }
  /* USER CODE END BALANCE_Chair */
}

/* USER CODE BEGIN Header_Observe_Task */
/**
* @brief Function implementing the OBSERVE_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Observe_Task */
void Observe_Task(void const * argument)
{
  /* USER CODE BEGIN Observe_Task */
  /* Infinite loop */
  for(;;)
  {
		Observe_task();
    osDelay(1);
  }
  /* USER CODE END Observe_Task */
}

/* USER CODE BEGIN Header_DRIVER_Task */
/**
* @brief Function implementing the DRIVER_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DRIVER_Task */
void DRIVER_Task(void const * argument)
{
  /* USER CODE BEGIN DRIVER_Task */
  /* Infinite loop */
  for(;;)
  {
		Driver_task();
    osDelay(1);
  }
  /* USER CODE END DRIVER_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
