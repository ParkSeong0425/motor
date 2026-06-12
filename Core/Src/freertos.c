/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "cli.h"
#include "motor.h"
#include "net.h"
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
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for TcpTask */
osThreadId_t TcpTaskHandle;
const osThreadAttr_t TcpTask_attributes = {
  .name = "TcpTask",
  .stack_size = 2048 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;
const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for CliTask */
osThreadId_t CliTaskHandle;
const osThreadAttr_t CliTask_attributes = {
  .name = "CliTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for MotorQueue */
osMessageQueueId_t MotorQueueHandle;
const osMessageQueueAttr_t MotorQueue_attributes = {
  .name = "MotorQueue"
};
/* Definitions for CliprintMutex */
osMutexId_t CliprintMutexHandle;
const osMutexAttr_t CliprintMutex_attributes = {
  .name = "CliprintMutex"
};
/* Definitions for MotorBusMutex */
osMutexId_t MotorBusMutexHandle;
const osMutexAttr_t MotorBusMutex_attributes = {
  .name = "MotorBusMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTcpTask(void *argument);
void StartMotorTask(void *argument);
void StartCliTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of CliprintMutex */
  CliprintMutexHandle = osMutexNew(&CliprintMutex_attributes);

  /* creation of MotorBusMutex */
  MotorBusMutexHandle = osMutexNew(&MotorBusMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of MotorQueue */
  MotorQueueHandle = osMessageQueueNew(8,
                                        sizeof(MotorCommand_t),
                                        &MotorQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask,
                                  NULL,
                                  &defaultTask_attributes);

  /* creation of TcpTask */
  TcpTaskHandle = osThreadNew(StartTcpTask,
                              NULL,
                              &TcpTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask,
                                NULL,
                                &MotorTask_attributes);

  /* creation of CliTask */
  CliTaskHandle = osThreadNew(StartCliTask,
                              NULL,
                              &CliTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  (void)argument;

  for (;;)
  {
    osDelay(1);
  }

  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTcpTask */
/**
* @brief Function implementing the TcpTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTcpTask */
void StartTcpTask(void *argument)
{
  /* USER CODE BEGIN StartTcpTask */

	  Net_TaskRun(argument);

  for (;;)
  {
	    osDelay(1000);
  }

  /* USER CODE END StartTcpTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */
	 Motor_TaskRun(argument);
	  for (;;)
	  {
	    osDelay(1000);
	  }

  /* USER CODE END StartMotorTask */
}

/* USER CODE BEGIN Header_StartCliTask */
/**
* @brief Function implementing the cliTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCliTask */
void StartCliTask(void *argument)
{
  /* USER CODE BEGIN StartCliTask */

	  CLI_TaskRun(argument);

  for (;;)
  {
	    osDelay(1000);
  }

  /* USER CODE END StartCliTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

