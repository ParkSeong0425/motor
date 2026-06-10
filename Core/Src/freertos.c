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
#include "usart.h"
#include <stdio.h>
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
  .priority = (osPriority_t) osPriorityNormal,
};

/* Definitions for SafetyTask */
osThreadId_t SafetyTaskHandle;
const osThreadAttr_t SafetyTask_attributes = {
  .name = "SafetyTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
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
void StartSafetyTask(void *argument);
void StartMotorTask(void *argument);
void StartCliTask(void *argument);

void MX_FREERTOS_Init(void);

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void)
{
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
  MotorQueueHandle = osMessageQueueNew(8, sizeof(MotorCommand_t), &MotorQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */

  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of TcpTask */
  TcpTaskHandle = osThreadNew(StartTcpTask, NULL, &TcpTask_attributes);

  /* creation of SafetyTask */
  SafetyTaskHandle = osThreadNew(StartSafetyTask, NULL, &SafetyTask_attributes);

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* creation of CliTask */
  CliTaskHandle = osThreadNew(StartCliTask, NULL, &CliTask_attributes);

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

  (void)argument;

  for (;;)
  {
    /*
     * 지금 단계에서 TCP는 LAN/IP/MAC 확인용으로만 사용할 예정.
     * 실제 TCP 서버 코드는 나중에 넣는다.
     */
    osDelay(10);
  }

  /* USER CODE END StartTcpTask */
}

/* USER CODE BEGIN Header_StartSafetyTask */
/**
* @brief Function implementing the SafetyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSafetyTask */
void StartSafetyTask(void *argument)
{
  /* USER CODE BEGIN StartSafetyTask */

  (void)argument;

  for (;;)
  {
    /*
     * 나중에 홈센서, DI 상태, 알람 입력 등을 감시할 자리.
     */
    osDelay(10);
  }

  /* USER CODE END StartSafetyTask */
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

  MotorCommand_t cmd;
  MotorCommand_t stop_cmd;
  HAL_StatusTypeDef st;
  uint32_t elapsed;
  uint32_t timeout_ms;
  uint8_t move_done;

  (void)argument;

  Motor_InitIO();

  for (;;)
  {
    if (osMessageQueueGet(MotorQueueHandle, &cmd, NULL, osWaitForever) != osOK)
    {
      osDelay(1);
      continue;
    }

    switch (cmd.id)
    {
      case MOTOR_CMD_MOVE:
        if (cmd.start_ms > 0U)
        {
          osDelay(cmd.start_ms);
        }

        st = Motor_Start(&huart5,
                         cmd.pos,
                         (uint8_t)cmd.speed,
                         (uint16_t)cmd.acc_ms);

        if (st != HAL_OK)
        {
          motor_state.running = 0U;
          motor_state.error = 1U;

          printf("[TASK] Motor_Start failed st=%d last_hal=%d ex=%u crc=%u uart=0x%08lX\r\n",
                 st,
                 motor_state.last_hal,
                 motor_debug.exception_code,
                 motor_debug.crc_ok,
                 (unsigned long)motor_debug.uart_error);

          break;
        }

        /*
         * 중요:
         * StartOn 직후 바로 완료 체크하면 안 된다.
         * 현재 모터에서 diff=0이 나오는 경우가 있어서
         * 바로 Motor_Done() -> StartOff()가 실행될 수 있다.
         */
        osDelay(300);

        elapsed = 300U;
        move_done = 0U;

        timeout_ms = motor_state.move_ms + 5000U;

        if (timeout_ms < 5000U)
        {
          timeout_ms = 5000U;
        }

        if (timeout_ms > 120000U)
        {
          timeout_ms = 120000U;
        }

        while (elapsed < timeout_ms)
        {
          if (osMessageQueueGet(MotorQueueHandle, &stop_cmd, NULL, 0U) == osOK)
          {
            if (stop_cmd.id == MOTOR_CMD_STOP)
            {
              (void)Motor_Stop(&huart5);
              move_done = 1U;
              break;
            }

            if (stop_cmd.id == MOTOR_CMD_ESTOP)
            {
              (void)Motor_EStop(&huart5);
              move_done = 1U;
              break;
            }

            if (stop_cmd.id == MOTOR_CMD_RELEASE)
            {
              (void)Motor_Release(&huart5);
              move_done = 1U;
              break;
            }

            /*
             * MOVE 중 새 MOVE가 들어오면 지금은 무시.
             * 나중에 필요하면 queue 정책을 따로 만든다.
             */
          }

          if (Motor_CheckDone(&huart5) != 0U)
          {
            (void)Motor_Done(&huart5);
            move_done = 1U;
            break;
          }

          osDelay(20);
          elapsed += 20U;
        }

        if (move_done == 0U)
        {
          int32_t pos_dbg = 0;
          int32_t diff_dbg = 0;

          (void)Motor_ReadPos(&huart5, &pos_dbg);
          (void)Motor_ReadDiff(&huart5, &diff_dbg);

          (void)Motor_Stop(&huart5);
          motor_state.error = 1U;

          printf("[TASK] move timeout target=%ld pos=%ld diff=%ld last_hal=%d ex=%u crc=%u uart=0x%08lX\r\n",
                 (long)motor_state.last_target,
                 (long)pos_dbg,
                 (long)diff_dbg,
                 motor_state.last_hal,
                 motor_debug.exception_code,
                 motor_debug.crc_ok,
                 (unsigned long)motor_debug.uart_error);
        }

        if (cmd.wait_ms > 0U)
        {
          osDelay(cmd.wait_ms);
        }
        break;

      case MOTOR_CMD_STOP:
        (void)Motor_Stop(&huart5);
        break;

      case MOTOR_CMD_ESTOP:
        (void)Motor_EStop(&huart5);
        break;

      case MOTOR_CMD_RELEASE:
        (void)Motor_Release(&huart5);
        break;

      case MOTOR_CMD_HOME:
      {
        int32_t first_pos = 0L;
        int32_t last_pos = 0L;
        int32_t now_pos = 0L;
        int32_t diff_pos = 0L;
        uint32_t stable_ms = 0U;
        uint8_t moved = 0U;

        printf("[HOME_TASK] start\r\n");

        st = Motor_StartHome(&huart5);

        if (st != HAL_OK)
        {
          motor_state.running = 0U;
          motor_state.error = 1U;

          printf("[HOME_TASK] start failed st=%d last_hal=%d ex=%u crc=%u uart=0x%08lX\r\n",
                 st,
                 motor_state.last_hal,
                 motor_debug.exception_code,
                 motor_debug.crc_ok,
                 (unsigned long)motor_debug.uart_error);
          break;
        }

        elapsed = 0U;
        timeout_ms = motor_state.move_ms;

        if (timeout_ms < 5000U)
        {
          timeout_ms = 5000U;
        }

        if (timeout_ms > 70000U)
        {
          timeout_ms = 70000U;
        }

        if (Motor_ReadPos(&huart5, &first_pos) != HAL_OK)
        {
          first_pos = motor_state.cur_pos;
        }

        last_pos = first_pos;

        while (elapsed < timeout_ms)
        {
          if (osMessageQueueGet(MotorQueueHandle, &stop_cmd, NULL, 0U) == osOK)
          {
            if (stop_cmd.id == MOTOR_CMD_STOP)
            {
              (void)Motor_Stop(&huart5);
              printf("[HOME_TASK] stopped\r\n");
              break;
            }

            if (stop_cmd.id == MOTOR_CMD_ESTOP)
            {
              (void)Motor_EStop(&huart5);
              printf("[HOME_TASK] estop\r\n");
              break;
            }

            if (stop_cmd.id == MOTOR_CMD_RELEASE)
            {
              (void)Motor_Release(&huart5);
              printf("[HOME_TASK] release\r\n");
              break;
            }
          }

          if (Motor_ReadPos(&huart5, &now_pos) == HAL_OK)
          {
            diff_pos = now_pos - first_pos;

            if (diff_pos < 0L)
            {
              diff_pos = -diff_pos;
            }

            if (diff_pos > 50L)
            {
              moved = 1U;
            }

            diff_pos = now_pos - last_pos;

            if (diff_pos < 0L)
            {
              diff_pos = -diff_pos;
            }

            if (diff_pos <= 5L)
            {
              stable_ms += 100U;
            }
            else
            {
              stable_ms = 0U;
              last_pos = now_pos;
            }

            /*
             * 모터가 실제로 움직인 뒤,
             * 위치가 1초 이상 거의 안 변하면
             * 센서 또는 드라이버 내부 정지로 보고 home 저장.
             */
            if (moved != 0U && stable_ms >= 1000U)
            {
              (void)Motor_SaveHomeHere(&huart5);

              printf("[HOME_TASK] done pos=%ld offset=%ld\r\n",
                     (long)motor_state.cur_pos,
                     (long)motor_state.home_offset);
              break;
            }
          }

          osDelay(100);
          elapsed += 100U;
        }

        if (elapsed >= timeout_ms)
        {
          (void)Motor_Stop(&huart5);
          motor_state.error = 1U;

          printf("[HOME_TASK] timeout\r\n");
        }

        break;

      default:
        break;
    }
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

  (void)argument;

  CLI_Init();

  for (;;)
  {
    CLI_Poll();
    osDelay(1);
  }

  /* USER CODE END StartCliTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
