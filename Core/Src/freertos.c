/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : FreeRTOS application code
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
#include "tcp_cmd_server.h"
#include "motor.h"
#include "usart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define TCP_CMD_SOCKET_NUM     0U
#define TCP_CMD_PORT           5000U
#define TCP_CMD_IP_MODE        TCP_CMD_MODE_IPV4

#define CREATE_SAFETY_TASK     0U
#define CREATE_LAMP_TASK       0U

#define USE_ESTOP_BTN          0U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

static uint8_t tcp_buf[2048];

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
/* Definitions for MotorQueue */
osMessageQueueId_t MotorQueueHandle;
const osMessageQueueAttr_t MotorQueue_attributes = {
  .name = "MotorQueue"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static uint8_t Motor_WaitMove(uint32_t wait_ms);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTcpTask(void *argument);
void StartSafetyTask(void *argument);
void StartMotorTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of MotorQueue */
  MotorQueueHandle = osMessageQueueNew (8, sizeof(MotorCommand_t), &MotorQueue_attributes);

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
      osDelay(1000);
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
      TcpCmdServer_Process(
          TCP_CMD_SOCKET_NUM,
          tcp_buf,
          TCP_CMD_PORT,
          TCP_CMD_IP_MODE
      );

      osDelay(1);
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
#if (USE_ESTOP_BTN != 0U)
      if (HAL_GPIO_ReadPin(ESTOP_BTN_GPIO_Port, ESTOP_BTN_Pin) == GPIO_PIN_RESET)
      {
          (void)Motor_SendEStop();
      }
#endif

      osDelay(5);
  }

  /* USER CODE END StartSafetyTask */
}

/* USER CODE BEGIN Header_StartMotorTask */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/*
 * 모터 이동 중 대기 함수.
 *
 * 기존:
 *   osDelay(move_ms);
 *
 * 변경:
 *   5ms마다 Queue를 확인한다.
 *   이동 중 01S가 들어오면 즉시 비상정지한다.
 *   이동 중 STOP이 들어오면 일반 정지한다.
 *
 * return:
 *   0 = 정상적으로 시간이 끝남
 *   1 = 중간에 정지/비상정지 처리됨
 */
static uint8_t Motor_WaitMove(uint32_t wait_ms)
{
    uint32_t start_tick;
    uint32_t now_tick;
    MotorCommand_t cmd;

    start_tick = osKernelGetTickCount();

    for (;;)
    {
        now_tick = osKernelGetTickCount();

        if ((now_tick - start_tick) >= wait_ms)
        {
            return 0U;
        }

        /*
         * 이동 중 새 명령 확인.
         * timeout 0 = 기다리지 않고 확인만 한다.
         */
        if (osMessageQueueGet(MotorQueueHandle, &cmd, NULL, 0U) == osOK)
        {
            if (cmd.id == MOTOR_CMD_ESTOP)
            {
                /*
                 * 01S 비상정지.
                 */
                (void)Motor_EStop(&huart5);

                motor_state.running = 0U;
                motor_state.error = 1U;

                return 1U;
            }

            if (cmd.id == MOTOR_CMD_STOP)
            {
                /*
                 * 일반 정지.
                 */
                (void)Motor_Stop(&huart5);

                motor_state.running = 0U;

                return 1U;
            }

            if (cmd.id == MOTOR_CMD_RELEASE)
            {
                /*
                 * 이동 중 01D는 여기서 바로 처리하지 않는다.
                 * 01S 후 정지 상태에서 다시 01D를 보내는 흐름으로 둔다.
                 */
            }

            /*
             * 이동 중 MOVE는 실행하지 않는다.
             * 현재 Motor_SendMove()에서 busy/queue count로 대부분 막지만,
             * 혹시 들어와도 여기서는 무시한다.
             */
        }

        osDelay(5);
    }
}
/* USER CODE END Header_StartMotorTask */
void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */

  MotorCommand_t cmd;
  HAL_StatusTypeDef st;

  (void)argument;

  /*
   * 모터 관련 GPIO, RS485 방향, 내부 상태 초기화.
   * main.c가 아니라 MotorTask 시작 시 한 번만 수행한다.
   */
  Motor_InitIO();

  for (;;)
  {
      /*
       * Queue에 명령이 들어올 때까지 MotorTask만 대기한다.
       * TCP Task는 막히지 않는다.
       */
      if (osMessageQueueGet(MotorQueueHandle, &cmd, NULL, osWaitForever) != osOK)
      {
          osDelay(10);
          continue;
      }

      switch (cmd.id)
      {
          case MOTOR_CMD_MOVE:
              /*
               * 이동 명령 처리.
               * 여기서 osDelay를 써도 MotorTask만 기다리므로 TCP는 계속 동작한다.
               */
              if (Motor_IsEStop() != 0U)
              {
                  motor_state.running = 0U;
                  motor_state.error = 1U;
                  break;
              }

              if (Motor_HomeOk() == 0U)
              {
                  motor_state.running = 0U;
                  motor_state.error = 1U;
                  break;
              }

              motor_state.running = 1U;
              motor_state.error = 0U;

              if (cmd.start_ms > 0U)
              {
                  osDelay(cmd.start_ms);
              }

              st = Motor_Start(
                  &huart5,
                  cmd.pos,
                  (uint8_t)cmd.speed,
                  (uint16_t)cmd.acc_ms
              );

              if (st != HAL_OK)
              {
                  motor_state.running = 0U;
                  motor_state.error = 1U;
                  break;
              }

              if (motor_state.move_ms > 0U)
              {
                  if (Motor_WaitMove(motor_state.move_ms) != 0U)
                  {
                      /*
                       * 이동 중 정지 또는 비상정지가 처리된 상태.
                       * Motor_Done()을 호출하지 않는다.
                       */
                      break;
                  }
              }

              st = Motor_Done(&huart5);

              if (st != HAL_OK)
              {
                  motor_state.running = 0U;
                  motor_state.error = 1U;
                  break;
              }

              if (cmd.wait_ms > 0U)
              {
                  osDelay(cmd.wait_ms);
              }

              motor_state.running = 0U;
              motor_state.error = 0U;
              break;

          case MOTOR_CMD_STOP:
              (void)Motor_Stop(&huart5);
              motor_state.running = 0U;
              break;

          case MOTOR_CMD_ESTOP:
              (void)Motor_EStop(&huart5);
              motor_state.running = 0U;
              motor_state.error = 1U;
              break;

          case MOTOR_CMD_RELEASE:
              st = Motor_Release(&huart5);

              if (st == HAL_OK)
              {
                  motor_state.running = 0U;
                  motor_state.error = 0U;
              }
              else
              {
                  motor_state.running = 0U;
                  motor_state.error = 1U;
              }
              break;

          default:
              break;
      }
  }

  /* USER CODE END StartMotorTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
/* USER CODE END Application */

