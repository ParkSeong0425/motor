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

#define TCP_CMD_SOCKET_NUM           0U
#define TCP_CMD_PORT                 5000U
#define TCP_CMD_IP_MODE              TCP_CMD_MODE_IPV4

/*
 * SafetyTask 사용 여부.
 *
 * 0U:
 *   물리 긴급버튼 Task를 만들지 않는다.
 *   TCP 01S 긴급정지는 그대로 동작한다.
 *
 * 1U:
 *   물리 긴급버튼 GPIO를 SafetyTask에서 계속 감시한다.
 */
#define CREATE_SAFETY_TASK           0U
#define USE_ESTOP_BTN                0U

/*
 * DI2 센서 사용 여부.
 *
 * 현재 목표:
 *   일반 이동 01MO / 01MI 중에는 DI2 센서를 무시한다.
 *   오직 01H 원점 탐색 중에만 DI2 센서를 본다.
 */
#define USE_DI2_HOME_SENSOR          1U

/*
 * DI2 센서 입력 극성.
 *
 * 손 감지 시 PE6이 RESET이면 GPIO_PIN_RESET 유지.
 * 손 감지 시 PE6이 SET이면 GPIO_PIN_SET으로 변경한다.
 */
#define DI2_SENSOR_ON_LEVEL          GPIO_PIN_RESET

/*
 * MotorTask 대기 시간 설정.
 */
#define MOTOR_WAIT_STEP_MS           10U
#define MOTOR_READ_PERIOD_MS         50U
#define MOTOR_MIN_DONE_MS            300U
#define MOTOR_MAX_TIMEOUT_MS         120000U

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

static uint8_t Di2On(void);
static uint32_t MakeTimeout(uint32_t guess_ms);
static uint8_t Motor_WaitMove(uint32_t guess_ms);
static uint8_t Motor_WaitHome(uint32_t guess_ms);

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
void MX_FREERTOS_Init(void)
{
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

  /* Create the queue(s) */
  /* creation of MotorQueue */
  MotorQueueHandle = osMessageQueueNew(8, sizeof(MotorCommand_t), &MotorQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of TcpTask */
  TcpTaskHandle = osThreadNew(StartTcpTask, NULL, &TcpTask_attributes);

  /* creation of SafetyTask */
#if (CREATE_SAFETY_TASK != 0U)
  SafetyTaskHandle = osThreadNew(StartSafetyTask, NULL, &SafetyTask_attributes);
#endif

  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
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

  /*
   * defaultTask는 현재 특별한 작업을 하지 않는다.
   * 나중에 상태 LED, 주기적인 시스템 체크 등을 넣을 수 있다.
   */
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

  /*
   * TCP 명령 처리 Task.
   *
   * 중요:
   *   모터 이동 완료까지 여기서 기다리면 안 된다.
   *   TCP Task는 계속 살아 있어야 이동 중에도 01S 긴급정지를 받을 수 있다.
   */
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

  /*
   * 물리 긴급버튼 감시 Task.
   *
   * 현재는 CREATE_SAFETY_TASK가 0U라서 생성되지 않는다.
   * 나중에 실제 버튼을 사용할 때 CREATE_SAFETY_TASK와 USE_ESTOP_BTN을 1U로 켠다.
   */
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
* @brief Helper functions for MotorTask.
* @note  DI2 센서는 일반 이동에서는 사용하지 않고, 01H 원점 탐색에서만 사용한다.
*/
/* USER CODE END Header_StartMotorTask */

/* USER CODE BEGIN MotorTask_Helper */

/*
 * DI2 광센서 입력 확인.
 *
 * 현재 사용 목적:
 *   01H 원점 탐색 중에만 사용한다.
 *
 * 일반 01MO / 01MI 이동 중에는 이 함수를 호출하지 않는다.
 */
static uint8_t Di2On(void)
{
#if (USE_DI2_HOME_SENSOR != 0U)
    if (HAL_GPIO_ReadPin(DI_2_GPIO_Port, DI_2_Pin) == DI2_SENSOR_ON_LEVEL)
    {
        return 1U;
    }
#endif

    return 0U;
}

/*
 * 이동 timeout 계산.
 *
 * 완료 판단은 RS485 위치 편차값으로 한다.
 * 이 timeout은 통신 문제, 센서 미검출, 기구 문제로 무한 대기하는 것을 막기 위한 안전 시간이다.
 */
static uint32_t MakeTimeout(uint32_t guess_ms)
{
    uint32_t timeout;

    timeout = (guess_ms * 3U) + 3000U;

    if (timeout < 5000U)
    {
        timeout = 5000U;
    }

    if (timeout > MOTOR_MAX_TIMEOUT_MS)
    {
        timeout = MOTOR_MAX_TIMEOUT_MS;
    }

    return timeout;
}

/*
 * 일반 이동 대기.
 *
 * 사용 대상:
 *   01MO
 *   01MI
 *
 * 동작:
 *   1. DI2 센서는 보지 않는다.
 *   2. 이동 중 01S / STOP 명령은 즉시 처리한다.
 *   3. RS485로 모터 위치 완료 여부를 확인한다.
 *   4. 완료되면 Motor_Done()으로 넘어간다.
 *
 * 중요:
 *   일반 이동 중 손이 센서에 닿아도 모터는 멈추면 안 된다.
 *   DI2는 오직 01H 원점 탐색 중에만 본다.
 */
static uint8_t Motor_WaitMove(uint32_t guess_ms)
{
    uint32_t elapsed;
    uint32_t read_tick;
    uint32_t timeout;
    MotorCommand_t cmd;
    uint8_t done_cnt;

    elapsed = 0U;
    read_tick = 0U;
    done_cnt = 0U;
    timeout = MakeTimeout(guess_ms);

    for (;;)
    {
        /*
         * 이동 중 새 명령 확인.
         * 이동 중에는 01S / STOP만 즉시 처리한다.
         */
        if (osMessageQueueGet(MotorQueueHandle, &cmd, NULL, 0U) == osOK)
        {
            if (cmd.id == MOTOR_CMD_ESTOP)
            {
                (void)Motor_EStop(&huart5);
                motor_state.running = 0U;
                motor_state.error = 1U;
                return 1U;
            }

            if (cmd.id == MOTOR_CMD_STOP)
            {
                (void)Motor_Stop(&huart5);
                motor_state.running = 0U;
                return 1U;
            }

            /*
             * 이동 중 들어온 MOVE / HOME / RELEASE는 여기서 실행하지 않는다.
             * 중복 이동 방지를 위해 무시한다.
             */
        }

        /*
         * 너무 빠른 완료 오판을 막기 위해
         * 이동 시작 후 MOTOR_MIN_DONE_MS 이후부터 완료 상태를 본다.
         */
        if (elapsed >= MOTOR_MIN_DONE_MS)
        {
            read_tick += MOTOR_WAIT_STEP_MS;

            if (read_tick >= MOTOR_READ_PERIOD_MS)
            {
                read_tick = 0U;

                /*
                 * RS485로 위치 완료 확인.
                 * 3번 연속 완료일 때 진짜 완료로 판단한다.
                 */
                if (Motor_CheckDone(&huart5) != 0U)
                {
                    done_cnt++;

                    if (done_cnt >= 3U)
                    {
                        return 0U;
                    }
                }
                else
                {
                    done_cnt = 0U;
                }
            }
        }

        /*
         * 완료가 너무 오래 안 되면 정지 후 에러 처리.
         */
        if (elapsed >= timeout)
        {
            (void)Motor_Stop(&huart5);
            motor_state.running = 0U;
            motor_state.error = 1U;
            return 1U;
        }

        elapsed += MOTOR_WAIT_STEP_MS;
        osDelay(MOTOR_WAIT_STEP_MS);
    }
}

/*
 * 01H 원점 탐색 대기.
 *
 * 사용 대상:
 *   01H
 *
 * 동작:
 *   1. Motor_StartHome()으로 MI 방향 저속 이동 시작
 *   2. DI2 센서가 감지되면 즉시 정지
 *   3. 그 위치를 새로운 원점으로 저장
 *   4. 이후 01MO / 01MI는 이 원점을 기준으로 움직임
 *
 * 일반 이동과 다르게 여기서는 DI2 센서를 반드시 본다.
 */
static uint8_t Motor_WaitHome(uint32_t guess_ms)
{
    uint32_t elapsed;
    uint32_t timeout;
    MotorCommand_t cmd;

    elapsed = 0U;
    timeout = MakeTimeout(guess_ms);

    for (;;)
    {
        /*
         * 01H 중 DI2 센서 감지.
         * 감지되면 이 위치를 원점으로 저장하고 종료한다.
         */
        if (Di2On() != 0U)
        {
            if (Motor_SaveHomeHere(&huart5) == HAL_OK)
            {
                motor_state.running = 0U;
                motor_state.error = 0U;
                return 0U;
            }

            motor_state.running = 0U;
            motor_state.error = 1U;
            return 1U;
        }

        /*
         * 원점 탐색 중에도 01S / STOP은 즉시 처리한다.
         */
        if (osMessageQueueGet(MotorQueueHandle, &cmd, NULL, 0U) == osOK)
        {
            if (cmd.id == MOTOR_CMD_ESTOP)
            {
                (void)Motor_EStop(&huart5);
                motor_state.running = 0U;
                motor_state.error = 1U;
                return 1U;
            }

            if (cmd.id == MOTOR_CMD_STOP)
            {
                (void)Motor_Stop(&huart5);
                motor_state.running = 0U;
                return 1U;
            }

            /*
             * 원점 탐색 중 다른 이동 명령은 실행하지 않는다.
             */
        }

        /*
         * 센서를 못 찾으면 정지 후 에러.
         */
        if (elapsed >= timeout)
        {
            (void)Motor_Stop(&huart5);
            motor_state.running = 0U;
            motor_state.error = 1U;
            return 1U;
        }

        elapsed += MOTOR_WAIT_STEP_MS;
        osDelay(MOTOR_WAIT_STEP_MS);
    }
}

/* USER CODE END MotorTask_Helper */

/* USER CODE BEGIN Header_StartMotorTaskFunction */
/**
* @brief Function implementing the MotorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorTaskFunction */
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

  /*
   * MotorTask main loop.
   *
   * Queue에 명령이 들어올 때까지 MotorTask만 대기한다.
   * TCP Task는 계속 동작하므로 네트워크는 막히지 않는다.
   */
  for (;;)
  {
      if (osMessageQueueGet(MotorQueueHandle, &cmd, NULL, osWaitForever) != osOK)
      {
          osDelay(10);
          continue;
      }

      switch (cmd.id)
      {
          case MOTOR_CMD_MOVE:
              /*
               * 일반 이동 명령.
               *
               * 01MO / 01MI가 여기로 들어온다.
               * 일반 이동 중에는 DI2 센서를 보지 않는다.
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

              if (Motor_WaitMove(motor_state.move_ms) != 0U)
              {
                  /*
                   * 이동 중 정지 또는 비상정지가 처리된 상태.
                   * Motor_Done()을 호출하지 않는다.
                   */
                  break;
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

          case MOTOR_CMD_HOME:
              /*
               * 01H 원점 탐색.
               *
               * MI 방향으로 천천히 움직이다가 DI2 센서가 감지되면
               * 그 위치를 새로운 원점으로 저장한다.
               */
              if (Motor_IsEStop() != 0U)
              {
                  motor_state.running = 0U;
                  motor_state.error = 1U;
                  break;
              }

              motor_state.running = 1U;
              motor_state.error = 0U;

              st = Motor_StartHome(&huart5);

              if (st != HAL_OK)
              {
                  motor_state.running = 0U;
                  motor_state.error = 1U;
                  break;
              }

              if (Motor_WaitHome(motor_state.move_ms) != 0U)
              {
                  /*
                   * 원점 탐색 중 정지, 비상정지, 센서 미검출 timeout.
                   */
                  break;
              }

              motor_state.running = 0U;
              motor_state.error = 0U;
              break;

          case MOTOR_CMD_STOP:
              /*
               * 일반 정지.
               */
              (void)Motor_Stop(&huart5);
              motor_state.running = 0U;
              break;

          case MOTOR_CMD_ESTOP:
              /*
               * 비상정지.
               */
              (void)Motor_EStop(&huart5);
              motor_state.running = 0U;
              motor_state.error = 1U;
              break;

          case MOTOR_CMD_RELEASE:
              /*
               * 비상정지 해제.
               */
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
