#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>

/*
 * motor.h
 *
 * 역할:
 *   모터 제어 함수, 모터 상태, FreeRTOS Queue 명령 구조체를 정의한다.
 *
 * 현재 구조:
 *   TCP 명령
 *   -> motion_protocol.c
 *   -> MotorQueue
 *   -> StartMotorTask()
 *   -> motor.c
 *   -> motor_bus.c
 *   -> UART5 / RS485
 *   -> AIMotor
 */

#define MOTOR_ID                    1U

#define MOTOR_UNIT_PER_TURN         1000L
#define LIFT_MAX_RPM                1000U
#define LIFT_MIN_PERCENT            10U
#define LIFT_MAX_PERCENT            100U

/*
 * PCB DI 출력 극성
 *   RESET = DI ON
 *   SET   = DI OFF
 */
#define MOTOR_GPIO_ON               GPIO_PIN_RESET
#define MOTOR_GPIO_OFF              GPIO_PIN_SET

#define MOTOR_ALARM_RESET_MS        500U

/*
 * MotorQueue에 넣는 명령 종류
 */
typedef enum
{
    MOTOR_CMD_NONE = 0,
    MOTOR_CMD_MOVE = 1,
    MOTOR_CMD_STOP = 2,
    MOTOR_CMD_ESTOP = 3,
    MOTOR_CMD_RELEASE = 4
} MotorCmdId_t;

/*
 * MotorQueue에 들어가는 명령 데이터
 *
 * IOC에서 MotorQueue Item Size는 반드시 sizeof(MotorCommand_t)로 설정한다.
 */
typedef struct
{
    uint32_t id;         /* MOTOR_CMD_MOVE, MOTOR_CMD_STOP 등 */
    int32_t pos;         /* 목표 위치, motor unit */
    uint32_t speed;      /* 속도 percent */
    uint32_t acc_ms;     /* 가감속 시간 ms */
    uint32_t start_ms;   /* 이동 전 대기 ms */
    uint32_t wait_ms;    /* 이동 후 대기 ms */
} MotorCommand_t;

/*
 * 통신 디버그 정보
 */
typedef struct
{
    uint32_t uart_error;

    uint16_t crc_calc;
    uint16_t crc_recv;
    uint8_t crc_ok;

    uint16_t last_reg;
    uint16_t exception_code;

    int32_t last_pos;
} MotorDebug_t;

extern MotorDebug_t motor_debug;

/*
 * STM 내부 모터 상태
 */
typedef struct
{
    uint8_t setup_done;
    uint8_t home_done;
    uint8_t running;
    uint8_t error;

    uint8_t enable_on;
    uint8_t estop_on;
    uint8_t servo_on;

    int32_t home_offset;
    int32_t cur_pos;
    int32_t last_target;

    uint8_t last_speed;
    uint16_t last_rpm;
    uint16_t last_acc_ms;
    uint32_t move_ms;
    uint32_t seq;

    HAL_StatusTypeDef last_hal;
} MotorState_t;

extern MotorState_t motor_state;

/*
 * IOC에서 생성한 Queue handle
 */
extern osMessageQueueId_t MotorQueueHandle;

/* ---------------------------------------------------------
 * motor.c 함수
 * --------------------------------------------------------- */
void Motor_InitIO(void);

HAL_StatusTypeDef Motor_Setup(UART_HandleTypeDef *huart);

HAL_StatusTypeDef Motor_Start(
    UART_HandleTypeDef *huart,
    int32_t pos,
    uint8_t speed,
    uint16_t acc_ms
);

HAL_StatusTypeDef Motor_Done(UART_HandleTypeDef *huart);
HAL_StatusTypeDef Motor_Stop(UART_HandleTypeDef *huart);

HAL_StatusTypeDef Motor_EStop(UART_HandleTypeDef *huart);
HAL_StatusTypeDef Motor_Release(UART_HandleTypeDef *huart);

HAL_StatusTypeDef Motor_SetHome(void);
HAL_StatusTypeDef Motor_ClearHome(void);

uint8_t Motor_HomeOk(void);
uint8_t Motor_IsEStop(void);
uint8_t Motor_IsBusy(void);

/* ---------------------------------------------------------
 * freertos.c에서 구현하는 Queue 전송 함수
 * --------------------------------------------------------- */
osStatus_t Motor_SendCmd(const MotorCommand_t *cmd);

osStatus_t Motor_SendMove(
    int32_t pos,
    uint32_t speed,
    uint32_t acc_ms,
    uint32_t start_ms,
    uint32_t wait_ms
);

osStatus_t Motor_SendStop(void);
osStatus_t Motor_SendEStop(void);
osStatus_t Motor_SendRelease(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
