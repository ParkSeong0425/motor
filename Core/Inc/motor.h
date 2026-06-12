#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>

/* -------------------------------------------------
 * Motor mechanical setting
 * ------------------------------------------------- */
#define MOTOR_ID                 1

/* 모터 1회전당 위치 unit 수 */
#define MOTOR_UNIT_PER_TURN      1000

/* 바퀴 지름. 500 = 50.0mm */
#define WHEEL_DIA_MM_X10         500

/* 감속비. 모터 1바퀴 = 바퀴 1바퀴면 1 / 1 */
#define DRIVE_RATIO_NUM          1
#define DRIVE_RATIO_DEN          1

/* speed 100%일 때 rpm */
#define LIFT_MAX_RPM             1000

/* 너무 낮거나 높은 speed 입력 제한 */
#define LIFT_MIN_PERCENT         10
#define LIFT_MAX_PERCENT         100

/*
 * 현재 PCB DI 출력 극성.
 * RESET = DI ON
 * SET   = DI OFF
 */
#define MOTOR_GPIO_ON            GPIO_PIN_RESET
#define MOTOR_GPIO_OFF           GPIO_PIN_SET

typedef enum
{
    MOTOR_CMD_NONE = 0,
    MOTOR_CMD_MOVE = 1,
    MOTOR_CMD_STOP = 2,
    MOTOR_CMD_ESTOP = 3,
    MOTOR_CMD_RELEASE = 4
} MotorCommandId_t;

typedef struct
{
    uint32_t id;

    /* 모터 목표 위치 */
    int32_t target;

    /* 속도 percent, 0~100 */
    uint32_t speed;

    /* 가감속 시간 */
    uint32_t acc_ms;

    /* 이동 시작 전 대기 시간 */
    uint32_t start_ms;

    /* 이동 완료 후 대기 시간 */
    uint32_t wait_ms;
} MotorCommand_t;

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

typedef struct
{
    uint8_t setup_ok;
    uint8_t moving;
    uint8_t error;
    uint8_t estop;
    uint8_t enable;

    int32_t pos;
    int32_t target;

    uint8_t speed;
    uint16_t rpm;
    uint16_t acc_ms;

    uint32_t seq;

    HAL_StatusTypeDef last_result;
} MotorState_t;

extern MotorDebug_t motor_debug;
extern MotorState_t motor_state;
extern osMessageQueueId_t MotorQueueHandle;

void Motor_TaskRun(void *argument);

void Motor_InitIO(void);
HAL_StatusTypeDef Motor_Setup(UART_HandleTypeDef *huart);

HAL_StatusTypeDef Motor_Start(UART_HandleTypeDef *huart,
                              int32_t target,
                              uint8_t speed,
                              uint16_t acc_ms);

HAL_StatusTypeDef Motor_Stop(UART_HandleTypeDef *huart);
HAL_StatusTypeDef Motor_EStop(UART_HandleTypeDef *huart);
HAL_StatusTypeDef Motor_Release(UART_HandleTypeDef *huart);

HAL_StatusTypeDef Motor_ReadPos(UART_HandleTypeDef *huart, int32_t *pos);

uint8_t Motor_IsBusy(void);
uint8_t Motor_IsEStop(void);

int32_t Motor_mmToUnit(int32_t mm);

osStatus_t Motor_SendMove(int32_t target,
                          uint32_t speed,
                          uint32_t acc_ms,
                          uint32_t start_ms,
                          uint32_t wait_ms);

osStatus_t Motor_SendStop(void);
osStatus_t Motor_SendEStop(void);
osStatus_t Motor_SendRelease(void);

#ifdef __cplusplus
}
#endif

#endif
