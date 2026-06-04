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
 * TCP 명령
 * -> motion_protocol.c
 * -> MotorQueue
 * -> StartMotorTask()
 * -> motor.c
 * -> motor_bus.c
 * -> UART5 / RS485
 * -> AIMotor
 */
#define MOTOR_ID                    1U

#define MOTOR_UNIT_PER_TURN         1000L

/*
 * 바퀴 지름 설정.
 *
 * 기계 도면에 있는 바퀴 파이 값을 여기에 넣는다.
 * 단위는 0.1mm이다.
 *
 * 예:
 *   50.0mm 바퀴  (50파이)  = 500
 *   65.0mm 바퀴  (65파이)  = 650
 *   100.0mm 바퀴 (100파이) = 1000
 */
#define WHEEL_DIA_MM_X10            500L

/*
 * 감속비 / 풀리비 보정.
 *
 * 모터 1바퀴 = 바퀴 1바퀴면 1 / 1 유지.
 *
 * 예:
 *   모터 2바퀴 돌 때 바퀴 1바퀴면
 *   DRIVE_RATIO_NUM = 2
 *   DRIVE_RATIO_DEN = 1
 */
#define DRIVE_RATIO_NUM             1L
#define DRIVE_RATIO_DEN             1L

#define LIFT_MAX_RPM                1000U
#define LIFT_MIN_PERCENT            10U
#define LIFT_MAX_PERCENT            100U

/*
 * PCB DI 출력 극성
 * RESET = DI ON
 * SET   = DI OFF
 */
#define MOTOR_GPIO_ON               GPIO_PIN_RESET
#define MOTOR_GPIO_OFF              GPIO_PIN_SET

typedef enum
{
    MOTOR_CMD_NONE = 0,
    MOTOR_CMD_MOVE = 1,
    MOTOR_CMD_STOP = 2,
    MOTOR_CMD_ESTOP = 3,
    MOTOR_CMD_RELEASE = 4,
	MOTOR_CMD_HOME = 5
} MotorCmdId_t;

typedef struct
{
    uint32_t id;
    int32_t pos;
    uint32_t speed;
    uint32_t acc_ms;
    uint32_t start_ms;
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

extern MotorDebug_t motor_debug;

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
extern osMessageQueueId_t MotorQueueHandle;

/* motor.c */
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

HAL_StatusTypeDef Motor_ReadPos(UART_HandleTypeDef *huart, int32_t *pos);
HAL_StatusTypeDef Motor_ReadDiff(UART_HandleTypeDef *huart, int32_t *diff);
uint8_t Motor_CheckDone(UART_HandleTypeDef *huart);

HAL_StatusTypeDef Motor_StartHome(UART_HandleTypeDef *huart);
HAL_StatusTypeDef Motor_SaveHomeHere(UART_HandleTypeDef *huart);

uint8_t Motor_HomeOk(void);
uint8_t Motor_IsEStop(void);
uint8_t Motor_IsBusy(void);
int32_t Motor_mmToUnit(int32_t mm);

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
osStatus_t Motor_SendHome(void);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_H */
