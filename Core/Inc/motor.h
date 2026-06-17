#ifndef MOTOR_H
#define MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os.h"
#include <stdint.h>

#define MOTOR_ID             1
#define MOTOR_UNIT_PER_TURN  1000
#define WHEEL_DIA_MM         100
#define DRIVE_RATIO_NUM      1
#define DRIVE_RATIO_DEN      1
#define LIFT_MAX_RPM         1000
#define LIFT_MIN_PERCENT     10
#define LIFT_MAX_PERCENT     100

#define MOTOR_GPIO_ON        GPIO_PIN_RESET
#define MOTOR_GPIO_OFF       GPIO_PIN_SET

typedef enum
{
    MOTOR_CMD_NONE = 0,
    MOTOR_CMD_MOVE = 1,
    MOTOR_CMD_STOP = 2,
} MotorCommandId_t;

typedef struct
{
    uint32_t id;
    int32_t target;
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

typedef struct
{
    uint8_t setup_ok;
    uint8_t moving;
    uint8_t error;
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
void Motor_ClearEStop(void);
HAL_StatusTypeDef Motor_Setup(UART_HandleTypeDef *huart);
HAL_StatusTypeDef Motor_Power(UART_HandleTypeDef *huart, uint8_t on);
HAL_StatusTypeDef Motor_Home(UART_HandleTypeDef *huart);
HAL_StatusTypeDef Motor_Start(UART_HandleTypeDef *huart,
                              int32_t target,
                              uint8_t speed,
                              uint16_t acc_ms);
HAL_StatusTypeDef Motor_Stop(UART_HandleTypeDef *huart);
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

#ifdef __cplusplus
}
#endif

#endif
