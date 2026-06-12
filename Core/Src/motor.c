#include "motor.h"
#include "rs485.h"
#include "usart.h"
#include "cmsis_os.h"
#include <stdint.h>

#define R_MODE      0x0200
#define R_DI1_FUNC  0x0302
#define R_START     0x0305
#define R_DI3_FUNC  0x0306
#define R_DI3_VAL   0x0307
#define R_SRC       0x0500
#define R_RUN       0x1100
#define R_END       0x1101
#define R_TYPE      0x1104
#define R_POS       0x110C
#define R_SPEED     0x110E
#define R_ACC       0x110F
#define R_WAIT      0x1110
#define R_REAL      0x0B07

#define V_POS       1
#define V_SERVO     1
#define V_ESTOP     34
#define V_SRC       2
#define V_RUN       0
#define V_END       1
#define V_ABS       1

#define GAP         5
#define MOVE_MS     60000
#define STABLE_MS   1000

#define DI3_ON()    HAL_GPIO_WritePin(DI_3_GPIO_Port, DI_3_Pin, MOTOR_GPIO_ON)
#define DI3_OFF()   HAL_GPIO_WritePin(DI_3_GPIO_Port, DI_3_Pin, MOTOR_GPIO_OFF)

typedef struct
{
    uint16_t reg;
    uint16_t val;
} Pair_t;

MotorDebug_t motor_debug;
MotorState_t motor_state;

static uint8_t lock = 0;

static void Clear(void)
{
    motor_state.setup_ok = 0;
    motor_state.moving = 0;
    motor_state.error = 0;
    motor_state.estop = 0;
    motor_state.enable = 0;
    motor_state.pos = 0;
    motor_state.target = 0;
    motor_state.speed = 0;
    motor_state.rpm = 0;
    motor_state.acc_ms = 0;
    motor_state.seq = 0;
    motor_state.last_result = HAL_OK;

    motor_debug.uart_error = 0;
    motor_debug.crc_calc = 0;
    motor_debug.crc_recv = 0;
    motor_debug.crc_ok = 0;
    motor_debug.last_reg = 0;
    motor_debug.exception_code = 0;
    motor_debug.last_pos = 0;

    lock = 0;
}

static HAL_StatusTypeDef Fail(HAL_StatusTypeDef result)
{
    motor_state.moving = 0;
    motor_state.error = 1;
    motor_state.last_result = result;
    return result;
}

static uint16_t Rpm(uint8_t percent)
{
    if (percent < LIFT_MIN_PERCENT)
    {
        percent = LIFT_MIN_PERCENT;
    }

    if (percent > LIFT_MAX_PERCENT)
    {
        percent = LIFT_MAX_PERCENT;
    }

    return (uint16_t)(((uint32_t)LIFT_MAX_RPM * percent) / 100);
}

int32_t Motor_mmToUnit(int32_t mm)
{
    int64_t sign = 1;
    int64_t value;
    int64_t cir;
    int64_t unit;

    if (mm < 0)
    {
        sign = -1;
        value = -(int64_t)mm;
    }
    else
    {
        value = mm;
    }

    cir = ((int64_t)WHEEL_DIA_MM_X10 * 31416) / 10000;

    if (cir <= 0)
    {
        return 0;
    }

    unit = (value * 10 *
            (int64_t)MOTOR_UNIT_PER_TURN *
            (int64_t)DRIVE_RATIO_NUM)
           /
           (cir * (int64_t)DRIVE_RATIO_DEN);

    return (int32_t)(unit * sign);
}

void Motor_InitIO(void)
{
    DI3_OFF();
    Bus_Rx();
    Clear();
}

static HAL_StatusTypeDef W16(UART_HandleTypeDef *huart,
                             uint16_t reg,
                             uint16_t val)
{
    HAL_StatusTypeDef result;

    result = Bus_Write16(huart, reg, val);
    motor_state.last_result = result;
    HAL_Delay(10);

    return result;
}

static HAL_StatusTypeDef W32(UART_HandleTypeDef *huart,
                             uint16_t reg,
                             int32_t val)
{
    HAL_StatusTypeDef result;

    result = Bus_Write32(huart, reg, val);
    motor_state.last_result = result;
    HAL_Delay(10);

    return result;
}

static HAL_StatusTypeDef Off(UART_HandleTypeDef *huart)
{
    return W16(huart, R_START, 0);
}

static HAL_StatusTypeDef On(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef result;

    result = W16(huart, R_START, 0);

    if (result == HAL_OK)
    {
        HAL_Delay(20);
        result = W16(huart, R_START, 1);
    }

    return result;
}

static HAL_StatusTypeDef Stop(UART_HandleTypeDef *huart, uint8_t estop)
{
    HAL_StatusTypeDef result = HAL_OK;
    int32_t pos = 0;

    if (estop != 0)
    {
        DI3_ON();
        lock = 1;
        motor_state.estop = 1;
        motor_state.error = 1;
    }

    if (huart != NULL)
    {
        result = Off(huart);
        (void)Motor_ReadPos(huart, &pos);
    }

    motor_state.moving = 0;
    motor_state.last_result = result;

    return result;
}

HAL_StatusTypeDef Motor_Setup(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef result = HAL_OK;

    const Pair_t list[] =
    {
        { R_MODE,     V_POS   },
        { R_DI1_FUNC, V_SERVO },
        { R_START,    0       },
        { R_DI3_FUNC, V_ESTOP },
        { R_DI3_VAL,  0       },
        { R_SRC,      V_SRC   },
        { R_RUN,      V_RUN   },
        { R_END,      V_END   },
        { R_WAIT,     0       }
    };

    if (huart == NULL)
    {
        return Fail(HAL_ERROR);
    }

    Bus_Rx();
    HAL_Delay(50);

    for (uint32_t i = 0; i < (sizeof(list) / sizeof(list[0])); i++)
    {
        result = W16(huart, list[i].reg, list[i].val);

        if (result != HAL_OK)
        {
            motor_state.setup_ok = 0;
            return Fail(result);
        }
    }

    DI3_OFF();

    motor_state.setup_ok = 1;
    motor_state.enable = 1;
    motor_state.estop = 0;
    motor_state.error = 0;
    motor_state.last_result = HAL_OK;

    return HAL_OK;
}

static HAL_StatusTypeDef WriteMove(UART_HandleTypeDef *huart,
                                   int32_t target,
                                   uint16_t rpm,
                                   uint16_t acc_ms)
{
    HAL_StatusTypeDef result;

    result = Off(huart);

    if (result == HAL_OK)
    {
        HAL_Delay(50);
        result = W16(huart, R_TYPE, V_ABS);
    }

    if (result == HAL_OK)
    {
        result = W32(huart, R_POS, target);
    }

    if (result == HAL_OK)
    {
        result = W16(huart, R_SPEED, rpm);
    }

    if (result == HAL_OK)
    {
        result = W16(huart, R_ACC, acc_ms);
    }

    if (result == HAL_OK)
    {
        result = W16(huart, R_WAIT, 0);
    }

    return result;
}

HAL_StatusTypeDef Motor_Start(UART_HandleTypeDef *huart,
                              int32_t target,
                              uint8_t speed,
                              uint16_t acc_ms)
{
    HAL_StatusTypeDef result;
    uint16_t rpm;

    if (huart == NULL || lock != 0)
    {
        return Fail(HAL_ERROR);
    }

    rpm = Rpm(speed);

    if (motor_state.setup_ok == 0)
    {
        result = Motor_Setup(huart);
    }
    else
    {
        result = HAL_OK;
    }

    if (result == HAL_OK)
    {
        result = WriteMove(huart, target, rpm, acc_ms);
    }

    if (result == HAL_OK)
    {
        result = On(huart);
    }

    if (result != HAL_OK)
    {
        return Fail(result);
    }

    motor_state.target = target;
    motor_state.speed = speed;
    motor_state.rpm = rpm;
    motor_state.acc_ms = acc_ms;
    motor_state.moving = 1;
    motor_state.error = 0;
    motor_state.last_result = HAL_OK;

    return HAL_OK;
}

HAL_StatusTypeDef Motor_ReadPos(UART_HandleTypeDef *huart, int32_t *pos)
{
    HAL_StatusTypeDef result;

    if (huart == NULL || pos == NULL)
    {
        motor_state.last_result = HAL_ERROR;
        return HAL_ERROR;
    }

    result = Bus_Read32(huart, R_REAL, pos);

    if (result == HAL_OK)
    {
        motor_state.pos = *pos;
        motor_debug.last_pos = *pos;
    }

    motor_state.last_result = result;

    return result;
}

HAL_StatusTypeDef Motor_Stop(UART_HandleTypeDef *huart)
{
    return Stop(huart, 0);
}

HAL_StatusTypeDef Motor_EStop(UART_HandleTypeDef *huart)
{
    return Stop(huart, 1);
}

HAL_StatusTypeDef Motor_Release(UART_HandleTypeDef *huart)
{
    (void)huart;

    DI3_OFF();
    HAL_Delay(300);

    lock = 0;
    motor_state.estop = 0;
    motor_state.moving = 0;
    motor_state.error = 0;
    motor_state.last_result = HAL_OK;

    return HAL_OK;
}

uint8_t Motor_IsBusy(void)
{
    return motor_state.moving;
}

uint8_t Motor_IsEStop(void)
{
    return lock;
}

static osStatus_t Put(uint32_t id,
                      int32_t target,
                      uint32_t speed,
                      uint32_t acc_ms,
                      uint32_t start_ms,
                      uint32_t wait_ms)
{
    MotorCommand_t cmd;

    if (MotorQueueHandle == NULL)
    {
        return osErrorResource;
    }

    cmd.id = id;
    cmd.target = target;
    cmd.speed = speed;
    cmd.acc_ms = acc_ms;
    cmd.start_ms = start_ms;
    cmd.wait_ms = wait_ms;

    return osMessageQueuePut(MotorQueueHandle, &cmd, 0, 0);
}

osStatus_t Motor_SendMove(int32_t target,
                          uint32_t speed,
                          uint32_t acc_ms,
                          uint32_t start_ms,
                          uint32_t wait_ms)
{
    if (MotorQueueHandle == NULL ||
        motor_state.moving != 0 ||
        motor_state.estop != 0 ||
        osMessageQueueGetCount(MotorQueueHandle) > 0)
    {
        return osErrorResource;
    }

    if (speed > LIFT_MAX_PERCENT)
    {
        speed = LIFT_MAX_PERCENT;
    }

    if (acc_ms > 60000)
    {
        acc_ms = 60000;
    }

    return Put(MOTOR_CMD_MOVE, target, speed, acc_ms, start_ms, wait_ms);
}

osStatus_t Motor_SendStop(void)
{
    return Put(MOTOR_CMD_STOP, 0, 0, 0, 0, 0);
}

osStatus_t Motor_SendEStop(void)
{
    return Put(MOTOR_CMD_ESTOP, 0, 0, 0, 0, 0);
}

osStatus_t Motor_SendRelease(void)
{
    return Put(MOTOR_CMD_RELEASE, 0, 0, 0, 0, 0);
}

static uint8_t CheckCmd(void)
{
    MotorCommand_t cmd;

    if (osMessageQueueGet(MotorQueueHandle, &cmd, NULL, 0) != osOK)
    {
        return 0;
    }

    if (cmd.id == MOTOR_CMD_STOP)
    {
        (void)Motor_Stop(&huart5);
        return 1;
    }

    if (cmd.id == MOTOR_CMD_ESTOP)
    {
        (void)Motor_EStop(&huart5);
        return 1;
    }

    if (cmd.id == MOTOR_CMD_RELEASE)
    {
        (void)Motor_Release(&huart5);
        return 1;
    }

    return 0;
}

static void WaitDone(void)
{
    int32_t last = 0;
    int32_t now = 0;
    int32_t gap;
    uint32_t time = 0;
    uint32_t still = 0;
    uint8_t moved = 0;

    osDelay(300);
    time = 300;

    if (Motor_ReadPos(&huart5, &last) != HAL_OK)
    {
        last = motor_state.pos;
    }

    while (time < MOVE_MS)
    {
        if (CheckCmd() != 0)
        {
            return;
        }

        if (Motor_ReadPos(&huart5, &now) == HAL_OK)
        {
            gap = now - motor_state.target;

            if (gap < 0)
            {
                gap = -gap;
            }

            if (gap <= GAP)
            {
                (void)Motor_Stop(&huart5);
                motor_state.error = 0;
                motor_state.seq++;
                return;
            }

            gap = now - last;

            if (gap < 0)
            {
                gap = -gap;
            }

            if (gap > GAP)
            {
                moved = 1;
                still = 0;
                last = now;
            }
            else if (moved != 0)
            {
                still += 100;
            }

            if (moved != 0 && still >= STABLE_MS)
            {
                (void)Motor_Stop(&huart5);
                motor_state.error = 0;
                motor_state.seq++;
                return;
            }
        }

        osDelay(100);
        time += 100;
    }

    (void)Motor_Stop(&huart5);
    motor_state.error = 1;
}

void Motor_TaskRun(void *argument)
{
    MotorCommand_t cmd;
    HAL_StatusTypeDef result;

    (void)argument;

    Motor_InitIO();

    for (;;)
    {
        if (osMessageQueueGet(MotorQueueHandle, &cmd, NULL, osWaitForever) != osOK)
        {
            osDelay(1);
            continue;
        }

        if (cmd.id == MOTOR_CMD_MOVE)
        {
            if (cmd.start_ms > 0)
            {
                osDelay(cmd.start_ms);
            }

            result = Motor_Start(&huart5,
                                 cmd.target,
                                 (uint8_t)cmd.speed,
                                 (uint16_t)cmd.acc_ms);

            if (result == HAL_OK)
            {
                WaitDone();
            }
            else
            {
                motor_state.moving = 0;
                motor_state.error = 1;
            }

            if (cmd.wait_ms > 0)
            {
                osDelay(cmd.wait_ms);
            }
        }
        else if (cmd.id == MOTOR_CMD_STOP)
        {
            (void)Motor_Stop(&huart5);
        }
        else if (cmd.id == MOTOR_CMD_ESTOP)
        {
            (void)Motor_EStop(&huart5);
        }
        else if (cmd.id == MOTOR_CMD_RELEASE)
        {
            (void)Motor_Release(&huart5);
        }
    }
}
