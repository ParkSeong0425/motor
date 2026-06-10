#include "motor.h"
#include "rs485.h"
#include <stdio.h>
#include <stdint.h>

/* AIMotor register */
#define REG_CONTROL_MODE       0x0200

#define REG_DI1_FUNC           0x0302
#define REG_DI1_LOGIC          0x0303
#define REG_DI2_FUNC           0x0304
#define REG_START              0x0305
#define REG_DI3_FUNC           0x0306
#define REG_DI3_LOGIC          0x0307

#define REG_POS_SOURCE         0x0500

#define REG_HOME_TRIGGER       0x051E  /* H05_30 */
#define REG_HOME_MODE          0x051F  /* H05_31 */
#define REG_HOME_FAST_SPEED    0x0520  /* H05_32 */
#define REG_HOME_SLOW_SPEED    0x0521  /* H05_33 */
#define REG_HOME_ACC           0x0522  /* H05_34 */
#define REG_HOME_TIMEOUT       0x0523  /* H05_35 */

#define REG_RUN_MODE           0x1100
#define REG_END_SEG            0x1101
#define REG_MOVE_TYPE          0x1104
#define REG_TARGET_POS         0x110C
#define REG_SPEED              0x110E
#define REG_ACC                0x110F
#define REG_WAIT               0x1110

#define REG_REAL_POS           0x0B07
#define REG_POS_DIFF           0x0B0F

/* AIMotor value */
#define VAL_POS_MODE           1U
#define VAL_DI_SERVO           1U
#define VAL_DI2_START          28U
#define VAL_DI_HOME_SWITCH     31U
#define VAL_DI_ESTOP           34U

#define VAL_POS_INTERNAL       2U
#define VAL_RUN_SINGLE         0U
#define VAL_END_SEG1           1U
#define VAL_MOVE_ABS           1U

#define VAL_HOME_COMM_START    4U

/* Home sensor test setting */
#define HOME_SENSOR_DI2        1U
#define HOME_MODE              1U     /* 0: forward, 1: reverse */
#define HOME_FAST_RPM          100U
#define HOME_SLOW_RPM          20U
#define HOME_ACC_MS            200U
#define HOME_TIMEOUT_MS        60000U

#define DONE_GAP_LIMIT         100L
#define USE_COMM_SERVO_ON      1U

MotorDebug_t motor_debug;
MotorState_t motor_state;

static uint8_t estop_latch = 0U;

typedef struct
{
    uint16_t reg;
    uint16_t val;
} RegPair_t;

#define DI1_ON()   HAL_GPIO_WritePin(DI_1_GPIO_Port, DI_1_Pin, MOTOR_GPIO_ON)
#define DI1_OFF()  HAL_GPIO_WritePin(DI_1_GPIO_Port, DI_1_Pin, MOTOR_GPIO_OFF)

#define DI3_ON()   HAL_GPIO_WritePin(DI_3_GPIO_Port, DI_3_Pin, MOTOR_GPIO_ON)
#define DI3_OFF()  HAL_GPIO_WritePin(DI_3_GPIO_Port, DI_3_Pin, MOTOR_GPIO_OFF)

static uint16_t ToRpm(uint8_t percent)
{
    uint8_t p = percent;

    if (p < LIFT_MIN_PERCENT)
    {
        p = LIFT_MIN_PERCENT;
    }

    if (p > LIFT_MAX_PERCENT)
    {
        p = LIFT_MAX_PERCENT;
    }

    return (uint16_t)(((uint32_t)LIFT_MAX_RPM * (uint32_t)p) / 100UL);
}

int32_t Motor_mmToUnit(int32_t mm)
{
    const int64_t PI_X10000 = 31416LL;
    int64_t sign = 1LL;
    int64_t mm_abs;
    int64_t wheel_cir_x10;
    int64_t unit;

    if (mm < 0L)
    {
        sign = -1LL;
        mm_abs = -(int64_t)mm;
    }
    else
    {
        mm_abs = (int64_t)mm;
    }

    wheel_cir_x10 = ((int64_t)WHEEL_DIA_MM_X10 * PI_X10000) / 10000LL;

    if (wheel_cir_x10 <= 0LL)
    {
        return 0L;
    }

    unit =
        (mm_abs * 10LL *
         (int64_t)MOTOR_UNIT_PER_TURN *
         (int64_t)DRIVE_RATIO_NUM)
        /
        (wheel_cir_x10 * (int64_t)DRIVE_RATIO_DEN);

    return (int32_t)(unit * sign);
}

static uint32_t GuessMs(int32_t now, int32_t target, uint16_t rpm, uint16_t acc_ms)
{
    int32_t diff = target - now;
    uint32_t units;
    uint32_t turns;
    uint32_t ms;

    if (rpm == 0U)
    {
        return 60000UL;
    }

    units = (diff < 0L) ? (uint32_t)(-diff) : (uint32_t)diff;

    if (units == 0UL)
    {
        return 500UL;
    }

    turns = units / (uint32_t)MOTOR_UNIT_PER_TURN;

    if ((units % (uint32_t)MOTOR_UNIT_PER_TURN) != 0UL)
    {
        turns++;
    }

    ms = ((turns * 60000UL) / (uint32_t)rpm) + (uint32_t)acc_ms + 300UL;

    if (ms < 500UL)
    {
        ms = 500UL;
    }

    if (ms > 60000UL)
    {
        ms = 60000UL;
    }

    return ms;
}

static void ClearState(void)
{
    motor_state.setup_done = 0U;
    motor_state.home_done = 0U;
    motor_state.running = 0U;
    motor_state.error = 0U;

    motor_state.enable_on = 0U;
    motor_state.estop_on = 0U;
    motor_state.servo_on = 0U;

    motor_state.home_offset = 0L;
    motor_state.cur_pos = 0L;
    motor_state.last_target = 0L;

    motor_state.last_speed = 0U;
    motor_state.last_rpm = 0U;
    motor_state.last_acc_ms = 0U;
    motor_state.move_ms = 0UL;
    motor_state.seq = 0UL;
    motor_state.last_hal = HAL_OK;

    motor_debug.uart_error = 0UL;
    motor_debug.crc_calc = 0U;
    motor_debug.crc_recv = 0U;
    motor_debug.crc_ok = 0U;
    motor_debug.last_reg = 0U;
    motor_debug.exception_code = 0U;
    motor_debug.last_pos = 0L;

    estop_latch = 0U;
}

void Motor_InitIO(void)
{
    DI1_OFF();
    DI3_OFF();
    Bus_Rx();
    ClearState();
}

static HAL_StatusTypeDef Write16(UART_HandleTypeDef *huart,
                                 uint16_t reg,
                                 uint16_t val,
                                 const char *name)
{
    HAL_StatusTypeDef st;

    st = Bus_Write16(huart, reg, val);
    motor_state.last_hal = st;

    if (st != HAL_OK)
    {
        printf("[MOTOR] %s fail reg=0x%04X val=%u st=%d ex=%u crc=%u uart=0x%08lX\r\n",
               name,
               reg,
               val,
               st,
               motor_debug.exception_code,
               motor_debug.crc_ok,
               (unsigned long)motor_debug.uart_error);
    }

    HAL_Delay(10U);
    return st;
}

static HAL_StatusTypeDef Write32(UART_HandleTypeDef *huart,
                                 uint16_t reg,
                                 int32_t val,
                                 const char *name)
{
    HAL_StatusTypeDef st;

    st = Bus_Write32(huart, reg, val);
    motor_state.last_hal = st;

    if (st != HAL_OK)
    {
        printf("[MOTOR] %s fail reg=0x%04X val=%ld st=%d ex=%u crc=%u uart=0x%08lX\r\n",
               name,
               reg,
               (long)val,
               st,
               motor_debug.exception_code,
               motor_debug.crc_ok,
               (unsigned long)motor_debug.uart_error);
    }

    HAL_Delay(10U);
    return st;
}

static void ServoOff(void)
{
    DI1_OFF();
    motor_state.enable_on = 0U;
    motor_state.servo_on = 0U;
}

static HAL_StatusTypeDef ServoOn(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef st;

    DI1_ON();
    HAL_Delay(300U);

#if USE_COMM_SERVO_ON
    if (huart != NULL)
    {
        st = Bus_Write16(huart, REG_DI1_LOGIC, 1U);

        if (st != HAL_OK)
        {
            printf("[SERVO] comm on fail st=%d ex=%u crc=%u uart=0x%08lX\r\n",
                   st,
                   motor_debug.exception_code,
                   motor_debug.crc_ok,
                   (unsigned long)motor_debug.uart_error);
        }

        HAL_Delay(100U);
    }
#else
    (void)huart;
#endif

    motor_state.enable_on = 1U;
    motor_state.servo_on = 1U;

    return HAL_OK;
}

static HAL_StatusTypeDef StartOff(UART_HandleTypeDef *huart)
{
    return Write16(huart, REG_START, 0U, "start_off");
}

static HAL_StatusTypeDef StartOn(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef st;

    st = Write16(huart, REG_START, 0U, "start_0");

    if (st != HAL_OK)
    {
        return st;
    }

    HAL_Delay(20U);

    return Write16(huart, REG_START, 1U, "start_1");
}

static HAL_StatusTypeDef StopCore(UART_HandleTypeDef *huart, uint8_t emergency)
{
    int32_t pos;

    if (huart != NULL)
    {
        (void)StartOff(huart);

        if (Motor_ReadPos(huart, &pos) != HAL_OK)
        {
            motor_state.cur_pos = motor_state.last_target;
        }
    }

    motor_state.running = 0U;

    if (emergency != 0U)
    {
        ServoOff();
        DI3_ON();

        motor_state.estop_on = 1U;
        motor_state.error = 1U;
        estop_latch = 1U;
    }

    motor_state.last_hal = HAL_OK;
    return HAL_OK;
}

static HAL_StatusTypeDef WriteMove(UART_HandleTypeDef *huart,
                                   int32_t target,
                                   uint16_t rpm,
                                   uint16_t acc_ms)
{
    HAL_StatusTypeDef st;

    printf("[MOVE] target=%ld rpm=%u acc=%u\r\n",
           (long)target,
           rpm,
           acc_ms);

    st = StartOff(huart);
    if (st != HAL_OK) return st;

    HAL_Delay(50U);

    st = Write16(huart, REG_MOVE_TYPE, VAL_MOVE_ABS, "move_type");
    if (st != HAL_OK) return st;

    st = Write32(huart, REG_TARGET_POS, target, "target");
    if (st != HAL_OK) return st;

    st = Write16(huart, REG_SPEED, rpm, "speed");
    if (st != HAL_OK) return st;

    st = Write16(huart, REG_ACC, acc_ms, "acc");
    if (st != HAL_OK) return st;

    st = Write16(huart, REG_WAIT, 0U, "wait");
    if (st != HAL_OK) return st;

    return HAL_OK;
}

HAL_StatusTypeDef Motor_Setup(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef st;

    const RegPair_t setup_list[] =
    {
        { REG_CONTROL_MODE, VAL_POS_MODE     },
        { REG_DI1_FUNC,     VAL_DI_SERVO     },
        { REG_DI1_LOGIC,    0U               },
        { REG_DI2_FUNC,     VAL_DI2_START    },
        { REG_START,        0U               },
        { REG_DI3_FUNC,     VAL_DI_ESTOP     },
        { REG_DI3_LOGIC,    0U               },
        { REG_POS_SOURCE,   VAL_POS_INTERNAL },
        { REG_RUN_MODE,     VAL_RUN_SINGLE   },
        { REG_END_SEG,      VAL_END_SEG1     },
        { REG_WAIT,         0U               }
    };

    if (huart == NULL)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    Bus_Rx();
    HAL_Delay(50U);

    for (uint32_t i = 0U; i < (sizeof(setup_list) / sizeof(setup_list[0])); i++)
    {
        st = Write16(huart, setup_list[i].reg, setup_list[i].val, "setup");

        if (st != HAL_OK)
        {
            motor_state.setup_done = 0U;
            motor_state.error = 1U;
            return st;
        }
    }

    (void)ServoOn(huart);

    motor_state.setup_done = 1U;
    motor_state.error = 0U;
    motor_state.last_hal = HAL_OK;

    return HAL_OK;
}

HAL_StatusTypeDef Motor_Start(UART_HandleTypeDef *huart,
                              int32_t pos,
                              uint8_t speed,
                              uint16_t acc_ms)
{
    HAL_StatusTypeDef st;
    int32_t target;
    uint16_t rpm;

    if (huart == NULL || estop_latch != 0U || motor_state.home_done == 0U)
    {
        motor_state.running = 0U;
        motor_state.error = 1U;
        motor_state.last_hal = HAL_ERROR;

        if (motor_state.home_done == 0U)
        {
            printf("[MOTOR] home not set\r\n");
        }

        return HAL_ERROR;
    }

    if (motor_state.setup_done == 0U)
    {
        st = Motor_Setup(huart);

        if (st != HAL_OK)
        {
            motor_state.running = 0U;
            motor_state.error = 1U;
            motor_state.last_hal = st;
            return st;
        }
    }

    target = motor_state.home_offset + pos;
    rpm = ToRpm(speed);

    motor_state.last_target = target;
    motor_state.last_speed = speed;
    motor_state.last_rpm = rpm;
    motor_state.last_acc_ms = acc_ms;
    motor_state.move_ms = GuessMs(motor_state.cur_pos, target, rpm, acc_ms);
    motor_state.running = 1U;

    (void)ServoOn(huart);

    st = WriteMove(huart, target, rpm, acc_ms);

    if (st == HAL_OK)
    {
        st = StartOn(huart);
    }

    if (st != HAL_OK)
    {
        motor_state.running = 0U;
        motor_state.error = 1U;
        motor_state.last_hal = st;
        return st;
    }

    printf("[MOTOR_START] target=%ld rpm=%u acc=%u start_ok\r\n",
           (long)target,
           rpm,
           acc_ms);

    motor_debug.last_pos = target;
    motor_state.error = 0U;
    motor_state.last_hal = HAL_OK;

    return HAL_OK;
}

/*
 * 사수가 말한 방식:
 * DI2를 Home_Switch로 잠깐 바꾸고,
 * H05_30=4로 모터 내부 원점복귀를 시작한다.
 *
 * 테스트 후 일반 move 전에 motor setup을 다시 실행하면
 * DI2가 다시 PosInSen으로 돌아간다.
 */
HAL_StatusTypeDef Motor_StartHome(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef st;

    const uint16_t REG_HOME_TRIGGER    = 0x051E;  /* H05_30 */
    const uint16_t REG_HOME_MODE       = 0x051F;  /* H05_31 */
    const uint16_t REG_HOME_FAST_SPEED = 0x0520;  /* H05_32 */
    const uint16_t REG_HOME_SLOW_SPEED = 0x0521;  /* H05_33 */
    const uint16_t REG_HOME_ACC        = 0x0522;  /* H05_34 */
    const uint16_t REG_HOME_TIMEOUT    = 0x0523;  /* H05_35 */

    const uint16_t VAL_HOME_SWITCH     = 31U;
    const uint16_t VAL_HOME_START      = 4U;

    /*
     * 방향이 반대면 1U를 0U로 바꿔서 테스트.
     */
    const uint16_t HOME_MODE_VALUE     = 1U;

    if (huart == NULL || estop_latch != 0U)
    {
        motor_state.running = 0U;
        motor_state.error = 1U;
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    if (motor_state.setup_done == 0U)
    {
        st = Motor_Setup(huart);

        if (st != HAL_OK)
        {
            motor_state.running = 0U;
            motor_state.error = 1U;
            motor_state.last_hal = st;
            return st;
        }
    }

    /*
     * DI2를 원점센서 입력으로 변경.
     * 일반 move 전에 motor setup을 다시 하면 DI2가 PosInSen으로 복구된다.
     */
    st = Bus_Write16(huart, REG_DI2_FUNC, VAL_HOME_SWITCH);
    if (st != HAL_OK) return st;
    HAL_Delay(20U);

    st = Bus_Write16(huart, REG_HOME_MODE, HOME_MODE_VALUE);
    if (st != HAL_OK) return st;
    HAL_Delay(20U);

    st = Bus_Write16(huart, REG_HOME_FAST_SPEED, 100U);
    if (st != HAL_OK) return st;
    HAL_Delay(20U);

    st = Bus_Write16(huart, REG_HOME_SLOW_SPEED, 20U);
    if (st != HAL_OK) return st;
    HAL_Delay(20U);

    st = Bus_Write16(huart, REG_HOME_ACC, 200U);
    if (st != HAL_OK) return st;
    HAL_Delay(20U);

    st = Bus_Write16(huart, REG_HOME_TIMEOUT, 60000U);
    if (st != HAL_OK) return st;
    HAL_Delay(20U);

    (void)ServoOn(huart);

    /*
     * H05_30 = 4
     * 통신으로 모터 내부 원점복귀 시작.
     */
    st = Bus_Write16(huart, REG_HOME_TRIGGER, VAL_HOME_START);

    if (st != HAL_OK)
    {
        motor_state.running = 0U;
        motor_state.error = 1U;
        motor_state.last_hal = st;

        printf("[HOME] trigger fail st=%d ex=%u crc=%u uart=0x%08lX\r\n",
               st,
               motor_debug.exception_code,
               motor_debug.crc_ok,
               (unsigned long)motor_debug.uart_error);

        return st;
    }

    motor_state.running = 1U;
    motor_state.error = 0U;
    motor_state.move_ms = 65000U;
    motor_state.setup_done = 0U;
    motor_state.last_hal = HAL_OK;

    printf("[HOME] internal home start mode=%u\r\n", HOME_MODE_VALUE);

    return HAL_OK;
}

HAL_StatusTypeDef Motor_ReadPos(UART_HandleTypeDef *huart, int32_t *pos)
{
    HAL_StatusTypeDef st;

    if (huart == NULL || pos == NULL)
    {
        return HAL_ERROR;
    }

    st = Bus_Read32(huart, REG_REAL_POS, pos);

    if (st == HAL_OK)
    {
        motor_state.cur_pos = *pos;
        motor_debug.last_pos = *pos;
    }

    motor_state.last_hal = st;
    return st;
}

HAL_StatusTypeDef Motor_ReadDiff(UART_HandleTypeDef *huart, int32_t *diff)
{
    HAL_StatusTypeDef st;

    if (huart == NULL || diff == NULL)
    {
        return HAL_ERROR;
    }

    st = Bus_Read32(huart, REG_POS_DIFF, diff);
    motor_state.last_hal = st;

    return st;
}

uint8_t Motor_CheckDone(UART_HandleTypeDef *huart)
{
    int32_t diff;
    int32_t pos;
    int32_t gap;

    if (Motor_ReadDiff(huart, &diff) == HAL_OK)
    {
        (void)diff;
    }

    if (Motor_ReadPos(huart, &pos) != HAL_OK)
    {
        return 0U;
    }

    gap = pos - motor_state.last_target;

    if (gap < 0L)
    {
        gap = -gap;
    }

    return (gap <= DONE_GAP_LIMIT) ? 1U : 0U;
}

HAL_StatusTypeDef Motor_Done(UART_HandleTypeDef *huart)
{
    (void)StopCore(huart, 0U);

    motor_state.error = 0U;
    motor_state.seq++;
    motor_state.last_hal = HAL_OK;

    return HAL_OK;
}

HAL_StatusTypeDef Motor_Stop(UART_HandleTypeDef *huart)
{
    return StopCore(huart, 0U);
}

HAL_StatusTypeDef Motor_EStop(UART_HandleTypeDef *huart)
{
    return StopCore(huart, 1U);
}

HAL_StatusTypeDef Motor_Release(UART_HandleTypeDef *huart)
{
    (void)huart;

    ServoOff();
    DI3_OFF();

    HAL_Delay(500U);

    estop_latch = 0U;
    motor_state.estop_on = 0U;
    motor_state.running = 0U;
    motor_state.error = 0U;
    motor_state.setup_done = 0U;
    motor_state.last_hal = HAL_OK;

    return HAL_OK;
}

HAL_StatusTypeDef Motor_SaveHomeHere(UART_HandleTypeDef *huart)
{
    int32_t pos;

    if (huart == NULL)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    (void)StartOff(huart);
    HAL_Delay(100U);

    if (Motor_ReadPos(huart, &pos) != HAL_OK)
    {
        pos = motor_state.cur_pos;
    }

    return Motor_SetHomeOffset(pos);
}

HAL_StatusTypeDef Motor_SetHome(void)
{
    motor_state.home_offset = motor_state.cur_pos;
    motor_state.home_done = 1U;
    motor_state.error = 0U;
    motor_state.last_hal = HAL_OK;

    return HAL_OK;
}

HAL_StatusTypeDef Motor_SetHomeOffset(int32_t home_offset)
{
    motor_state.home_offset = home_offset;
    motor_state.home_done = 1U;
    motor_state.running = 0U;
    motor_state.error = 0U;
    motor_state.seq++;
    motor_state.last_hal = HAL_OK;

    return HAL_OK;
}

HAL_StatusTypeDef Motor_ClearHome(void)
{
    motor_state.home_done = 0U;
    motor_state.last_hal = HAL_OK;

    return HAL_OK;
}

uint8_t Motor_HomeOk(void)
{
    return motor_state.home_done;
}

uint8_t Motor_IsEStop(void)
{
    return estop_latch;
}

uint8_t Motor_IsBusy(void)
{
    return motor_state.running;
}

osStatus_t Motor_SendCmd(const MotorCommand_t *cmd)
{
    if (cmd == NULL)
    {
        return osErrorParameter;
    }

    if (MotorQueueHandle == NULL)
    {
        return osErrorResource;
    }

    return osMessageQueuePut(MotorQueueHandle, cmd, 0U, 0U);
}

osStatus_t Motor_SendMove(int32_t pos,
                          uint32_t speed,
                          uint32_t acc_ms,
                          uint32_t start_ms,
                          uint32_t wait_ms)
{
    MotorCommand_t cmd;

    if (MotorQueueHandle == NULL) return osErrorResource;
    if (Motor_IsBusy() != 0U) return osErrorResource;
    if (osMessageQueueGetCount(MotorQueueHandle) > 0U) return osErrorResource;
    if (Motor_IsEStop() != 0U) return osErrorResource;

    cmd.id = MOTOR_CMD_MOVE;
    cmd.pos = pos;
    cmd.speed = speed;
    cmd.acc_ms = acc_ms;
    cmd.start_ms = start_ms;
    cmd.wait_ms = wait_ms;

    return Motor_SendCmd(&cmd);
}

osStatus_t Motor_SendStop(void)
{
    MotorCommand_t cmd;

    cmd.id = MOTOR_CMD_STOP;
    cmd.pos = 0L;
    cmd.speed = 0U;
    cmd.acc_ms = 0U;
    cmd.start_ms = 0U;
    cmd.wait_ms = 0U;

    return Motor_SendCmd(&cmd);
}

osStatus_t Motor_SendEStop(void)
{
    MotorCommand_t cmd;

    cmd.id = MOTOR_CMD_ESTOP;
    cmd.pos = 0L;
    cmd.speed = 0U;
    cmd.acc_ms = 0U;
    cmd.start_ms = 0U;
    cmd.wait_ms = 0U;

    return Motor_SendCmd(&cmd);
}

osStatus_t Motor_SendRelease(void)
{
    MotorCommand_t cmd;

    cmd.id = MOTOR_CMD_RELEASE;
    cmd.pos = 0L;
    cmd.speed = 0U;
    cmd.acc_ms = 0U;
    cmd.start_ms = 0U;
    cmd.wait_ms = 0U;

    return Motor_SendCmd(&cmd);
}

osStatus_t Motor_SendHome(void)
{
    MotorCommand_t cmd;

    if (MotorQueueHandle == NULL) return osErrorResource;
    if (Motor_IsBusy() != 0U) return osErrorResource;
    if (osMessageQueueGetCount(MotorQueueHandle) > 0U) return osErrorResource;
    if (Motor_IsEStop() != 0U) return osErrorResource;

    cmd.id = MOTOR_CMD_HOME;
    cmd.pos = 0L;
    cmd.speed = HOME_FAST_RPM;
    cmd.acc_ms = HOME_ACC_MS;
    cmd.start_ms = 0U;
    cmd.wait_ms = 0U;

    return Motor_SendCmd(&cmd);
}
