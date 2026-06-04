#include "motor.h"
#include "motor_bus.h"

/*
 * motor.c
 *
 * 모터 동작 순서를 관리한다.
 * RS485 통신은 motor_bus.c가 담당한다.
 */

/* AIMotor Register */
#define REG_CONTROL_MODE             0x0200

#define REG_DI1_FUNC                 0x0302
#define REG_START                    0x0305
#define REG_DI3_FUNC                 0x0306
#define REG_DI3_LOGIC                0x0307

#define REG_POS_SOURCE               0x0500

#define REG_END_SEG                  0x1101
#define REG_MOVE_TYPE                0x1104
#define REG_TARGET_POS               0x110C
#define REG_SPEED                    0x110E
#define REG_ACC                      0x110F
#define REG_WAIT                     0x1110

/*
 * 01H 원점 탐색 거리.
 * 현재 위치에서 MI 방향으로 충분히 크게 움직이면서 센서를 찾는다.
 * 방향이 반대면 - 부호를 + 로 바꾸면 된다.
 */
#define HOME_FIND_POS                (-100000L) // (100000L) 시계방향으로 안돌 때 이렇게 적용
#define HOME_FIND_SPEED              10U
#define HOME_FIND_ACC_MS             1000U

/* Monitor Register */
#define REG_REAL_POS                 0x0B07
#define REG_POS_DIFF                 0x0B0F

/* AIMotor 설정값 */
#define VAL_POS_MODE                 1U
#define VAL_DI_SERVO                 1U
#define VAL_DI_ESTOP                 34U

#define VAL_POS_INTERNAL             2U
#define VAL_END_SEG1                 1U
#define VAL_MOVE_ABS                 1U

#define MOTOR_DONE_DIFF_LIMIT        100L

MotorDebug_t motor_debug;
MotorState_t motor_state;

static uint8_t estop_latch = 0U;

typedef struct
{
    uint16_t reg;
    uint16_t val;
} RegPair_t;

#define DI1_ON()     HAL_GPIO_WritePin(DI_1_GPIO_Port, DI_1_Pin, MOTOR_GPIO_ON)
#define DI1_OFF()    HAL_GPIO_WritePin(DI_1_GPIO_Port, DI_1_Pin, MOTOR_GPIO_OFF)

#define DI3_ON()     HAL_GPIO_WritePin(DI_3_GPIO_Port, DI_3_Pin, MOTOR_GPIO_ON)
#define DI3_OFF()    HAL_GPIO_WritePin(DI_3_GPIO_Port, DI_3_Pin, MOTOR_GPIO_OFF)

static uint16_t ToRpm(uint8_t percent)
{
    uint8_t p;

    p = percent;

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

/*
 * mm 거리를 motor unit으로 변환한다.
 *
 * TCP에서 들어오는 위치값은 이제 motor unit이 아니라 mm로 본다.
 *
 * 공식:
 *   바퀴 1바퀴 거리 = 지름 × pi
 *   unit = mm × 모터 1바퀴 unit / 바퀴 1바퀴 거리
 *
 * 정수 연산만 사용하기 위해:
 *   WHEEL_DIA_MM_X10 = 지름(mm) × 10
 *   PI_X10000 = 3.1416 × 10000
 */
int32_t Motor_mmToUnit(int32_t mm)
{
    const int64_t PI_X10000 = 31416LL;

    int64_t sign;
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
        sign = 1LL;
        mm_abs = (int64_t)mm;
    }

    /*
     * wheel_cir_x10 단위:
     *   0.1mm
     *
     * 예:
     *   지름 50.0mm = WHEEL_DIA_MM_X10 500
     *   원주 = 50.0 × 3.1416 = 157.08mm
     *   wheel_cir_x10 = 약 1570
     */
    wheel_cir_x10 = ((int64_t)WHEEL_DIA_MM_X10 * PI_X10000) / 10000LL;

    if (wheel_cir_x10 <= 0LL)
    {
        return 0L;
    }

    /*
     * mm_abs × 10:
     *   mm를 0.1mm 단위로 변환.
     *
     * DRIVE_RATIO_NUM / DRIVE_RATIO_DEN:
     *   모터와 바퀴가 1:1이 아닐 때 보정.
     */
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
    int32_t diff;
    uint32_t units;
    uint32_t turns;
    uint32_t move_ms;
    uint32_t wait_ms;

    if (rpm == 0U)
    {
        return 60000UL;
    }

    diff = target - now;

    if (diff < 0L)
    {
        units = (uint32_t)(-diff);
    }
    else
    {
        units = (uint32_t)diff;
    }

    if (units == 0UL)
    {
        return 500UL;
    }

    turns = units / (uint32_t)MOTOR_UNIT_PER_TURN;

    if ((units % (uint32_t)MOTOR_UNIT_PER_TURN) != 0UL)
    {
        turns++;
    }

    move_ms = (turns * 60000UL) / (uint32_t)rpm;
    wait_ms = move_ms + (uint32_t)acc_ms + 300UL;

    if (wait_ms < 500UL)
    {
        wait_ms = 500UL;
    }

    if (wait_ms > 60000UL)
    {
        wait_ms = 60000UL;
    }

    return wait_ms;
}

void Motor_InitIO(void)
{
    /*
     * DI1 = STM이 Servo Enable 제어
     * DI2 = 광센서 입력으로 읽기만 함
     * DI3 = STM이 Emergency Stop 제어
     */
    DI1_OFF();
    DI3_OFF();

    Bus_Rx();

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

static void ServoOn(void)
{
    DI1_ON();
    HAL_Delay(300U);

    motor_state.enable_on = 1U;
    motor_state.servo_on = 1U;
}

static void ServoOff(void)
{
    DI1_OFF();

    motor_state.enable_on = 0U;
    motor_state.servo_on = 0U;
}

static HAL_StatusTypeDef StartOff(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef st;

    st = Bus_Write16(huart, REG_START, 0U);
    motor_state.last_hal = st;

    return st;
}

static HAL_StatusTypeDef StartOn(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef st;

    st = Bus_Write16(huart, REG_START, 1U);
    motor_state.last_hal = st;

    return st;
}

HAL_StatusTypeDef Motor_Setup(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef st;
    uint32_t i;

    const RegPair_t list[] =
    {
        { REG_CONTROL_MODE, VAL_POS_MODE     },
        { REG_DI1_FUNC,     VAL_DI_SERVO     },
        { REG_START,        0U               },
        { REG_DI3_FUNC,     VAL_DI_ESTOP     },
        { REG_DI3_LOGIC,    0U               },
        { REG_POS_SOURCE,   VAL_POS_INTERNAL },
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

    for (i = 0U; i < (sizeof(list) / sizeof(list[0])); i++)
    {
        st = Bus_Write16(huart, list[i].reg, list[i].val);

        if (st != HAL_OK)
        {
            motor_state.setup_done = 0U;
            motor_state.error = 1U;
            motor_state.last_hal = st;
            return st;
        }
    }

    motor_state.setup_done = 1U;
    motor_state.error = 0U;
    motor_state.last_hal = HAL_OK;

    return HAL_OK;
}

static HAL_StatusTypeDef WriteMove(
    UART_HandleTypeDef *huart,
    int32_t target,
    uint16_t rpm,
    uint16_t acc_ms
)
{
    HAL_StatusTypeDef st;

    st = StartOff(huart);

    if (st != HAL_OK)
    {
        return st;
    }

    HAL_Delay(50U);

    st = Bus_Write16(huart, REG_MOVE_TYPE, VAL_MOVE_ABS);

    if (st != HAL_OK)
    {
        return st;
    }

    st = Bus_Write32(huart, REG_TARGET_POS, target);

    if (st != HAL_OK)
    {
        return st;
    }

    st = Bus_Write16(huart, REG_SPEED, rpm);

    if (st != HAL_OK)
    {
        return st;
    }

    st = Bus_Write16(huart, REG_ACC, acc_ms);

    if (st != HAL_OK)
    {
        return st;
    }

    return Bus_Write16(huart, REG_WAIT, 0U);
}

HAL_StatusTypeDef Motor_Start(
    UART_HandleTypeDef *huart,
    int32_t pos,
    uint8_t speed,
    uint16_t acc_ms
)
{
    HAL_StatusTypeDef st;
    int32_t target;
    uint16_t rpm;

    if (huart == NULL)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    if (estop_latch != 0U)
    {
        motor_state.running = 0U;
        motor_state.error = 1U;
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    if (motor_state.home_done == 0U)
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

        HAL_Delay(300U);
    }

    target = motor_state.home_offset + pos;
    rpm = ToRpm(speed);

    motor_state.last_target = target;
    motor_state.last_speed = speed;
    motor_state.last_rpm = rpm;
    motor_state.last_acc_ms = acc_ms;
    motor_state.move_ms = GuessMs(motor_state.cur_pos, target, rpm, acc_ms);
    motor_state.running = 1U;

    st = WriteMove(huart, target, rpm, acc_ms);

    if (st != HAL_OK)
    {
        motor_state.running = 0U;
        motor_state.error = 1U;
        motor_state.last_hal = st;
        return st;
    }

    ServoOn();

    st = StartOn(huart);

    if (st != HAL_OK)
    {
        ServoOff();

        motor_state.running = 0U;
        motor_state.error = 1U;
        motor_state.last_hal = st;
        return st;
    }

    motor_debug.last_pos = target;
    motor_state.error = 0U;
    motor_state.last_hal = HAL_OK;

    return HAL_OK;
}
HAL_StatusTypeDef Motor_StartHome(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef st;
    int32_t now;
    int32_t target;
    uint16_t rpm;

    if (huart == NULL)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    if (estop_latch != 0U)
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

        HAL_Delay(300U);
    }

    /*
     * 현재 모터 실제 위치를 읽고,
     * 그 위치에서 MI 방향으로 길게 움직이면서 센서를 찾는다.
     */
    if (Motor_ReadPos(huart, &now) != HAL_OK)
    {
        now = motor_state.cur_pos;
    }

    target = now + HOME_FIND_POS;
    rpm = ToRpm(HOME_FIND_SPEED);

    motor_state.last_target = target;
    motor_state.last_speed = HOME_FIND_SPEED;
    motor_state.last_rpm = rpm;
    motor_state.last_acc_ms = HOME_FIND_ACC_MS;
    motor_state.move_ms = GuessMs(now, target, rpm, HOME_FIND_ACC_MS);
    motor_state.running = 1U;
    motor_state.error = 0U;

    st = WriteMove(huart, target, rpm, HOME_FIND_ACC_MS);

    if (st != HAL_OK)
    {
        motor_state.running = 0U;
        motor_state.error = 1U;
        motor_state.last_hal = st;
        return st;
    }

    ServoOn();

    st = StartOn(huart);

    if (st != HAL_OK)
    {
        ServoOff();
        motor_state.running = 0U;
        motor_state.error = 1U;
        motor_state.last_hal = st;
        return st;
    }

    motor_state.last_hal = HAL_OK;
    return HAL_OK;
}

HAL_StatusTypeDef Motor_ReadPos(UART_HandleTypeDef *huart, int32_t *pos)
{
    HAL_StatusTypeDef st;

    if ((huart == NULL) || (pos == NULL))
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

    if ((huart == NULL) || (diff == NULL))
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

    /*
     * 1차 판단:
     * H0B_15 위치 편차값으로 완료 판단.
     */
    if (Motor_ReadDiff(huart, &diff) == HAL_OK)
    {
        if (diff < 0L)
        {
            diff = -diff;
        }

        if (diff <= MOTOR_DONE_DIFF_LIMIT)
        {
            return 1U;
        }
    }

    /*
     * 2차 판단:
     * H0B_07 현재 위치가 목표 위치 근처인지 확인.
     *
     * H0B_15 읽기 실패나 값 이상이 있어도
     * 실제 위치가 목표 근처라면 완료로 본다.
     */
    if (Motor_ReadPos(huart, &pos) == HAL_OK)
    {
        gap = pos - motor_state.last_target;

        if (gap < 0L)
        {
            gap = -gap;
        }

        if (gap <= MOTOR_DONE_DIFF_LIMIT)
        {
            return 1U;
        }
    }

    return 0U;
}

HAL_StatusTypeDef Motor_Done(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef st;
    int32_t pos;

    st = StartOff(huart);

    if (st != HAL_OK)
    {
        motor_state.running = 0U;
        motor_state.error = 1U;
        motor_state.last_hal = st;
        return st;
    }

    if (Motor_ReadPos(huart, &pos) != HAL_OK)
    {
        motor_state.cur_pos = motor_state.last_target;
    }


    motor_state.running = 0U;
    motor_state.error = 0U;
    motor_state.seq++;
    motor_state.last_hal = HAL_OK;

    return HAL_OK;
}

HAL_StatusTypeDef Motor_SaveHomeHere(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef st;
    int32_t pos;

    if (huart == NULL)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    /*
     * 센서 감지 순간 즉시 Start OFF.
     */
    st = StartOff(huart);

    if (st != HAL_OK)
    {
        motor_state.running = 0U;
        motor_state.error = 1U;
        motor_state.last_hal = st;
        return st;
    }

    /*
     * 모터가 완전히 정지할 시간을 조금 준다.
     * 너무 길면 오차가 커질 수 있고, 너무 짧으면 위치 읽기가 흔들릴 수 있다.
     */
    HAL_Delay(100U);

    /*
     * 멈춘 위치를 읽고, 그 위치를 새로운 원점으로 저장한다.
     */
    if (Motor_ReadPos(huart, &pos) != HAL_OK)
    {
        pos = motor_state.cur_pos;
    }

    motor_state.cur_pos = pos;
    motor_state.home_offset = pos;
    motor_state.home_done = 1U;

    motor_state.running = 0U;
    motor_state.error = 0U;
    motor_state.seq++;
    motor_state.last_hal = HAL_OK;

    return HAL_OK;
}


HAL_StatusTypeDef Motor_Stop(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef st;
    int32_t pos;

    if (huart != NULL)
    {
        st = StartOff(huart);

        if (Motor_ReadPos(huart, &pos) != HAL_OK)
        {
            motor_state.cur_pos = motor_state.last_target;
        }
    }
    else
    {
        st = HAL_OK;
    }

    motor_state.running = 0U;
    motor_state.last_hal = st;

    return st;
}

HAL_StatusTypeDef Motor_EStop(UART_HandleTypeDef *huart)
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

    ServoOff();

    DI3_ON();
    motor_state.estop_on = 1U;

    motor_state.running = 0U;
    motor_state.error = 1U;
    estop_latch = 1U;
    motor_state.last_hal = HAL_OK;

    return HAL_OK;
}

HAL_StatusTypeDef Motor_Release(UART_HandleTypeDef *huart)
{
    (void)huart;

    ServoOff();

    DI3_OFF();
    motor_state.estop_on = 0U;
    HAL_Delay(500U);

    estop_latch = 0U;
    motor_state.running = 0U;
    motor_state.error = 0U;

    motor_state.setup_done = 0U;
    motor_state.last_hal = HAL_OK;

    return HAL_OK;
}


HAL_StatusTypeDef Motor_SetHome(void)
{
    motor_state.home_offset = motor_state.cur_pos;
    motor_state.home_done = 1U;
    motor_state.error = 0U;
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

osStatus_t Motor_SendMove(
    int32_t pos,
    uint32_t speed,
    uint32_t acc_ms,
    uint32_t start_ms,
    uint32_t wait_ms
)
{
    MotorCommand_t cmd;

    if (MotorQueueHandle == NULL)
    {
        return osErrorResource;
    }

    if (Motor_IsBusy() != 0U)
    {
        return osErrorResource;
    }

    if (osMessageQueueGetCount(MotorQueueHandle) > 0U)
    {
        return osErrorResource;
    }

    if (Motor_IsEStop() != 0U)
    {
        return osErrorResource;
    }

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

    if (MotorQueueHandle == NULL)
    {
        return osErrorResource;
    }

    if (Motor_IsBusy() != 0U)
    {
        return osErrorResource;
    }

    if (osMessageQueueGetCount(MotorQueueHandle) > 0U)
    {
        return osErrorResource;
    }

    if (Motor_IsEStop() != 0U)
    {
        return osErrorResource;
    }

    /*
     * 01H는 일반 MOVE가 아니다.
     * 센서를 찾는 원점 탐색 명령이다.
     */
    cmd.id = MOTOR_CMD_HOME;
    cmd.pos = 0L;
    cmd.speed = HOME_FIND_SPEED;
    cmd.acc_ms = HOME_FIND_ACC_MS;
    cmd.start_ms = 0U;
    cmd.wait_ms = 0U;

    return Motor_SendCmd(&cmd);
}
