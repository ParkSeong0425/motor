#include "motor.h"
#include "motor_bus.h"

/*
 * motor.c
 *
 * 역할:
 *   모터 동작 순서를 관리한다.
 *   RS485 통신 세부 처리는 motor_bus.c가 담당한다.
 */

/* AIMotor Register */
#define REG_CONTROL_MODE             0x0200

#define REG_DI1_FUNC                 0x0302
#define REG_DI2_FUNC                 0x0304
#define REG_START                    0x0305
#define REG_DI3_FUNC                 0x0306
#define REG_DI3_LOGIC                0x0307

#define REG_POS_SOURCE               0x0500

#define REG_RUN_MODE                 0x1100
#define REG_END_SEG                  0x1101
#define REG_MOVE_TYPE                0x1104
#define REG_TARGET_POS               0x110C
#define REG_SPEED                    0x110E
#define REG_ACC                      0x110F
#define REG_WAIT                     0x1110

/* AIMotor 설정값 */
#define VAL_POS_MODE                 1U
#define VAL_DI_SERVO                 1U
#define VAL_DI_ESTOP                 34U

#define VAL_POS_INTERNAL             2U
#define VAL_RUN_SINGLE               0U
#define VAL_END_SEG1                 1U
#define VAL_MOVE_ABS                 1U

MotorDebug_t motor_debug;
MotorState_t motor_state;

static uint8_t estop_latch = 0U;

typedef struct
{
    uint16_t reg;
    uint16_t val;
} RegPair_t;

/* DI GPIO */
#define DI1_ON()     HAL_GPIO_WritePin(DI_1_GPIO_Port, DI_1_Pin, MOTOR_GPIO_ON)
#define DI1_OFF()    HAL_GPIO_WritePin(DI_1_GPIO_Port, DI_1_Pin, MOTOR_GPIO_OFF)

#define DI3_ON()     HAL_GPIO_WritePin(DI_3_GPIO_Port, DI_3_Pin, MOTOR_GPIO_ON)
#define DI3_OFF()    HAL_GPIO_WritePin(DI_3_GPIO_Port, DI_3_Pin, MOTOR_GPIO_OFF)

/*
 * 속도 percent를 rpm으로 변환한다.
 */
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
 * 예상 이동 시간 계산.
 * 현재는 완료 신호를 읽지 않고, 예상 시간 후 Start OFF 한다.
 */
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

/*
 * 모터 GPIO와 내부 상태 초기화.
 * main.c가 아니라 StartMotorTask()에서 1번 호출한다.
 */
void Motor_InitIO(void)
{
	 /*
	     * DI1 = STM이 Servo Enable 제어
	     * DI2 = 광센서가 모터로 직접 입력
	     * DI3 = STM이 Emergency Stop 제어
	     *
	     * 그래서 STM은 DI2를 절대 ON/OFF 하지 않는다.
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

/*
 * AIMotor 기본 설정.
 */
HAL_StatusTypeDef Motor_Setup(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef st;
    uint32_t i;

    const RegPair_t list[] =
    {
        { REG_CONTROL_MODE, VAL_POS_MODE     },
        { REG_DI1_FUNC,     VAL_DI_SERVO     },

        /*
         * DI2는 광센서가 모터에 직접 들어가는 입력이다.
         * 지금은 STM에서 DI2 기능을 덮어쓰지 않는다.
         *
         * 나중에 AIMotor 매뉴얼에서 DI2 Home/Origin 기능 번호를 확인하면
         * 그때 아래처럼 추가한다.
         *
         * { REG_DI2_FUNC, VAL_DI2_HOME_SENSOR },
         */

        { REG_START,        0U               },
        { REG_DI3_FUNC,     VAL_DI_ESTOP     },
        { REG_DI3_LOGIC,    0U               },
        { REG_POS_SOURCE,   VAL_POS_INTERNAL },

        /*
         * 현재 0x1100은 제외 유지
         */
        /* { REG_RUN_MODE,  VAL_RUN_SINGLE   }, */

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

/*
 * 목표 위치, 속도, 가감속을 모터 레지스터에 쓴다.
 */
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

    st = Bus_Write16(huart, REG_WAIT, 0U);

    return st;
}

/*
 * 이동 시작만 한다.
 * 이동 완료 대기는 StartMotorTask()에서 tick으로 처리한다.
 *
 * 순서:
 *   1. Home / E-Stop 상태 확인
 *   2. Setup 안 되어 있으면 Motor_Setup()
 *   3. 목표 위치, 속도, 가감속 레지스터 쓰기
 *   4. DI1 Servo Enable ON
 *   5. Start ON
 */
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

    /*
     * Setup은 DI1을 켜기 전에 먼저 한다.
     */
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

    /*
     * 실제 이동 시작 준비 중.
     * running은 MotorTask가 명령을 꺼낸 시점에도 1이 될 수 있지만,
     * 여기서도 실제 시작 준비 중이라는 의미로 1 유지한다.
     */
    motor_state.running = 1U;

    st = WriteMove(huart, target, rpm, acc_ms);

    if (st != HAL_OK)
    {
        motor_state.running = 0U;
        motor_state.error = 1U;
        motor_state.last_hal = st;
        return st;
    }

    /*
     * 이동 레지스터 쓰기가 성공한 뒤 DI1 Servo Enable ON.
     * ServoOn은 여기서 한 번만 실행한다.
     */
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

/*
 * 이동 예상 시간이 지난 뒤 Start OFF.
 * 이동 완료 후 DI1 Servo Enable도 OFF 한다.
 */
HAL_StatusTypeDef Motor_Done(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef st;

    st = StartOff(huart);

    if (st != HAL_OK)
    {
        motor_state.running = 0U;
        motor_state.error = 1U;
        motor_state.last_hal = st;
        return st;
    }

    /*
     * 이동 완료 후 Servo Enable OFF.
     * 즉, DI1 LED도 꺼진다.
     *
     * 주의:
     * 모터 힘으로 위치를 잡고 있어야 하는 구조면
     * DI1을 끄면 위치 유지력이 떨어질 수 있다.
     */
    ServoOff();

    motor_state.cur_pos = motor_state.last_target;
    motor_state.running = 0U;
    motor_state.error = 0U;
    motor_state.seq++;
    motor_state.last_hal = HAL_OK;

    return HAL_OK;
}

/*
 * 일반 정지.
 */
HAL_StatusTypeDef Motor_Stop(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef st;

    if (huart != NULL)
    {
        st = StartOff(huart);
    }
    else
    {
        st = HAL_OK;
    }

    motor_state.running = 0U;
    motor_state.last_hal = st;

    return st;
}

/*
 * 비상정지.
 */
HAL_StatusTypeDef Motor_EStop(UART_HandleTypeDef *huart)
{
    if (huart != NULL)
    {
        (void)StartOff(huart);
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

/*
 * 비상정지 해제.
 */
HAL_StatusTypeDef Motor_Release(UART_HandleTypeDef *huart)
{
    ServoOff();

    DI3_OFF();
    motor_state.estop_on = 0U;
    HAL_Delay(500U);


    estop_latch = 0U;
    motor_state.running = 0U;
    motor_state.error = 0U;

    /*
     * 비상정지 해제 후 다음 이동 전에 setup을 다시 한다.
     */
    motor_state.setup_done = 0U;
    motor_state.last_hal = HAL_OK;

    return HAL_OK;
}

/*
 * Software Home 설정.
 */
HAL_StatusTypeDef Motor_SetHome(void)
{
    motor_state.home_offset = motor_state.cur_pos;
    motor_state.home_done = 1U;
    motor_state.error = 0U;
    motor_state.last_hal = HAL_OK;

    return HAL_OK;
}

/*
 * Software Home 해제.
 */
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

/*
 * 현재 모터가 명령 처리 중인지 확인한다.
 */
uint8_t Motor_IsBusy(void)
{
    return motor_state.running;
}

/*
 * MotorQueue에 명령을 넣는다.
 */
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

/*
 * 이동 명령을 Queue에 넣는다.
 *
 * 중요:
 *   여기서는 motor_state.running을 1로 만들지 않는다.
 *   running은 StartMotorTask가 Queue에서 명령을 꺼낸 뒤 켜야 한다.
 */
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

    /*
     * 이미 실행 중이면 새 이동 명령 거부.
     */
    if (Motor_IsBusy() != 0U)
    {
        return osErrorResource;
    }

    /*
     * Queue에 아직 처리 안 된 명령이 있으면 새 이동 명령 거부.
     */
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

/*
 * 일반 정지 명령을 Queue에 넣는다.
 */
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

/*
 * 비상정지 명령을 Queue에 넣는다.
 */
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

/*
 * 비상정지 해제 명령을 Queue에 넣는다.
 */
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
