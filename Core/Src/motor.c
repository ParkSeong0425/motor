#include "motor.h"
#include "rs485.h"
#include "usart.h"
#include "cmsis_os.h"
#include "storage.h"
#include <stdint.h>
/* AIMotor register/value */
#define R_MODE 0x0200
#define R_START 0x0305
#define R_SRC 0x0500
#define R_RUN 0x1100
#define R_END 0x1101
#define R_TYPE 0x1104
#define R_POS 0x110C
#define R_SPEED 0x110E
#define R_ACC 0x110F
#define R_WAIT 0x1110
#define R_REAL 0x0B07
#define V_POS 1
#define V_SRC 2
#define V_RUN 0
#define V_END 1
#define V_ABS 1
/* move done check */
#define GAP 5
#define STILL_GAP 1
#define STILL_CNT 10
#define READ_MS 50
#define PI_X10000 31416LL
/* DO polarity: IOC Low=off, High=on */
#define DO_ON GPIO_PIN_SET
#define DO_OFF GPIO_PIN_RESET
/* DO pin map */
#define DO0_PORT GPIOA      /* PA3: spare/manual zero line */
#define DO0_PIN GPIO_PIN_3
#define DO1_PORT GPIOC      /* PC0: data sheet 28 */
#define DO1_PIN GPIO_PIN_0
#define DO2_PORT GPIOC      /* PC3: emergency stop */
#define DO2_PIN GPIO_PIN_3
#define DO3_PORT GPIOF      /* PF3: enable on */
#define DO3_PIN GPIO_PIN_3
#define DO4_PORT GPIOF      /* PF5: enable off */
#define DO4_PIN GPIO_PIN_5
/* pulse time */
#define DO28_MS 200u
#define EN_MS 500u
typedef struct { uint16_t reg; uint16_t val; } Pair_t;
MotorDebug_t motor_debug;
MotorState_t motor_state;
static volatile uint8_t estop_req = 0;
static volatile uint8_t hold_on = 0;   /* 광센서 중간 정지 */
static int32_t hold_pos = 0;           /* 멈춘 raw 위치 */
/* DO output */
static void dox(GPIO_TypeDef *port, uint16_t pin, uint8_t on)
{
    HAL_GPIO_WritePin(port, pin, (on != 0u) ? DO_ON : DO_OFF);
}
/* DO pulse */
static void pulse(GPIO_TypeDef *port, uint16_t pin, uint32_t ms)
{
    dox(port, pin, 1); HAL_Delay(ms); dox(port, pin, 0);
}
/* all DO off */
static void all_off(void)
{
    dox(DO0_PORT, DO0_PIN, 0); dox(DO1_PORT, DO1_PIN, 0);
    dox(DO2_PORT, DO2_PIN, 0); dox(DO3_PORT, DO3_PIN, 0);
    dox(DO4_PORT, DO4_PIN, 0);
}
/* PC13: toggle DO2 and sync software estop */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last = 0;
    uint32_t now = HAL_GetTick();
    if (GPIO_Pin != GPIO_PIN_13 || (now - last) <= 60u) return;
    HAL_GPIO_TogglePin(DO2_PORT, DO2_PIN);
    if (HAL_GPIO_ReadPin(DO2_PORT, DO2_PIN) == DO_ON)
    {
        estop_req = 1; motor_state.error = 1; motor_state.moving = 0;
    }
    else
    {
        estop_req = 0; motor_state.error = 0; motor_state.moving = 0;
    }
    last = now;
}
/* clear state */
static void clear_state(void)
{
    motor_state.setup_ok = 0; motor_state.moving = 0; motor_state.error = 0;
    motor_state.enable = 0; motor_state.pos = 0; motor_state.target = 0;
    motor_state.speed = 0; motor_state.rpm = 0; motor_state.acc_ms = 0;
    motor_state.seq = 0; motor_state.last_result = HAL_OK;
    motor_debug.uart_error = 0; motor_debug.crc_calc = 0; motor_debug.crc_recv = 0;
    motor_debug.crc_ok = 0; motor_debug.last_reg = 0; motor_debug.exception_code = 0;
    motor_debug.last_pos = 0;
}
/* GPIO mode is from IOC; here only DO off */
static void io(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE(); __HAL_RCC_GPIOF_CLK_ENABLE();
    all_off();
}
/* save error */
static HAL_StatusTypeDef fail(HAL_StatusTypeDef r)
{
    motor_state.moving = 0; motor_state.error = 1; motor_state.last_result = r;
    return r;
}
/* percent to rpm */
static uint16_t rpm(uint8_t p)
{
    if (p < LIFT_MIN_PERCENT) p = LIFT_MIN_PERCENT;
    if (p > LIFT_MAX_PERCENT) p = LIFT_MAX_PERCENT;
    return (uint16_t)(((uint32_t)LIFT_MAX_RPM * p) / 100u);
}
/* mm to motor unit. wheel is diameter(mm), not distance */
int32_t Motor_mmToUnit(int32_t mm)
{
    int64_t sign = 1, v = mm, den, unit;
    uint32_t dia = St_Dia();
    if (v < 0) { sign = -1; v = -v; }
    if (dia == 0) dia = WHEEL_DIA_MM;
    if (DRIVE_RATIO_DEN == 0) return 0;
    den = (int64_t)dia * PI_X10000 * (int64_t)DRIVE_RATIO_DEN;
    if (den <= 0) return 0;
    unit = (v * 10000LL * (int64_t)MOTOR_UNIT_PER_TURN *
            (int64_t)DRIVE_RATIO_NUM) / den;
    return (int32_t)(unit * sign);
}
/* target = FRAM zero + distance(mm) */
int32_t Motor_TargetUnit(int32_t mm)
{
    int32_t z = (St_ZeroOk() != 0) ? St_Zero() : 0;
    return z + Motor_mmToUnit(mm);
}
/* init */
void Motor_InitIO(void)
{
    Bus_Rx(); io(); clear_state();
}
/* write 16/32 */
static HAL_StatusTypeDef w16(UART_HandleTypeDef *huart, uint16_t reg, uint16_t val)
{
    HAL_StatusTypeDef r = Bus_Write16(huart, reg, val);
    motor_state.last_result = r; HAL_Delay(10); return r;
}
static HAL_StatusTypeDef w32(UART_HandleTypeDef *huart, uint16_t reg, int32_t val)
{
    HAL_StatusTypeDef r = Bus_Write32(huart, reg, val);
    motor_state.last_result = r; HAL_Delay(10); return r;
}
/* START off/on */
static HAL_StatusTypeDef off(UART_HandleTypeDef *huart)
{
    return w16(huart, R_START, 0);
}
static HAL_StatusTypeDef on(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef r = off(huart);
    if (r == HAL_OK) { HAL_Delay(20); r = w16(huart, R_START, 1); }
    return r;
}
/* stop: START bit off */
static HAL_StatusTypeDef stop(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef r = HAL_OK;
    int32_t p = 0;
    if (huart != NULL) { r = off(huart); (void)Motor_ReadPos(huart, &p); }
    motor_state.moving = 0; motor_state.last_result = r;
    return r;
}
/* estop state */
static uint8_t btn(void)
{
    if (estop_req == 0) return 0;
    motor_state.error = 1; motor_state.moving = 0;
    return 1;
}
/* clear estop */
void Motor_ClearEStop(void)
{
    estop_req = 0; motor_state.error = 0; dox(DO2_PORT, DO2_PIN, 0);
}
/* store current raw pos as software zero */
HAL_StatusTypeDef Motor_Zero(UART_HandleTypeDef *huart)
{
    int32_t now = 0;
    if (Motor_ReadPos(huart, &now) != HAL_OK) return fail(HAL_ERROR);
    St_SetZero(now, 1); St_SetIn(0, 1);
    if (St_Save() != HAL_OK) return fail(HAL_ERROR);
    motor_state.last_result = HAL_OK;
    return HAL_OK;
}
/* DO1 data sheet 28 */
HAL_StatusTypeDef Motor_Do28(UART_HandleTypeDef *huart)
{
    (void)huart;
    if (btn() != 0) return fail(HAL_ERROR);
    pulse(DO1_PORT, DO1_PIN, DO28_MS);
    motor_state.last_result = HAL_OK;
    return HAL_OK;
}
/* power on/off: 실제 데이터 송신은 rs485.c의 Bus_Power()가 담당 */
HAL_StatusTypeDef Motor_Power(UART_HandleTypeDef *huart, uint8_t on)
{
    HAL_StatusTypeDef r;

    if (huart == NULL) return fail(HAL_ERROR);

    if (on != 0u)
    {
        if (btn() != 0) return fail(HAL_ERROR);

        r = Bus_Power(huart, 1);
        if (r == HAL_OK)
        {
            motor_state.enable = 1;
        }
    }
    else
    {
        (void)off(huart);

        r = Bus_Power(huart, 0);
        if (r == HAL_OK)
        {
            motor_state.enable = 0;
            motor_state.setup_ok = 0;
            motor_state.moving = 0;
        }
    }

    motor_state.last_result = r;
    return (r == HAL_OK) ? HAL_OK : fail(r);
}
/* setup position mode */
HAL_StatusTypeDef Motor_Setup(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef r;
    const Pair_t list[] = {
        {R_MODE, V_POS}, {R_START, 0}, {R_SRC, V_SRC},
        {R_RUN, V_RUN}, {R_END, V_END}, {R_WAIT, 0}
    };
    if (huart == NULL) return fail(HAL_ERROR);
    if (btn() != 0) return fail(HAL_ERROR);
    Bus_Rx(); HAL_Delay(50);
    for (uint32_t i = 0; i < (sizeof(list) / sizeof(list[0])); i++)
    {
        r = w16(huart, list[i].reg, list[i].val);
        if (r != HAL_OK) { motor_state.setup_ok = 0; return fail(r); }
    }
    r = Motor_Power(huart, 1);
    if (r != HAL_OK) return r;
    motor_state.setup_ok = 1; motor_state.error = 0; motor_state.last_result = HAL_OK;
    return HAL_OK;
}
/* set move registers */
static HAL_StatusTypeDef move(UART_HandleTypeDef *huart,
                              int32_t target,
                              uint16_t run_rpm,
                              uint16_t acc_ms)
{
    HAL_StatusTypeDef r = off(huart);
    if (r == HAL_OK) { HAL_Delay(50); r = w16(huart, R_TYPE, V_ABS); }
    if (r == HAL_OK) r = w32(huart, R_POS, target);
    if (r == HAL_OK) r = w16(huart, R_SPEED, run_rpm);
    if (r == HAL_OK) r = w16(huart, R_ACC, acc_ms);
    if (r == HAL_OK) r = w16(huart, R_WAIT, 0);
    return r;
}
/* start move */
HAL_StatusTypeDef Motor_Start(UART_HandleTypeDef *huart,
                              int32_t target,
                              uint8_t speed,
                              uint16_t acc_ms)
{
    HAL_StatusTypeDef r;
    uint16_t run_rpm = rpm(speed);
    if (btn() != 0) return fail(HAL_ERROR);
    r = (motor_state.setup_ok == 0) ? Motor_Setup(huart) : HAL_OK;
    if (r == HAL_OK) r = move(huart, target, run_rpm, acc_ms);
    if (r == HAL_OK) r = on(huart);
    if (r != HAL_OK) return fail(r);
    motor_state.target = target; motor_state.speed = speed; motor_state.rpm = run_rpm;
    motor_state.acc_ms = acc_ms; motor_state.moving = 1; motor_state.error = 0;
    motor_state.last_result = HAL_OK;
    return HAL_OK;
}
/* home: 저장된 zero 위치로 이동 */
HAL_StatusTypeDef Motor_Home(UART_HandleTypeDef *huart)
{
    int32_t z = 0;
    int32_t now = 0;
    int32_t gap;
    osStatus_t q;

    if (btn() != 0) return fail(HAL_ERROR);

    if (St_ZeroOk() != 0) z = St_Zero();

    /* 이미 home 위치면 busy 만들지 않고 바로 성공 */
    if (Motor_ReadPos(huart, &now) == HAL_OK)
    {
        gap = now - z;
        if (gap < 0) gap = -gap;

        if (gap <= GAP)
        {
            motor_state.target = z;
            motor_state.moving = 0;
            motor_state.error = 0;
            motor_state.last_result = HAL_OK;
            return HAL_OK;
        }
    }

    /* 실제 이동이 필요할 때만 큐에 넣음 */
    q = Motor_SendMove(z, 10, 1000, 0, 0);
    if (q != osOK) return fail(HAL_ERROR);

    motor_state.target = z;
    motor_state.moving = 1;
    motor_state.error = 0;
    motor_state.last_result = HAL_OK;
    return HAL_OK;
}
/* read position */
HAL_StatusTypeDef Motor_ReadPos(UART_HandleTypeDef *huart, int32_t *pos)
{
    HAL_StatusTypeDef r;
    if (huart == NULL || pos == NULL) { motor_state.last_result = HAL_ERROR; return HAL_ERROR; }
    r = Bus_Read32(huart, R_REAL, pos);
    if (r == HAL_OK) { motor_state.pos = *pos; motor_debug.last_pos = *pos; }
    motor_state.last_result = r;
    return r;
}
/* 일반 정지 */
HAL_StatusTypeDef Motor_Stop(UART_HandleTypeDef *huart)
{
    Motor_ClearHold();
    return stop(huart);
}

/* 이동 중 여부 */
uint8_t Motor_IsBusy(void)
{
    return motor_state.moving;
}

/* Emergency Stop 여부 */
uint8_t Motor_IsEStop(void)
{
    return btn();
}

/* 광센서 등으로 중간 정지 상태인지 확인 */
uint8_t Motor_IsHold(void)
{
    return hold_on;
}

/* 중간 정지된 raw 위치 */
int32_t Motor_HoldPos(void)
{
    return hold_pos;
}

/* 중간 정지 상태 해제 */
void Motor_ClearHold(void)
{
    hold_on = 0;
}
/* queue put */
static osStatus_t put(uint32_t id, int32_t target, uint32_t speed,
                      uint32_t acc_ms, uint32_t start_ms, uint32_t wait_ms)
{
    MotorCommand_t cmd;
    if (MotorQueueHandle == NULL) return osErrorResource;
    cmd.id = id; cmd.target = target; cmd.speed = speed;
    cmd.acc_ms = acc_ms; cmd.start_ms = start_ms; cmd.wait_ms = wait_ms;
    return osMessageQueuePut(MotorQueueHandle, &cmd, 0, 0);
}
/* 이동 명령 넣기 */
osStatus_t Motor_SendMove(int32_t target, uint32_t speed, uint32_t acc_ms,
                          uint32_t start_ms, uint32_t wait_ms)
{
    if (MotorQueueHandle == NULL) return osErrorResource;
    if (btn() != 0) return osErrorResource;
    if (motor_state.moving != 0) return osErrorResource;
    if (osMessageQueueGetCount(MotorQueueHandle) > 0u) return osErrorResource;

    if (speed > LIFT_MAX_PERCENT) speed = LIFT_MAX_PERCENT;
    if (acc_ms > 60000u) acc_ms = 60000u;

    Motor_ClearHold();

    return put(MOTOR_CMD_MOVE, target, speed, acc_ms, start_ms, wait_ms);
}
osStatus_t Motor_SendStop(void)
{
    return put(MOTOR_CMD_STOP, 0, 0, 0, 0, 0);
}
/* moving check */
static uint8_t check(void)
{
    MotorCommand_t cmd;
    if (btn() != 0) { motor_state.moving = 0; motor_state.error = 1; return 1; }
    if (osMessageQueueGet(MotorQueueHandle, &cmd, NULL, 0) != osOK) return 0;
    if (cmd.id == MOTOR_CMD_STOP) { (void)Motor_Stop(&huart5); return 1; }
    return 0;
}
/* 목표 전 중간 정지: 광센서 stop으로 판단 */
static void hold(int32_t now)
{
    (void)off(&huart5);

    hold_on = 1;
    hold_pos = now;

    motor_state.moving = 0;
    motor_state.error = 0;
    motor_state.seq++;
}

/* 목표 정상 도착 */
static void done(void)
{
    Motor_ClearHold();

    motor_state.moving = 0;
    motor_state.error = 0;
    motor_state.seq++;
}

/* 목표 도착 또는 중간 정지 대기 */
static void wait(void)
{
    int32_t now = 0;
    int32_t last = 0;
    int32_t gap;
    int32_t diff;
    uint8_t have = 0;
    uint8_t still = 0;

    for (;;)
    {
        if (check() != 0) return;

        if (Motor_ReadPos(&huart5, &now) == HAL_OK)
        {
            gap = now - motor_state.target;
            if (gap < 0) gap = -gap;

            if (gap <= GAP)
            {
                done();
                return;
            }

            if (have != 0)
            {
                diff = now - last;
                if (diff < 0) diff = -diff;

                if (diff <= STILL_GAP)
                {
                    if (++still >= STILL_CNT)
                    {
                        hold(now);
                        return;
                    }
                }
                else
                {
                    still = 0;
                }
            }

            last = now;
            have = 1;
        }

        osDelay(READ_MS);
    }
}
/* Motor task */
void Motor_TaskRun(void *argument)
{
    MotorCommand_t cmd;
    HAL_StatusTypeDef r;
    (void)argument;
    Motor_InitIO();
    for (;;)
    {
        if (osMessageQueueGet(MotorQueueHandle, &cmd, NULL, 10) != osOK)
        {
            (void)btn(); continue;
        }
        if (cmd.id == MOTOR_CMD_MOVE)
        {
            if (cmd.start_ms > 0) osDelay(cmd.start_ms);
            r = Motor_Start(&huart5, cmd.target, (uint8_t)cmd.speed, (uint16_t)cmd.acc_ms);
            if (r == HAL_OK) wait();
            else { motor_state.moving = 0; motor_state.error = 1; }
            if (cmd.wait_ms > 0) osDelay(cmd.wait_ms);
        }
        else if (cmd.id == MOTOR_CMD_STOP) (void)Motor_Stop(&huart5);
    }
}
