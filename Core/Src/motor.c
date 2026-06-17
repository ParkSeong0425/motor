#include "motor.h"
#include "rs485.h"
#include "usart.h"
#include "cmsis_os.h"
#include "storage.h"
#include <stdint.h>

/* AIMotor register */
#define R_MODE      0x0200
#define R_DI1_FUNC  0x0302
#define R_START     0x0305
#define R_SRC       0x0500
#define R_HOME_TRIG 0x051E  /* H05_30 */
#define R_HOME_MODE 0x051F  /* H05_31 */
#define R_RUN       0x1100
#define R_END       0x1101
#define R_TYPE      0x1104
#define R_POS       0x110C
#define R_SPEED     0x110E
#define R_ACC       0x110F
#define R_WAIT      0x1110
#define R_REAL      0x0B07
#define R_PWR       0x3201  /* axis power on/off */

/* AIMotor value */
#define V_POS       1
#define V_SERVO     1
#define V_HOME_DI   32     /* DI1 = Homeing_Start */
#define V_SRC       2
#define V_RUN       0
#define V_END       1
#define V_ABS       1
#define V_HOME_ELEC 2      /* H05_30 = DI electrical home */
#define V_HOME_AUTO 16     /* H05_31 = shortest electrical zero */

/* done check */
#define GAP         5      /* target gap, motor unit */
#define STILL_GAP   1      /* no-change gap, motor unit */
#define STILL_CNT   10     /* 50ms x 10 = 500ms */
#define READ_MS     50
#define PI_X10000   31416LL

/* DI 출력 ON/OFF, 현재 회로는 RESET이 ON */
#define DI_ON   MOTOR_GPIO_ON
#define DI_OFF  MOTOR_GPIO_OFF


#ifndef USER_BTN_Pin
#define USER_BTN_Pin GPIO_PIN_13
#define USER_BTN_GPIO_Port GPIOC
#endif

#define BTN_ON      GPIO_PIN_SET

#define DI1_ON()    HAL_GPIO_WritePin(DI_1_GPIO_Port, DI_1_Pin, MOTOR_GPIO_ON)
#define DI1_OFF()   HAL_GPIO_WritePin(DI_1_GPIO_Port, DI_1_Pin, MOTOR_GPIO_OFF)
#define DI3_ON()    HAL_GPIO_WritePin(DI_3_GPIO_Port, DI_3_Pin, MOTOR_GPIO_ON)
#define DI3_OFF()   HAL_GPIO_WritePin(DI_3_GPIO_Port, DI_3_Pin, MOTOR_GPIO_OFF)
#define BTN_PUSH()  (HAL_GPIO_ReadPin(USER_BTN_GPIO_Port, USER_BTN_Pin) == BTN_ON)

typedef struct { uint16_t reg; uint16_t val; } Pair_t;

MotorDebug_t motor_debug;
MotorState_t motor_state;
static volatile uint8_t estop_req = 0;
static volatile uint8_t estop_reported = 0;

/* DI3 실제 긴급정지 입력 확인 */
static uint8_t di3_on(void)
{
    return (HAL_GPIO_ReadPin(DI_3_GPIO_Port, DI_3_Pin) == DI_ACT) ? 1u : 0u;
}

/* PC13 테스트 버튼: 모터 정지 명령은 보내지 않고 요청 flag만 세움 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_exti_time = 0;
    uint32_t now = HAL_GetTick();

    if (GPIO_Pin == DI_3_btn_Pin)
    {
        if ((now - last_exti_time) > 60u)
        {
            estop_req = 1;
            motor_state.error = 1;
            last_exti_time = now;
        }
    }
}

/* 상태 초기화 */
static void clear(void)
{
    motor_state.setup_ok = 0; motor_state.moving = 0; motor_state.error = 0;
    motor_state.enable = 0; motor_state.pos = 0; motor_state.target = 0;
    motor_state.speed = 0; motor_state.rpm = 0; motor_state.acc_ms = 0;
    motor_state.seq = 0; motor_state.last_result = HAL_OK;
    motor_debug.uart_error = 0; motor_debug.crc_calc = 0; motor_debug.crc_recv = 0;
    motor_debug.crc_ok = 0; motor_debug.last_reg = 0;
    motor_debug.exception_code = 0; motor_debug.last_pos = 0;
}

/* GPIO는 MX_GPIO_Init() 설정을 그대로 사용 */
static void io(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
}


/* 에러 저장 */
static HAL_StatusTypeDef fail(HAL_StatusTypeDef r)
{
    motor_state.moving = 0;
    motor_state.error = 1;
    motor_state.last_result = r;
    return r;
}

/* 0~100%를 rpm으로 변환 */
static uint16_t rpm(uint8_t p)
{
    if (p < LIFT_MIN_PERCENT) p = LIFT_MIN_PERCENT;
    if (p > LIFT_MAX_PERCENT) p = LIFT_MAX_PERCENT;
    return (uint16_t)(((uint32_t)LIFT_MAX_RPM * p) / 100u);
}

/* mm를 모터 내부 unit으로 변환 */
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

/* 모터 버스/GPIO 상태 초기화 */
void Motor_InitIO(void)
{
    Bus_Rx();
    io();
    clear();
}

/* 16bit 레지스터 쓰기 */
static HAL_StatusTypeDef w16(UART_HandleTypeDef *huart, uint16_t reg, uint16_t val)
{
    HAL_StatusTypeDef r = Bus_Write16(huart, reg, val);
    motor_state.last_result = r;
    HAL_Delay(10);
    return r;
}

/* 32bit 레지스터 쓰기 */
static HAL_StatusTypeDef w32(UART_HandleTypeDef *huart, uint16_t reg, int32_t val)
{
    HAL_StatusTypeDef r = Bus_Write32(huart, reg, val);
    motor_state.last_result = r;
    HAL_Delay(10);
    return r;
}

/* START OFF */
static HAL_StatusTypeDef off(UART_HandleTypeDef *huart)
{
    return w16(huart, R_START, 0);
}

/* START 0->1 */
static HAL_StatusTypeDef on(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef r = off(huart);
    if (r == HAL_OK) { HAL_Delay(20); r = w16(huart, R_START, 1); }
    return r;
}

/* STOP 명령 */
static HAL_StatusTypeDef stop(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef r = HAL_OK;
    int32_t pos = 0;

    if (huart != NULL) { r = off(huart); (void)Motor_ReadPos(huart, &pos); }
    motor_state.moving = 0;
    motor_state.last_result = r;
    return r;
}

/* 긴급정지 상태 확인: DI3 또는 PC13 요청 */
static uint8_t btn(void)
{
    if (di3_on() != 0)
    {
        estop_req = 1;
    }

    if (estop_req != 0)
    {
        motor_state.error = 1;
        motor_state.moving = 0;
    }

    return estop_req;
}

/* STM32 내부 긴급정지 flag 해제 */
void Motor_ClearEStop(void)
{
    estop_req = 0;
    estop_reported = 0;
    motor_state.error = 0;
}
/* DI4/DI5는 입력이므로 STM32가 전원 ON/OFF를 직접 누르지 않는다 */
HAL_StatusTypeDef Motor_Power(UART_HandleTypeDef *huart, uint8_t on)
{
    (void)huart;
    (void)on;

    motor_state.last_result = HAL_OK;
    return HAL_OK;
}

/* AIMotor 위치 운전 기본 설정 */
HAL_StatusTypeDef Motor_Setup(UART_HandleTypeDef *huart)
{
    HAL_StatusTypeDef r;
    const Pair_t list[] = {
        {R_MODE, V_POS}, {R_DI1_FUNC, V_SERVO}, {R_START, 0},
        {R_SRC, V_SRC}, {R_RUN, V_RUN}, {R_END, V_END}, {R_WAIT, 0}
    };

    if (huart == NULL) return fail(HAL_ERROR);
    Bus_Rx(); HAL_Delay(50);

    for (uint32_t i = 0; i < (sizeof(list) / sizeof(list[0])); i++)
    {
        r = w16(huart, list[i].reg, list[i].val);
        if (r != HAL_OK) { motor_state.setup_ok = 0; return fail(r); }
    }

    r = Motor_Power(huart, 1);
    if (r != HAL_OK) return r;

    motor_state.setup_ok = 1;
    motor_state.error = 0;
    motor_state.last_result = HAL_OK;
    return HAL_OK;
}

/* DI1은 입력이므로 STM32가 HOME 신호를 직접 출력하지 않는다 */
HAL_StatusTypeDef Motor_Home(UART_HandleTypeDef *huart)
{
    (void)huart;

    if (btn() != 0) return fail(HAL_ERROR);

    motor_state.last_result = HAL_OK;
    return HAL_OK;
}

/* 목표 위치, 속도, 가속 시간 설정 */
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

/* 모터 이동 시작 */
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

/* 현재 위치 읽기 */
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
    return stop(huart);
}

/* 이동 중인지 확인 */
uint8_t Motor_IsBusy(void)
{
    return motor_state.moving;
}

/* PC13 버튼 긴급정지 상태 */
uint8_t Motor_IsEStop(void)
{
    return btn();
}

/* 큐에 명령 넣기 */
static osStatus_t put(uint32_t id, int32_t target, uint32_t speed,
                      uint32_t acc_ms, uint32_t start_ms, uint32_t wait_ms)
{
    MotorCommand_t cmd;

    if (MotorQueueHandle == NULL) return osErrorResource;
    cmd.id = id; cmd.target = target; cmd.speed = speed;
    cmd.acc_ms = acc_ms; cmd.start_ms = start_ms; cmd.wait_ms = wait_ms;
    return osMessageQueuePut(MotorQueueHandle, &cmd, 0, 0);
}

/* 이동 명령 */
osStatus_t Motor_SendMove(int32_t target, uint32_t speed, uint32_t acc_ms,
                          uint32_t start_ms, uint32_t wait_ms)
{
    if (speed > LIFT_MAX_PERCENT) speed = LIFT_MAX_PERCENT;
    if (acc_ms > 60000u) acc_ms = 60000u;
    return put(MOTOR_CMD_MOVE, target, speed, acc_ms, start_ms, wait_ms);
}

/* 정지 명령 */
osStatus_t Motor_SendStop(void)
{
    return put(MOTOR_CMD_STOP, 0, 0, 0, 0, 0);
}

/* 이동 중 stop/DI3 확인 */
static uint8_t check(void)
{
    MotorCommand_t cmd;

    /*
     * DI3 긴급정지는 모터 드라이버가 하드웨어로 직접 처리한다.
     * STM32는 상태만 반영하고 통신 STOP은 보내지 않는다.
     */
    if (btn() != 0)
    {
        motor_state.moving = 0;
        motor_state.error = 1;
        return 1;
    }

    if (osMessageQueueGet(MotorQueueHandle, &cmd, NULL, 0) != osOK) return 0;

    if (cmd.id == MOTOR_CMD_STOP)
    {
        /*
         * 사용자가 CLI/TCP로 stop 명령을 보냈을 때만 통신 STOP.
         */
        (void)Motor_Stop(&huart5);
        return 1;
    }

    return 0;
}

/* 완료 표시 */
static void done(void)
{
    motor_state.moving = 0;
    motor_state.error = 0;
    motor_state.seq++;
}

/* 목표 근처 도착 또는 정지 상태까지 대기 */
static void wait(void)
{
    int32_t now = 0, last = 0, gap, diff;
    uint8_t have = 0, still = 0;

    for (;;)
    {
        if (check() != 0) return;

        if (Motor_ReadPos(&huart5, &now) == HAL_OK)
        {
            gap = now - motor_state.target;
            if (gap < 0) gap = -gap;
            if (gap <= GAP) { done(); return; }

            if (have != 0)
            {
                diff = now - last;
                if (diff < 0) diff = -diff;
                if (diff <= STILL_GAP)
                {
                    if (++still >= STILL_CNT) { done(); return; }
                }
                else still = 0;
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
            (void)btn();
            continue;
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
