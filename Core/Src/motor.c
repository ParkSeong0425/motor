#include "motor.h"
#include "rs485.h"
#include "usart.h"
#include "cmsis_os.h"
#include "storage.h"
#include <stdint.h>

/* AIMotor 레지스터 정의 */
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

/* 판단 제어 상수 */
#define GAP 5
#define STILL_GAP 1
#define STILL_CNT 10
#define READ_MS 50
#define PI_X10000 31416LL

/* DO 핀 맵 구성 */
#define DO0_PORT GPIOA      /* PA3: Spare */
#define DO0_PIN  GPIO_PIN_3
#define DO1_PORT GPIOC      /* PC0: Data Sheet 28 */
#define DO1_PIN  GPIO_PIN_0
#define DO2_PORT GPIOC      /* PC3: 비상정지(E-Stop) */
#define DO2_PIN  GPIO_PIN_3
#define DO3_PORT GPIOF      /* PF3: Enable ON */
#define DO3_PIN  GPIO_PIN_3
#define DO4_PORT GPIOF      /* PF5: Enable OFF */
#define DO4_PIN  GPIO_PIN_5

MotorDebug_t motor_debug;
MotorState_t motor_state;
static volatile uint8_t estop_req = 0;
static volatile uint8_t hold_on = 0; /* 1이면 광센서/손에 의한 임시 원점 모드 활성화 */
static int32_t hold_pos = 0; /* 손으로 멈춘 시점의 임시 원점 좌표(Raw) */

/* DO 출력 제어 공통 함수 */
static void dox(GPIO_TypeDef *port, uint16_t pin, uint8_t on) {
	HAL_GPIO_WritePin(port, pin, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}
static void all_off(void) {
	dox(DO0_PORT, DO0_PIN, 0);
	dox(DO1_PORT, DO1_PIN, 0);
	dox(DO2_PORT, DO2_PIN, 0);
	dox(DO3_PORT, DO3_PIN, 0);
	dox(DO4_PORT, DO4_PIN, 0);
}

/* PC13 외부 인터럽트: 하드웨어 비상정지 동기화 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	static uint32_t last = 0;
	uint32_t now = HAL_GetTick();
	if (GPIO_Pin != GPIO_PIN_13 || (now - last) <= 60u)
		return;
	HAL_GPIO_TogglePin(DO2_PORT, DO2_PIN);
	estop_req = (HAL_GPIO_ReadPin(DO2_PORT, DO2_PIN) == GPIO_PIN_SET);
	motor_state.error = estop_req;
	motor_state.moving = 0;
	last = now;
}

/* 상태 초기화 및 에러 핸들러 */
static void clear_state(void) {
	motor_state = (MotorState_t ) { 0 };
	motor_debug = (MotorDebug_t ) { 0 };
}
static HAL_StatusTypeDef fail(HAL_StatusTypeDef r) {
	motor_state.moving = 0;
	motor_state.error = 1;
	motor_state.last_result = r;
	return r;
}
static uint8_t btn(void) {
	if (estop_req) {
		motor_state.error = 1;
		motor_state.moving = 0;
		return 1;
	}
	return 0;
}

/* 레지스터 쓰기 래퍼 함수 */
static HAL_StatusTypeDef w16(UART_HandleTypeDef *huart, uint16_t reg,
		uint16_t val) {
	HAL_StatusTypeDef r = Bus_Write16(huart, reg, val);
	motor_state.last_result = r;
	HAL_Delay(10);
	return r;
}
static HAL_StatusTypeDef w32(UART_HandleTypeDef *huart, uint16_t reg,
		int32_t val) {
	HAL_StatusTypeDef r = Bus_Write32(huart, reg, val);
	motor_state.last_result = r;
	HAL_Delay(10);
	return r;
}
static HAL_StatusTypeDef set_start(UART_HandleTypeDef *huart, uint16_t val) {
	return w16(huart, R_START, val);
}

/* 단위 변환 함수 */
int32_t Motor_mmToUnit(int32_t mm) {
	int64_t sign = (mm < 0) ? -1 : 1;
	int64_t v = (mm < 0) ? -mm : mm;
	uint32_t dia = St_Dia() ? St_Dia() : WHEEL_DIA_MM;
	if (DRIVE_RATIO_DEN == 0)
		return 0;
	int64_t den = (int64_t) dia * PI_X10000 * (int64_t) DRIVE_RATIO_DEN;
	if (den <= 0)
		return 0;
	return (int32_t) (((v * 10000LL * MOTOR_UNIT_PER_TURN * DRIVE_RATIO_NUM)
			/ den) * sign);
}

/* [★핵심 변경] 목표 위치 계산 시 임시 원점 모드(hold_on)면 그 위치를 기준으로 계산 */
int32_t Motor_TargetUnit(int32_t mm) {
	int32_t base = hold_on ? hold_pos : (St_ZeroOk() ? St_Zero() : 0);
	return base + Motor_mmToUnit(mm);
}

/* 기본 제어 및 상태 정보 제공 API */
void Motor_InitIO(void) {
	Bus_Rx();
	all_off();
	clear_state();
}
void Motor_ClearEStop(void) {
	estop_req = 0;
	motor_state.error = 0;
	dox(DO2_PORT, DO2_PIN, 0);
}
uint8_t Motor_IsBusy(void) {
	return motor_state.moving;
}
uint8_t Motor_IsEStop(void) {
	return btn();
}
uint8_t Motor_IsHold(void) {
	return hold_on;
}
int32_t Motor_HoldPos(void) {
	return hold_pos;
}
void Motor_ClearHold(void) {
	hold_on = 0;
}

/* 위치 읽기 */
HAL_StatusTypeDef Motor_ReadPos(UART_HandleTypeDef *huart, int32_t *pos) {
	if (!huart || !pos)
		return motor_state.last_result = HAL_ERROR;
	HAL_StatusTypeDef r = Bus_Read32(huart, R_REAL, pos);
	if (r == HAL_OK)
		motor_state.pos = motor_debug.last_pos = *pos;
	return motor_state.last_result = r;
}

/* 모터 정지 */
HAL_StatusTypeDef Motor_Stop(UART_HandleTypeDef *huart) {
	Motor_ClearHold();
	HAL_StatusTypeDef r = set_start(huart, 0);
	int32_t p = 0;
	(void) Motor_ReadPos(huart, &p);
	motor_state.moving = 0;
	return motor_state.last_result = r;
}

/* 현재 위치를 실제 원점으로 저장 */
HAL_StatusTypeDef Motor_Zero(UART_HandleTypeDef *huart) {
	int32_t now = 0;
	if (Motor_ReadPos(huart, &now) != HAL_OK)
		return fail(HAL_ERROR);

	St_SetZero(now, 1); // 반환값이 없으므로 그냥 호출

	// 이후 저장(Save) 과정이 성공했는지 체크하는 것이 논리적으로 맞습니다.
	if (St_Save() != HAL_OK)
		return fail(HAL_ERROR);

	return HAL_OK;
}

/* 전원 제어 */
HAL_StatusTypeDef Motor_Power(UART_HandleTypeDef *huart, uint8_t on) {
	if (!huart)
		return fail(HAL_ERROR);
	if (on) {
		if (btn())
			return fail(HAL_ERROR);
		if (Bus_Power(huart, 1) == HAL_OK)
			motor_state.enable = 1;
	} else {
		set_start(huart, 0);
		if (Bus_Power(huart, 0) == HAL_OK) {
			motor_state.enable = 0;
			motor_state.setup_ok = 0;
			motor_state.moving = 0;
		}
	}
	return motor_state.enable == on ? HAL_OK : fail(HAL_ERROR);
}

/* 드라이버 포지션 모드 셋업 */
HAL_StatusTypeDef Motor_Setup(UART_HandleTypeDef *huart) {
	if (!huart || btn())
		return fail(HAL_ERROR);
	Bus_Rx();
	HAL_Delay(50);
	if (w16(huart, R_MODE, V_POS) != HAL_OK || set_start(huart, 0) != HAL_OK
			|| w16(huart, R_SRC, V_SRC) != HAL_OK
			|| w16(huart, R_RUN, V_RUN) != HAL_OK
			|| w16(huart, R_END, V_END) != HAL_OK
			|| w16(huart, R_WAIT, 0) != HAL_OK) {
		motor_state.setup_ok = 0;
		return fail(HAL_ERROR);
	}
	return Motor_Power(huart, 1) == HAL_OK ?
			(motor_state.setup_ok = 1, motor_state.error = 0, HAL_OK) :
			fail(HAL_ERROR);
}

/* 모터 주행 시작 파라미터 주입 */
HAL_StatusTypeDef Motor_Start(UART_HandleTypeDef *huart, int32_t target,
		uint8_t speed, uint16_t acc_ms) {
	if (btn())
		return fail(HAL_ERROR);
	if (!motor_state.setup_ok && Motor_Setup(huart) != HAL_OK)
		return fail(HAL_ERROR);

	uint16_t run_rpm =
			(speed < LIFT_MIN_PERCENT) ?
					LIFT_MIN_PERCENT :
					(speed > LIFT_MAX_PERCENT ? LIFT_MAX_PERCENT : speed);
	run_rpm = (uint16_t) (((uint32_t) LIFT_MAX_RPM * run_rpm) / 100u);

	if (set_start(huart, 0) != HAL_OK)
		return fail(HAL_ERROR);
	HAL_Delay(50);
	if (w16(huart, R_TYPE, V_ABS) != HAL_OK
			|| w32(huart, R_POS, target) != HAL_OK
			|| w16(huart, R_SPEED, run_rpm) != HAL_OK
			|| w16(huart, R_ACC, acc_ms) != HAL_OK
			|| w16(huart, R_WAIT, 0) != HAL_OK || set_start(huart, 1) != HAL_OK)
		return fail(HAL_ERROR);

	motor_state.target = target;
	motor_state.speed = speed;
	motor_state.rpm = run_rpm;
	motor_state.acc_ms = acc_ms;
	motor_state.moving = 1;
	motor_state.error = 0;
	return HAL_OK;
}

/* 절대 원점 복귀 명령 (손을 떼거나 홈 복귀 시 호출) */
HAL_StatusTypeDef Motor_Home(UART_HandleTypeDef *huart) {
	if (btn())
		return fail(HAL_ERROR);
	Motor_ClearHold(); /* 홈 복귀 명령 시 임시 원점을 즉시 해제하여 진짜 원점으로 복귀하도록 유도 */
	int32_t z = St_ZeroOk() ? St_Zero() : 0, now = 0;
	int32_t gap = (Motor_ReadPos(huart, &now) == HAL_OK) ? (now - z) : 999;
	if (gap < 0)
		gap = -gap;
	if (gap <= GAP) {
		motor_state.target = z;
		motor_state.moving = 0;
		motor_state.error = 0;
		return HAL_OK;
	}

	if (Motor_SendMove(z, 10, 1000, 0, 0) != osOK)
		return fail(HAL_ERROR);
	motor_state.target = z;
	motor_state.moving = 1;
	motor_state.error = 0;
	return HAL_OK;
}

/* OS 메시지 큐 통신 제어 */
static osStatus_t put(uint32_t id, int32_t target, uint32_t speed,
		uint32_t acc_ms, uint32_t start_ms, uint32_t wait_ms) {
	if (!MotorQueueHandle)
		return osErrorResource;
	MotorCommand_t cmd = { id, target, speed, acc_ms, start_ms, wait_ms };
	return osMessageQueuePut(MotorQueueHandle, &cmd, 0, 0);
}
osStatus_t Motor_SendMove(int32_t target, uint32_t speed, uint32_t acc_ms,
		uint32_t start_ms, uint32_t wait_ms) {
	if (!MotorQueueHandle || btn() || motor_state.moving
			|| osMessageQueueGetCount(MotorQueueHandle) > 0)
		return osErrorResource;
	if (speed > LIFT_MAX_PERCENT)
		speed = LIFT_MAX_PERCENT;
	if (acc_ms > 60000u)
		acc_ms = 60000u;
	return put(MOTOR_CMD_MOVE, target, speed, acc_ms, start_ms, wait_ms);
}
osStatus_t Motor_SendStop(void) {
	return put(MOTOR_CMD_STOP, 0, 0, 0, 0, 0);
}

/* 주행 완료 및 실시간 감속/정지 상태 모니터링 루프 */
static void wait(void) {
	int32_t now = 0, last = 0, still = 0, gap = 0, diff = 0;
	uint8_t have = 0;
	uint32_t start_tick = HAL_GetTick();
	for (;;) {
		MotorCommand_t cmd;
		if (btn()) {
			motor_state.moving = 0;
			motor_state.error = 1;
			return;
		}
		if (osMessageQueueGet(MotorQueueHandle, &cmd, NULL, 0) == osOK
				&& cmd.id == MOTOR_CMD_STOP) {
			(void) Motor_Stop(&huart5);
			return;
		}

		if (Motor_ReadPos(&huart5, &now) == HAL_OK) {
			gap = now - motor_state.target;
			if (gap < 0)
				gap = -gap;
			if (gap <= GAP) {
				Motor_ClearHold();
				motor_state.moving = 0;
				motor_state.seq++;
				return;
			} // 정상 도착 시 홀드 해제

			if (have) {
				diff = now - last;
				if (diff < 0)
					diff = -diff;
				if (diff <= STILL_GAP) {
					/* 초기 가속구간 1500ms 예외 차단 후 정지 카운트 누적 */
					if ((HAL_GetTick() - start_tick > 1500u)
							&& (++still >= STILL_CNT)) {
						set_start(&huart5, 0); // 모터 정지 프레임 송신
						hold_on = 1;
						hold_pos = now; // [★핵심 반영] 현재 걸린 위치를 임시 원점으로 박제
						motor_state.moving = 0;
						motor_state.seq++;
						return;
					}
				} else {
					still = 0;
				}
			}
			last = now;
			have = 1;
		}
		osDelay(READ_MS);
	}
}

/* FreeRTOS 모터 연동 태스크 주루프 */
void Motor_TaskRun(void *argument) {
	MotorCommand_t cmd;
	(void) argument;
	Motor_InitIO();
	for (;;) {
		if (osMessageQueueGet(MotorQueueHandle, &cmd, NULL, 10) != osOK) {
			(void) btn();
			continue;
		}
		if (cmd.id == MOTOR_CMD_MOVE) {
			if (cmd.start_ms > 0)
				osDelay(cmd.start_ms);
			if (Motor_Start(&huart5, cmd.target, (uint8_t) cmd.speed,
					(uint16_t) cmd.acc_ms) == HAL_OK)
				wait();
			else {
				motor_state.moving = 0;
				motor_state.error = 1;
			}
			if (cmd.wait_ms > 0)
				osDelay(cmd.wait_ms);
		} else if (cmd.id == MOTOR_CMD_STOP)
			(void) Motor_Stop(&huart5);
	}
}
