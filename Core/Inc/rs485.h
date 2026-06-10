/*
 * rs485.h
 *
 *  Created on: Jun 8, 2026
 *      Author: HWNOT
 */
#ifndef RS485_H
#define RS485_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * rs485.h
 *
 * 이 파일 하나에서 두 개의 RS485 포트를 관리한다.
 *
 * 1. CLI RS485
 *    - USART6
 *    - RS485_3 direction pin
 *    - printf 출력용
 *
 * 2. Motor RS485
 *    - UART5
 *    - RS485_2 direction pin
 *    - AIMotor Modbus RTU용
 */

/* ------------------------------------------------------------
 * Common init
 * ------------------------------------------------------------ */
void RS485_Init(void);

/* ------------------------------------------------------------
 * CLI RS485
 * ------------------------------------------------------------ */
void CliRs485_SetTx(void);
void CliRs485_SetRx(void);

HAL_StatusTypeDef CliRs485_Write(const uint8_t *data,
                                 uint16_t len,
                                 uint32_t timeout);

/*
 * 기존 코드 호환용 이름.
 * printf retarget에서 사용한다.
 */
#define RS485_SetTx       CliRs485_SetTx
#define RS485_SetRx       CliRs485_SetRx
#define RS485_Transmit    CliRs485_Write

/* ------------------------------------------------------------
 * Motor RS485 / AIMotor Modbus RTU
 * ------------------------------------------------------------ */
void MotorRs485_SetRx(void);

HAL_StatusTypeDef MotorBus_WriteU16(UART_HandleTypeDef *huart,
                                    uint16_t reg,
                                    uint16_t value);

HAL_StatusTypeDef MotorBus_WriteI32(UART_HandleTypeDef *huart,
                                    uint16_t reg,
                                    int32_t value);

HAL_StatusTypeDef MotorBus_ReadI32(UART_HandleTypeDef *huart,
                                   uint16_t reg,
                                   int32_t *value);

/*
 * motor.c에서 쓰는 짧은 이름.
 */
#define Bus_Rx       MotorRs485_SetRx
#define Bus_Write16  MotorBus_WriteU16
#define Bus_Write32  MotorBus_WriteI32
#define Bus_Read32   MotorBus_ReadI32

#ifdef __cplusplus
}
#endif

#endif /* RS485_H */
