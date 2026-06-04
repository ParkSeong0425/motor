#ifndef MOTOR_BUS_H
#define MOTOR_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

/*
 * motor_bus.h
 *
 * 역할:
 *   AIMotor RS485 / Modbus 통신만 담당한다.
 */

void Bus_Rx(void);

HAL_StatusTypeDef Bus_Write16(
    UART_HandleTypeDef *huart,
    uint16_t reg,
    uint16_t val
);

HAL_StatusTypeDef Bus_Write32(
    UART_HandleTypeDef *huart,
    uint16_t reg,
    int32_t val
);

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_BUS_H */
