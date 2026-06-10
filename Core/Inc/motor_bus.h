#ifndef MOTOR_BUS_H
#define MOTOR_BUS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

void MotorBus_SetRx(void);

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
 * Old short names.
 * motor.c already uses these names.
 */
#define Bus_Rx       MotorBus_SetRx
#define Bus_Write16  MotorBus_WriteU16
#define Bus_Write32  MotorBus_WriteI32
#define Bus_Read32   MotorBus_ReadI32

#ifdef __cplusplus
}
#endif

#endif /* MOTOR_BUS_H */
