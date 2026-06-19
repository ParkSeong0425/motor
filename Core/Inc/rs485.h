#ifndef RS485_H
#define RS485_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

void RS485_Init(void);

void CliRs485_SetTx(void);
void CliRs485_SetRx(void);
HAL_StatusTypeDef CliRs485_Write(const uint8_t *data,
                                 uint16_t len,
                                 uint32_t timeout);

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

HAL_StatusTypeDef MotorBus_Power(UART_HandleTypeDef *huart,
								 uint8_t on);

#define RS485_SetTx       CliRs485_SetTx
#define RS485_SetRx       CliRs485_SetRx
#define RS485_Transmit    CliRs485_Write

#define Bus_Rx            MotorRs485_SetRx
#define Bus_Write16       MotorBus_WriteU16
#define Bus_Write32       MotorBus_WriteI32
#define Bus_Read32        MotorBus_ReadI32

#define Bus_Power MotorBus_Power


#ifdef __cplusplus
}
#endif

#endif
