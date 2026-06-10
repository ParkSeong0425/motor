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

void RS485_Init(void);
void RS485_SetTx(void);
void RS485_SetRx(void);

HAL_StatusTypeDef RS485_Transmit(const uint8_t *data,
                                 uint16_t len,
                                 uint32_t timeout);

#ifdef __cplusplus
}
#endif

#endif /* RS485_H */
