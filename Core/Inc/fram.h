#ifndef FRAM_H
#define FRAM_H

#include "main.h"
#include <stdint.h>

#define FRAM_MB85RS64_SIZE_BYTES      8192U

#define FRAM_STATUS_WPEN              0x80U
#define FRAM_STATUS_BP1               0x08U
#define FRAM_STATUS_BP0               0x04U
#define FRAM_STATUS_WEL               0x02U

HAL_StatusTypeDef Fram_InitPins(void);

HAL_StatusTypeDef Fram_ReadStatus(SPI_HandleTypeDef *hspi,
                                  uint8_t *status);

HAL_StatusTypeDef Fram_WriteEnable(SPI_HandleTypeDef *hspi);

HAL_StatusTypeDef Fram_WriteDisable(SPI_HandleTypeDef *hspi);

HAL_StatusTypeDef Fram_CheckWriteEnableLatch(SPI_HandleTypeDef *hspi,
                                             uint8_t *status_before,
                                             uint8_t *status_after_wren,
                                             uint8_t *status_after_wrdi);

HAL_StatusTypeDef Fram_Read(SPI_HandleTypeDef *hspi,
                            uint16_t addr,
                            uint8_t *data,
                            uint16_t len);

HAL_StatusTypeDef Fram_Write(SPI_HandleTypeDef *hspi,
                             uint16_t addr,
                             const uint8_t *data,
                             uint16_t len);

#endif /* FRAM_H */
