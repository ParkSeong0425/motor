/*
 * net.h
 *
 *  Created on: Jun 10, 2026
 *      Author: HWNOT
 */

#ifndef INC_NET_H_
#define INC_NET_H_

#include "main.h"
#include <stdint.h>

#define MAC_EEPROM_I2C_ADDR_7BIT      0x50U
#define MAC_EEPROM_I2C_ADDR_HAL       (MAC_EEPROM_I2C_ADDR_7BIT << 1)

#define MAC_EEPROM_SIZE_BYTES         256U
#define MAC_EEPROM_EUI48_OFFSET       0xFAU
#define MAC_EEPROM_EUI48_LEN          6U

HAL_StatusTypeDef MacEeprom_IsReady(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef MacEeprom_Read(I2C_HandleTypeDef *hi2c,
                                 uint8_t mem_addr,
                                 uint8_t *data,
                                 uint16_t len);

HAL_StatusTypeDef MacEeprom_ReadByte(I2C_HandleTypeDef *hi2c,
                                     uint8_t mem_addr,
                                     uint8_t *value);

HAL_StatusTypeDef MacEeprom_ReadMac(I2C_HandleTypeDef *hi2c,
                                    uint8_t mac[MAC_EEPROM_EUI48_LEN]);

uint8_t MacEeprom_IsMacPlausible(const uint8_t mac[MAC_EEPROM_EUI48_LEN]);


void Net_TaskRun(void *argument);


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




#endif /* INC_NET_H_ */
