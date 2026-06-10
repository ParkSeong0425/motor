#ifndef MAC_EEPROM_H
#define MAC_EEPROM_H

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

#endif /* MAC_EEPROM_H */
