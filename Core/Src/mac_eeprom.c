#include "mac_eeprom.h"

HAL_StatusTypeDef MacEeprom_IsReady(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_IsDeviceReady(hi2c,
                                 MAC_EEPROM_I2C_ADDR_HAL,
                                 3,
                                 100);
}

HAL_StatusTypeDef MacEeprom_Read(I2C_HandleTypeDef *hi2c,
                                 uint8_t mem_addr,
                                 uint8_t *data,
                                 uint16_t len)
{
    if (hi2c == NULL || data == NULL || len == 0U)
    {
        return HAL_ERROR;
    }

    if (((uint16_t)mem_addr + len) > MAC_EEPROM_SIZE_BYTES)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(hi2c,
                            MAC_EEPROM_I2C_ADDR_HAL,
                            mem_addr,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            len,
                            100);
}

HAL_StatusTypeDef MacEeprom_ReadByte(I2C_HandleTypeDef *hi2c,
                                     uint8_t mem_addr,
                                     uint8_t *value)
{
    return MacEeprom_Read(hi2c, mem_addr, value, 1U);
}

HAL_StatusTypeDef MacEeprom_ReadMac(I2C_HandleTypeDef *hi2c,
                                    uint8_t mac[MAC_EEPROM_EUI48_LEN])
{
    return MacEeprom_Read(hi2c,
                          MAC_EEPROM_EUI48_OFFSET,
                          mac,
                          MAC_EEPROM_EUI48_LEN);
}

uint8_t MacEeprom_IsMacPlausible(const uint8_t mac[MAC_EEPROM_EUI48_LEN])
{
    uint8_t all_00 = 1U;
    uint8_t all_ff = 1U;

    if (mac == NULL)
    {
        return 0U;
    }

    for (uint8_t i = 0; i < MAC_EEPROM_EUI48_LEN; i++)
    {
        if (mac[i] != 0x00U)
        {
            all_00 = 0U;
        }

        if (mac[i] != 0xFFU)
        {
            all_ff = 0U;
        }
    }

    if (all_00 || all_ff)
    {
        return 0U;
    }

    return 1U;
}
