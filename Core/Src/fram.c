/*
 * MB85RS64PNF SPI FRAM driver for STM32 HAL.
 *
 * WP and HOLD pins must be HIGH when not used.
 * SPI mode should be mode 0 or mode 3.
 * Start with SPI mode 0 and low speed during bring-up.
 */

#include "fram.h"

#define FRAM_CMD_WREN      0x06U
#define FRAM_CMD_WRDI      0x04U
#define FRAM_CMD_RDSR      0x05U
#define FRAM_CMD_WRSR      0x01U
#define FRAM_CMD_READ      0x03U
#define FRAM_CMD_WRITE     0x02U

static void Fram_CsLow(void)
{
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);
}

static void Fram_CsHigh(void)
{
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef Fram_InitPins(void)
{
    Fram_CsHigh();
    return HAL_OK;
}

static HAL_StatusTypeDef Fram_SendCommand(SPI_HandleTypeDef *hspi, uint8_t cmd)
{
    HAL_StatusTypeDef st;

    if (hspi == NULL)
    {
        return HAL_ERROR;
    }

    Fram_CsLow();
    st = HAL_SPI_Transmit(hspi, &cmd, 1, 100);
    Fram_CsHigh();

    return st;
}

HAL_StatusTypeDef Fram_WriteEnable(SPI_HandleTypeDef *hspi)
{
    return Fram_SendCommand(hspi, FRAM_CMD_WREN);
}

HAL_StatusTypeDef Fram_WriteDisable(SPI_HandleTypeDef *hspi)
{
    return Fram_SendCommand(hspi, FRAM_CMD_WRDI);
}

HAL_StatusTypeDef Fram_ReadStatus(SPI_HandleTypeDef *hspi, uint8_t *status)
{
    HAL_StatusTypeDef st;
    uint8_t cmd = FRAM_CMD_RDSR;
    uint8_t rx = 0x00U;

    if (hspi == NULL || status == NULL)
    {
        return HAL_ERROR;
    }

    Fram_CsLow();

    st = HAL_SPI_Transmit(hspi, &cmd, 1, 100);
    if (st == HAL_OK)
    {
        st = HAL_SPI_Receive(hspi, &rx, 1, 100);
    }

    Fram_CsHigh();

    if (st == HAL_OK)
    {
        *status = rx;
    }

    return st;
}

HAL_StatusTypeDef Fram_CheckWriteEnableLatch(SPI_HandleTypeDef *hspi,
                                             uint8_t *status_before,
                                             uint8_t *status_after_wren,
                                             uint8_t *status_after_wrdi)
{
    HAL_StatusTypeDef st;

    if (hspi == NULL || status_before == NULL ||
        status_after_wren == NULL || status_after_wrdi == NULL)
    {
        return HAL_ERROR;
    }

    st = Fram_ReadStatus(hspi, status_before);
    if (st != HAL_OK)
    {
        return st;
    }

    st = Fram_WriteEnable(hspi);
    if (st != HAL_OK)
    {
        return st;
    }

    st = Fram_ReadStatus(hspi, status_after_wren);
    if (st != HAL_OK)
    {
        return st;
    }

    st = Fram_WriteDisable(hspi);
    if (st != HAL_OK)
    {
        return st;
    }

    st = Fram_ReadStatus(hspi, status_after_wrdi);
    if (st != HAL_OK)
    {
        return st;
    }

    if (((*status_after_wren & FRAM_STATUS_WEL) == 0U) ||
        ((*status_after_wrdi & FRAM_STATUS_WEL) != 0U))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef Fram_Read(SPI_HandleTypeDef *hspi,
                            uint16_t addr,
                            uint8_t *data,
                            uint16_t len)
{
    HAL_StatusTypeDef st;
    uint8_t cmd[3];

    if (hspi == NULL || data == NULL || len == 0U)
    {
        return HAL_ERROR;
    }

    if (((uint32_t)addr + len) > FRAM_MB85RS64_SIZE_BYTES)
    {
        return HAL_ERROR;
    }

    cmd[0] = FRAM_CMD_READ;
    cmd[1] = (uint8_t)(addr >> 8);
    cmd[2] = (uint8_t)(addr & 0xFFU);

    Fram_CsLow();

    st = HAL_SPI_Transmit(hspi, cmd, sizeof(cmd), 100);
    if (st == HAL_OK)
    {
        st = HAL_SPI_Receive(hspi, data, len, 100);
    }

    Fram_CsHigh();

    return st;
}

HAL_StatusTypeDef Fram_Write(SPI_HandleTypeDef *hspi,
                             uint16_t addr,
                             const uint8_t *data,
                             uint16_t len)
{
    HAL_StatusTypeDef st;
    uint8_t cmd[3];

    if (hspi == NULL || data == NULL || len == 0U)
    {
        return HAL_ERROR;
    }

    if (((uint32_t)addr + len) > FRAM_MB85RS64_SIZE_BYTES)
    {
        return HAL_ERROR;
    }

    st = Fram_WriteEnable(hspi);
    if (st != HAL_OK)
    {
        return st;
    }

    cmd[0] = FRAM_CMD_WRITE;
    cmd[1] = (uint8_t)(addr >> 8);
    cmd[2] = (uint8_t)(addr & 0xFFU);

    Fram_CsLow();

    st = HAL_SPI_Transmit(hspi, cmd, sizeof(cmd), 100);
    if (st == HAL_OK)
    {
        st = HAL_SPI_Transmit(hspi, (uint8_t *)data, len, 100);
    }

    Fram_CsHigh();

    /*
     * MB85RS64 normally resets WEL after WRITE.
     * WRDI is harmless and keeps the state explicit.
     */
    (void)Fram_WriteDisable(hspi);

    return st;
}
