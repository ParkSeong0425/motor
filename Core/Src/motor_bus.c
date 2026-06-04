#include "motor_bus.h"
#include "motor.h"
#include <string.h>

/*
 * motor_bus.c
 *
 * RS485 방향 제어, UART 송수신, Modbus CRC,
 * 16bit 쓰기, 32bit 쓰기, 32bit 읽기를 담당한다.
 */

#define RS485_DIR_PORT              GPIOD
#define RS485_DIR_PIN               GPIO_PIN_13

#define RS485_TX                    GPIO_PIN_SET
#define RS485_RX                    GPIO_PIN_RESET

#define BUS_TX_TIMEOUT_MS           100U
#define BUS_RX_TIMEOUT_MS           500U
#define BUS_TC_TIMEOUT_MS           100U
#define BUS_WRITE_DELAY_MS          20U

static void Bus_Tx(void)
{
    HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, RS485_TX);
}

void Bus_Rx(void)
{
    HAL_GPIO_WritePin(RS485_DIR_PORT, RS485_DIR_PIN, RS485_RX);
}

static void Bus_ClearErr(UART_HandleTypeDef *huart)
{
    if (huart == NULL)
    {
        return;
    }

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    motor_debug.uart_error = 0U;
}

static void Bus_ClearRx(UART_HandleTypeDef *huart)
{
    uint8_t dummy;

    if (huart == NULL)
    {
        return;
    }

    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
    {
        dummy = (uint8_t)(huart->Instance->DR & 0xFFU);
        (void)dummy;
    }
}

static uint16_t Bus_Crc(const uint8_t *buf, uint16_t len)
{
    uint16_t crc;
    uint16_t i;
    uint8_t bit;

    crc = 0xFFFFU;

    for (i = 0U; i < len; i++)
    {
        crc ^= (uint16_t)buf[i];

        for (bit = 0U; bit < 8U; bit++)
        {
            if ((crc & 0x0001U) != 0U)
            {
                crc >>= 1U;
                crc ^= 0xA001U;
            }
            else
            {
                crc >>= 1U;
            }
        }
    }

    return crc;
}

static HAL_StatusTypeDef Bus_CheckCrc(const uint8_t *rx, uint16_t len)
{
    uint16_t calc;
    uint16_t recv;

    if ((rx == NULL) || (len < 2U))
    {
        motor_debug.crc_ok = 0U;
        return HAL_ERROR;
    }

    calc = Bus_Crc(rx, (uint16_t)(len - 2U));
    recv = ((uint16_t)rx[len - 1U] << 8U) | rx[len - 2U];

    motor_debug.crc_calc = calc;
    motor_debug.crc_recv = recv;

    if (calc != recv)
    {
        motor_debug.crc_ok = 0U;
        return HAL_ERROR;
    }

    motor_debug.crc_ok = 1U;
    return HAL_OK;
}

static HAL_StatusTypeDef Bus_WaitTxDone(UART_HandleTypeDef *huart)
{
    uint32_t tick;

    if (huart == NULL)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    tick = HAL_GetTick();

    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET)
    {
        if ((HAL_GetTick() - tick) > BUS_TC_TIMEOUT_MS)
        {
            motor_debug.uart_error = huart->ErrorCode;
            motor_state.last_hal = HAL_TIMEOUT;
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef Bus_SendRead(
    UART_HandleTypeDef *huart,
    uint8_t *tx,
    uint16_t tx_len,
    uint8_t *rx,
    uint16_t rx_len
)
{
    HAL_StatusTypeDef st;

    if ((huart == NULL) || (tx == NULL) || (rx == NULL))
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    Bus_ClearRx(huart);
    Bus_ClearErr(huart);

    Bus_Tx();
    HAL_Delay(1U);

    st = HAL_UART_Transmit(huart, tx, tx_len, BUS_TX_TIMEOUT_MS);

    if (st == HAL_OK)
    {
        st = Bus_WaitTxDone(huart);
    }

    Bus_Rx();

    if (st != HAL_OK)
    {
        motor_state.last_hal = st;
        motor_debug.uart_error = huart->ErrorCode;
        return st;
    }

    memset(rx, 0, rx_len);

    st = HAL_UART_Receive(huart, rx, rx_len, BUS_RX_TIMEOUT_MS);

    motor_state.last_hal = st;
    motor_debug.uart_error = huart->ErrorCode;

    return st;
}

HAL_StatusTypeDef Bus_Write16(
    UART_HandleTypeDef *huart,
    uint16_t reg,
    uint16_t val
)
{
    uint8_t tx[8];
    uint8_t rx[8];
    uint16_t crc;
    HAL_StatusTypeDef st;
    uint8_t i;

    motor_debug.last_reg = reg;
    motor_debug.exception_code = 0U;
    motor_debug.crc_ok = 0U;

    tx[0] = (uint8_t)MOTOR_ID;
    tx[1] = 0x06U;
    tx[2] = (uint8_t)((reg >> 8U) & 0xFFU);
    tx[3] = (uint8_t)(reg & 0xFFU);
    tx[4] = (uint8_t)((val >> 8U) & 0xFFU);
    tx[5] = (uint8_t)(val & 0xFFU);

    crc = Bus_Crc(tx, 6U);
    tx[6] = (uint8_t)(crc & 0xFFU);
    tx[7] = (uint8_t)((crc >> 8U) & 0xFFU);

    st = Bus_SendRead(huart, tx, 8U, rx, 8U);

    if (st != HAL_OK)
    {
        motor_state.last_hal = st;
        return st;
    }

    if (rx[0] != (uint8_t)MOTOR_ID)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    if (rx[1] == 0x86U)
    {
        motor_debug.exception_code = rx[2];
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    if (rx[1] != 0x06U)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    if (Bus_CheckCrc(rx, 8U) != HAL_OK)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    for (i = 0U; i < 6U; i++)
    {
        if (rx[i] != tx[i])
        {
            motor_state.last_hal = HAL_ERROR;
            return HAL_ERROR;
        }
    }

    motor_state.last_hal = HAL_OK;
    HAL_Delay(BUS_WRITE_DELAY_MS);

    return HAL_OK;
}

HAL_StatusTypeDef Bus_Write32(
    UART_HandleTypeDef *huart,
    uint16_t reg,
    int32_t val
)
{
    uint8_t tx[13];
    uint8_t rx[8];
    uint16_t crc;
    uint32_t raw;
    HAL_StatusTypeDef st;

    raw = (uint32_t)val;

    motor_debug.last_reg = reg;
    motor_debug.last_pos = val;
    motor_debug.exception_code = 0U;
    motor_debug.crc_ok = 0U;

    tx[0] = (uint8_t)MOTOR_ID;
    tx[1] = 0x10U;
    tx[2] = (uint8_t)((reg >> 8U) & 0xFFU);
    tx[3] = (uint8_t)(reg & 0xFFU);
    tx[4] = 0x00U;
    tx[5] = 0x02U;
    tx[6] = 0x04U;

    /*
     * 기존 실기 동작 기준:
     * low word 먼저, high word 나중.
     */
    tx[7]  = (uint8_t)((raw >> 8U) & 0xFFU);
    tx[8]  = (uint8_t)(raw & 0xFFU);
    tx[9]  = (uint8_t)((raw >> 24U) & 0xFFU);
    tx[10] = (uint8_t)((raw >> 16U) & 0xFFU);

    crc = Bus_Crc(tx, 11U);
    tx[11] = (uint8_t)(crc & 0xFFU);
    tx[12] = (uint8_t)((crc >> 8U) & 0xFFU);

    st = Bus_SendRead(huart, tx, 13U, rx, 8U);

    if (st != HAL_OK)
    {
        motor_state.last_hal = st;
        return st;
    }

    if (rx[0] != (uint8_t)MOTOR_ID)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    if (rx[1] == 0x90U)
    {
        motor_debug.exception_code = rx[2];
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    if (rx[1] != 0x10U)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    if (Bus_CheckCrc(rx, 8U) != HAL_OK)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    if (rx[2] != ((reg >> 8U) & 0xFFU))
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    if (rx[3] != (reg & 0xFFU))
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    if (rx[4] != 0x00U)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    if (rx[5] != 0x02U)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    motor_state.last_hal = HAL_OK;
    HAL_Delay(BUS_WRITE_DELAY_MS);

    return HAL_OK;
}

HAL_StatusTypeDef Bus_Read32(
    UART_HandleTypeDef *huart,
    uint16_t reg,
    int32_t *val
)
{
    uint8_t tx[8];
    uint8_t rx[9];
    uint16_t crc;
    uint16_t low_word;
    uint16_t high_word;
    uint32_t raw;
    HAL_StatusTypeDef st;

    if (val == NULL)
    {
        return HAL_ERROR;
    }

    motor_debug.last_reg = reg;
    motor_debug.exception_code = 0U;
    motor_debug.crc_ok = 0U;

    tx[0] = (uint8_t)MOTOR_ID;
    tx[1] = 0x03U;
    tx[2] = (uint8_t)((reg >> 8U) & 0xFFU);
    tx[3] = (uint8_t)(reg & 0xFFU);
    tx[4] = 0x00U;
    tx[5] = 0x02U;

    crc = Bus_Crc(tx, 6U);
    tx[6] = (uint8_t)(crc & 0xFFU);
    tx[7] = (uint8_t)((crc >> 8U) & 0xFFU);

    st = Bus_SendRead(huart, tx, 8U, rx, 9U);

    if (st != HAL_OK)
    {
        motor_state.last_hal = st;
        return st;
    }

    if (rx[0] != (uint8_t)MOTOR_ID)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    if (rx[1] == 0x83U)
    {
        motor_debug.exception_code = rx[2];
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    if (rx[1] != 0x03U)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    if (rx[2] != 0x04U)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    if (Bus_CheckCrc(rx, 9U) != HAL_OK)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    low_word  = ((uint16_t)rx[3] << 8U) | rx[4];
    high_word = ((uint16_t)rx[5] << 8U) | rx[6];

    raw = ((uint32_t)high_word << 16U) | low_word;

    *val = (int32_t)raw;

    motor_state.last_hal = HAL_OK;
    return HAL_OK;
}
