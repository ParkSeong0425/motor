#include "rs485.h"

#include "usart.h"
#include "gpio.h"
#include "cmsis_os.h"
#include "motor.h"

#include <stdint.h>

/*
 * CLI RS485  : USART6, RS485_3 direction pin, printf output
 * Motor RS485: UART5,  RS485_2 direction pin, AIMotor Modbus RTU
 */

#define CLI_UART_HANDLE      huart6
#define CLI_TX_TIMEOUT_MS    100

#define MOTOR_TX_TIMEOUT_MS  100
#define MOTOR_RX_TIMEOUT_MS  500
#define MOTOR_FRAME_GAP_MS   10

extern osMutexId_t CliprintMutexHandle;
extern osMutexId_t MotorBusMutexHandle;

static uint16_t ModbusCrc(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= buf[i];

        for (uint8_t bit = 0; bit < 8; bit++)
        {
            if ((crc & 1) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

static HAL_StatusTypeDef CheckCrc(const uint8_t *rx, uint16_t len)
{
    uint16_t calc;
    uint16_t recv;

    if (rx == NULL || len < 2)
    {
        motor_debug.crc_ok = 0;
        return HAL_ERROR;
    }

    calc = ModbusCrc(rx, (uint16_t)(len - 2));
    recv = ((uint16_t)rx[len - 1] << 8) | rx[len - 2];

    motor_debug.crc_calc = calc;
    motor_debug.crc_recv = recv;
    motor_debug.crc_ok = (calc == recv) ? 1 : 0;

    return (calc == recv) ? HAL_OK : HAL_ERROR;
}

void CliRs485_SetTx(void)
{
    HAL_GPIO_WritePin(RS485_3_GPIO_Port, RS485_3_Pin, GPIO_PIN_SET);
}

void CliRs485_SetRx(void)
{
    HAL_GPIO_WritePin(RS485_3_GPIO_Port, RS485_3_Pin, GPIO_PIN_RESET);
}

HAL_StatusTypeDef CliRs485_Write(const uint8_t *data,
                                 uint16_t len,
                                 uint32_t timeout)
{
    HAL_StatusTypeDef result;
    uint8_t locked = 0;

    if (data == NULL || len == 0)
    {
        return HAL_ERROR;
    }

    if (CliprintMutexHandle != NULL && osKernelGetState() == osKernelRunning)
    {
        if (osMutexAcquire(CliprintMutexHandle, timeout) == osOK)
        {
            locked = 1;
        }
    }

    CliRs485_SetTx();
    HAL_Delay(1);

    result = HAL_UART_Transmit(&CLI_UART_HANDLE,
                               (uint8_t *)data,
                               len,
                               timeout);

    HAL_Delay(1);
    CliRs485_SetRx();

    if (locked != 0)
    {
        (void)osMutexRelease(CliprintMutexHandle);
    }

    return result;
}

int _write(int file, char *ptr, int len)
{
    (void)file;

    if (ptr == NULL || len <= 0)
    {
        return 0;
    }

    (void)CliRs485_Write((const uint8_t *)ptr,
                         (uint16_t)len,
                         CLI_TX_TIMEOUT_MS);

    return len;
}

static void MotorRs485_SetTx(void)
{
    HAL_GPIO_WritePin(RS485_2_GPIO_Port, RS485_2_Pin, GPIO_PIN_SET);
}

void MotorRs485_SetRx(void)
{
    HAL_GPIO_WritePin(RS485_2_GPIO_Port, RS485_2_Pin, GPIO_PIN_RESET);
}

static void ClearMotorUart(UART_HandleTypeDef *huart)
{
    volatile uint8_t dummy;

    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_CLEAR_FEFLAG(huart);
    __HAL_UART_CLEAR_NEFLAG(huart);
    __HAL_UART_CLEAR_PEFLAG(huart);
    huart->ErrorCode = HAL_UART_ERROR_NONE;

    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
    {
        dummy = (uint8_t)(huart->Instance->DR & 0xFF);
        (void)dummy;
    }

    motor_debug.uart_error = 0;
}

static HAL_StatusTypeDef MotorSendReceive(UART_HandleTypeDef *huart,
                                          uint8_t *tx,
                                          uint16_t tx_len,
                                          uint8_t *rx,
                                          uint16_t rx_len)
{
    HAL_StatusTypeDef result;
    uint8_t locked = 0;

    if (huart == NULL || tx == NULL || rx == NULL)
    {
        motor_state.last_result = HAL_ERROR;
        return HAL_ERROR;
    }

    if (MotorBusMutexHandle != NULL && osKernelGetState() == osKernelRunning)
    {
        if (osMutexAcquire(MotorBusMutexHandle, osWaitForever) == osOK)
        {
            locked = 1;
        }
    }

    ClearMotorUart(huart);

    MotorRs485_SetTx();
    HAL_Delay(1);

    result = HAL_UART_Transmit(huart,
                               tx,
                               tx_len,
                               MOTOR_TX_TIMEOUT_MS);

    if (result == HAL_OK)
    {
        while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET);

        for (volatile uint32_t i = 0; i < 20; i++);
    }

    MotorRs485_SetRx();

    if (result == HAL_OK)
    {
        result = HAL_UART_Receive(huart,
                                  rx,
                                  rx_len,
                                  MOTOR_RX_TIMEOUT_MS);
    }

    HAL_Delay(MOTOR_FRAME_GAP_MS);

    motor_state.last_result = result;
    motor_debug.uart_error = huart->ErrorCode;

    if (locked != 0)
    {
        (void)osMutexRelease(MotorBusMutexHandle);
    }

    return result;
}

HAL_StatusTypeDef MotorBus_WriteU16(UART_HandleTypeDef *huart,
                                    uint16_t reg,
                                    uint16_t value)
{
    uint8_t tx[8];
    uint8_t rx[8];
    uint16_t crc;
    HAL_StatusTypeDef result;

    motor_debug.last_reg = reg;
    motor_debug.exception_code = 0;
    motor_debug.crc_ok = 0;

    tx[0] = (uint8_t)MOTOR_ID;
    tx[1] = 0x06;
    tx[2] = (uint8_t)(reg >> 8);
    tx[3] = (uint8_t)reg;
    tx[4] = (uint8_t)(value >> 8);
    tx[5] = (uint8_t)value;

    crc = ModbusCrc(tx, 6);
    tx[6] = (uint8_t)crc;
    tx[7] = (uint8_t)(crc >> 8);

    result = MotorSendReceive(huart, tx, 8, rx, 8);

    if (result != HAL_OK)
    {
        return result;
    }

    if (rx[0] != (uint8_t)MOTOR_ID)
    {
        return HAL_ERROR;
    }

    if (rx[1] == 0x86)
    {
        motor_debug.exception_code = rx[2];
        return HAL_ERROR;
    }

    if (rx[1] != 0x06 || CheckCrc(rx, 8) != HAL_OK)
    {
        return HAL_ERROR;
    }

    for (uint8_t i = 0; i < 6; i++)
    {
        if (rx[i] != tx[i])
        {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef MotorBus_WriteI32(UART_HandleTypeDef *huart,
                                    uint16_t reg,
                                    int32_t value)
{
    uint8_t tx[13];
    uint8_t rx[8];
    uint16_t crc;
    uint32_t raw = (uint32_t)value;
    uint16_t low_word = (uint16_t)raw;
    uint16_t high_word = (uint16_t)(raw >> 16);
    HAL_StatusTypeDef result;

    motor_debug.last_reg = reg;
    motor_debug.exception_code = 0;
    motor_debug.crc_ok = 0;

    tx[0] = (uint8_t)MOTOR_ID;
    tx[1] = 0x10;
    tx[2] = (uint8_t)(reg >> 8);
    tx[3] = (uint8_t)reg;
    tx[4] = 0x00;
    tx[5] = 0x02;
    tx[6] = 0x04;

    /* AIMotor observed order: low word first, high word second */
    tx[7]  = (uint8_t)(low_word >> 8);
    tx[8]  = (uint8_t)low_word;
    tx[9]  = (uint8_t)(high_word >> 8);
    tx[10] = (uint8_t)high_word;

    crc = ModbusCrc(tx, 11);
    tx[11] = (uint8_t)crc;
    tx[12] = (uint8_t)(crc >> 8);

    result = MotorSendReceive(huart, tx, 13, rx, 8);

    if (result != HAL_OK)
    {
        return result;
    }

    if (rx[0] != (uint8_t)MOTOR_ID)
    {
        return HAL_ERROR;
    }

    if (rx[1] == 0x90)
    {
        motor_debug.exception_code = rx[2];
        return HAL_ERROR;
    }

    if (rx[1] != 0x10 || CheckCrc(rx, 8) != HAL_OK)
    {
        return HAL_ERROR;
    }

    if (rx[2] != tx[2] || rx[3] != tx[3] || rx[4] != 0x00 || rx[5] != 0x02)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef MotorBus_ReadI32(UART_HandleTypeDef *huart,
                                   uint16_t reg,
                                   int32_t *value)
{
    uint8_t tx[8];
    uint8_t rx[9];
    uint16_t crc;
    uint16_t low_word;
    uint16_t high_word;
    uint32_t raw;
    HAL_StatusTypeDef result;

    if (value == NULL)
    {
        motor_state.last_result = HAL_ERROR;
        return HAL_ERROR;
    }

    motor_debug.last_reg = reg;
    motor_debug.exception_code = 0;
    motor_debug.crc_ok = 0;

    tx[0] = (uint8_t)MOTOR_ID;
    tx[1] = 0x03;
    tx[2] = (uint8_t)(reg >> 8);
    tx[3] = (uint8_t)reg;
    tx[4] = 0x00;
    tx[5] = 0x02;

    crc = ModbusCrc(tx, 6);
    tx[6] = (uint8_t)crc;
    tx[7] = (uint8_t)(crc >> 8);

    result = MotorSendReceive(huart, tx, 8, rx, 9);

    if (result != HAL_OK)
    {
        return result;
    }

    if (rx[0] != (uint8_t)MOTOR_ID)
    {
        return HAL_ERROR;
    }

    if (rx[1] == 0x83)
    {
        motor_debug.exception_code = rx[2];
        return HAL_ERROR;
    }

    if (rx[1] != 0x03 || rx[2] != 0x04 || CheckCrc(rx, 9) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* AIMotor observed order: first word = low, second word = high */
    low_word = ((uint16_t)rx[3] << 8) | rx[4];
    high_word = ((uint16_t)rx[5] << 8) | rx[6];

    raw = ((uint32_t)high_word << 16) | low_word;

    *value = (int32_t)raw;
    motor_debug.last_pos = *value;

    return HAL_OK;
}

/* [수정] 모터 Enable ON/OFF를 0x06 Function Code 기반 새 프레임으로 송신 */
HAL_StatusTypeDef MotorBus_Power(UART_HandleTypeDef *huart, uint8_t on)
{
    // 입력받은 정적 헥사 프레임 적용 (8바이트 구조)
    uint8_t p_on[8] =
        {0x01, 0x06, 0x03, 0x03, 0x00, 0x01, 0xB8, 0x4E};

    uint8_t p_off[8] =
        {0x01, 0x06, 0x03, 0x03, 0x00, 0x00, 0x79, 0x8E};

    uint8_t rx[8];
    uint8_t *tx;
    HAL_StatusTypeDef r;

    motor_debug.last_reg = 0x0303;
    motor_debug.exception_code = 0;
    motor_debug.crc_ok = 0;

    tx = (on != 0u) ? p_on : p_off;

    // 송신 패킷 크기를 11에서 8바이트로, 수신 버퍼 크기를 8바이트로 세팅
    r = MotorSendReceive(huart, tx, 8, rx, 8);
    if (r != HAL_OK) return r;

    if (rx[0] != 0x01) return HAL_ERROR;

    // 모터의 예외 응답(0x86) 처리 예외 규격 검증
    if (rx[1] == 0x86)
    {
        motor_debug.exception_code = rx[2];
        return HAL_ERROR;
    }

    // Function Code가 0x06이 맞는지 및 CRC 유효성 검사
    if (rx[1] != 0x06 || CheckCrc(rx, 8) != HAL_OK)
    {
        return HAL_ERROR;
    }

    // 0x06 스펙에 따라 응답 프레임이 송신 에코 데이터와 완벽히 일치하는지 확인 (앞 6바이트)
    for (uint8_t i = 0; i < 6; i++)
    {
        if (rx[i] != tx[i])
        {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

void RS485_Init(void)
{
    CliRs485_SetRx();
    MotorRs485_SetRx();

    __HAL_UART_CLEAR_OREFLAG(&huart6);
    __HAL_UART_CLEAR_NEFLAG(&huart6);
    __HAL_UART_CLEAR_FEFLAG(&huart6);
    __HAL_UART_CLEAR_PEFLAG(&huart6);

    __HAL_UART_CLEAR_OREFLAG(&huart5);
    __HAL_UART_CLEAR_NEFLAG(&huart5);
    __HAL_UART_CLEAR_FEFLAG(&huart5);
    __HAL_UART_CLEAR_PEFLAG(&huart5);
}
