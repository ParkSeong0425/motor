#include "rs485.h"

#include "usart.h"
#include "gpio.h"
#include "cmsis_os.h"
#include "motor.h"

#include <string.h>
#include <unistd.h>
#include <stdio.h>

/*
 * rs485.c
 *
 * 이 파일 하나에서 RS485 관련 기능을 모두 처리한다.
 *
 * ------------------------------------------------------------
 * 1. CLI RS485
 * ------------------------------------------------------------
 * USART6 : CLI terminal
 * PF12   : RS485_3 direction pin
 *
 * printf() 출력은 여기 있는 _write()를 통해 CLI RS485로 나간다.
 *
 * ------------------------------------------------------------
 * 2. Motor RS485
 * ------------------------------------------------------------
 * UART5  : AIMotor Modbus RTU
 * PD13   : RS485_2 direction pin
 *
 * motor.c는 Bus_Write16 / Bus_Write32 / Bus_Read32 이름으로
 * 이 파일의 Modbus 함수를 사용한다.
 */

/* ============================================================
 * CLI RS485 section
 * ============================================================ */

#define CLI_UART_HANDLE              huart6
#define CLI_TX_TIMEOUT_MS            100U

extern osMutexId_t CliprintMutexHandle;

static uint8_t CliNeedMutex(void)
{
    if (CliprintMutexHandle == NULL)
    {
        return 0U;
    }

    if (osKernelGetState() != osKernelRunning)
    {
        return 0U;
    }

    return 1U;
}

static void ShortDelay(void)
{
    for (volatile uint32_t i = 0U; i < 300U; i++)
    {
        __NOP();
    }
}

void CliRs485_SetTx(void)
{
    HAL_GPIO_WritePin(RS485_3_GPIO_Port, RS485_3_Pin, GPIO_PIN_SET);
    ShortDelay();
}

void CliRs485_SetRx(void)
{
    HAL_GPIO_WritePin(RS485_3_GPIO_Port, RS485_3_Pin, GPIO_PIN_RESET);
    ShortDelay();
}

HAL_StatusTypeDef CliRs485_Write(const uint8_t *data,
                                 uint16_t len,
                                 uint32_t timeout)
{
    HAL_StatusTypeDef st;
    uint32_t start_tick;
    uint8_t locked = 0U;

    if (data == NULL || len == 0U)
    {
        return HAL_ERROR;
    }

    if (CliNeedMutex() != 0U)
    {
        if (osMutexAcquire(CliprintMutexHandle, timeout) == osOK)
        {
            locked = 1U;
        }
    }

    CliRs485_SetTx();

    /*
     * RS485 transceiver가 TX 모드로 바뀔 시간.
     * 앞 글자 깨짐 방지.
     */
    HAL_Delay(1U);

    st = HAL_UART_Transmit(&CLI_UART_HANDLE,
                           (uint8_t *)data,
                           len,
                           timeout);

    start_tick = HAL_GetTick();

    while (__HAL_UART_GET_FLAG(&CLI_UART_HANDLE, UART_FLAG_TC) == RESET)
    {
        if ((HAL_GetTick() - start_tick) > timeout)
        {
            st = HAL_TIMEOUT;
            break;
        }
    }

    /*
     * 마지막 stop bit까지 완전히 나간 뒤 RX로 복귀.
     */
    HAL_Delay(1U);

    CliRs485_SetRx();

    if (locked != 0U)
    {
        (void)osMutexRelease(CliprintMutexHandle);
    }

    return st;
}

/*
 * retarget.c를 없애기 위해 _write()를 이 파일로 이동.
 * syscalls.c의 _write는 weak라서 이 함수가 우선 적용된다.
 */
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

/* ============================================================
 * Motor RS485 / Modbus RTU section
 * ============================================================ */

#define MOTOR_TX_TIMEOUT_MS          100U
#define MOTOR_RX_TIMEOUT_MS          500U
#define MOTOR_TC_TIMEOUT_MS          100U
#define MOTOR_FRAME_GAP_MS           10U

extern osMutexId_t MotorBusMutexHandle;

static uint8_t MotorNeedMutex(void)
{
    if (MotorBusMutexHandle == NULL)
    {
        return 0U;
    }

    if (osKernelGetState() != osKernelRunning)
    {
        return 0U;
    }

    return 1U;
}

static void MotorRs485_SetTx(void)
{
    HAL_GPIO_WritePin(RS485_2_GPIO_Port, RS485_2_Pin, GPIO_PIN_SET);
}

void MotorRs485_SetRx(void)
{
    HAL_GPIO_WritePin(RS485_2_GPIO_Port, RS485_2_Pin, GPIO_PIN_RESET);
}

static void MotorClearUartError(UART_HandleTypeDef *huart)
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

static void MotorClearRxBuffer(UART_HandleTypeDef *huart)
{
    volatile uint8_t dummy;

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

static uint16_t ModbusCrc(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFU;

    for (uint16_t i = 0U; i < len; i++)
    {
        crc ^= (uint16_t)buf[i];

        for (uint8_t bit = 0U; bit < 8U; bit++)
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

static HAL_StatusTypeDef ModbusCheckCrc(const uint8_t *rx, uint16_t len)
{
    uint16_t calc;
    uint16_t recv;

    if (rx == NULL || len < 2U)
    {
        motor_debug.crc_ok = 0U;
        return HAL_ERROR;
    }

    calc = ModbusCrc(rx, (uint16_t)(len - 2U));
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

static HAL_StatusTypeDef MotorWaitTxDone(UART_HandleTypeDef *huart)
{
    uint32_t start_tick;

    if (huart == NULL)
    {
        return HAL_ERROR;
    }

    start_tick = HAL_GetTick();

    while (__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) == RESET)
    {
        if ((HAL_GetTick() - start_tick) > MOTOR_TC_TIMEOUT_MS)
        {
            motor_debug.uart_error = huart->ErrorCode;
            return HAL_TIMEOUT;
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef MotorSendAndReceive(UART_HandleTypeDef *huart,
                                             uint8_t *tx,
                                             uint16_t tx_len,
                                             uint8_t *rx,
                                             uint16_t rx_len)
{
    HAL_StatusTypeDef st;
    uint8_t locked = 0U;

    if (huart == NULL || tx == NULL || rx == NULL)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    if (MotorNeedMutex() != 0U)
    {
        if (osMutexAcquire(MotorBusMutexHandle, osWaitForever) == osOK)
        {
            locked = 1U;
        }
    }

    MotorClearRxBuffer(huart);
    MotorClearUartError(huart);

    memset(rx, 0, rx_len);

    MotorRs485_SetTx();
    HAL_Delay(1U);

    st = HAL_UART_Transmit(huart,
                           tx,
                           tx_len,
                           MOTOR_TX_TIMEOUT_MS);

    if (st == HAL_OK)
    {
        st = MotorWaitTxDone(huart);
    }

    MotorRs485_SetRx();

    if (st == HAL_OK)
    {
        st = HAL_UART_Receive(huart,
                              rx,
                              rx_len,
                              MOTOR_RX_TIMEOUT_MS);
    }

    HAL_Delay(MOTOR_FRAME_GAP_MS);

    motor_state.last_hal = st;
    motor_debug.uart_error = huart->ErrorCode;

    if (locked != 0U)
    {
        (void)osMutexRelease(MotorBusMutexHandle);
    }

    return st;
}

HAL_StatusTypeDef MotorBus_WriteU16(UART_HandleTypeDef *huart,
                                    uint16_t reg,
                                    uint16_t value)
{
    uint8_t tx[8];
    uint8_t rx[8];
    uint16_t crc;
    HAL_StatusTypeDef st;

    motor_debug.last_reg = reg;
    motor_debug.exception_code = 0U;
    motor_debug.crc_ok = 0U;

    tx[0] = (uint8_t)MOTOR_ID;
    tx[1] = 0x06U;
    tx[2] = (uint8_t)((reg >> 8U) & 0xFFU);
    tx[3] = (uint8_t)(reg & 0xFFU);
    tx[4] = (uint8_t)((value >> 8U) & 0xFFU);
    tx[5] = (uint8_t)(value & 0xFFU);

    crc = ModbusCrc(tx, 6U);
    tx[6] = (uint8_t)(crc & 0xFFU);
    tx[7] = (uint8_t)((crc >> 8U) & 0xFFU);

    st = MotorSendAndReceive(huart, tx, 8U, rx, 8U);

    if (st != HAL_OK)
    {
        return st;
    }

    if (rx[0] != (uint8_t)MOTOR_ID)
    {
        return HAL_ERROR;
    }

    if (rx[1] == 0x86U)
    {
        motor_debug.exception_code = rx[2];
        return HAL_ERROR;
    }

    if (rx[1] != 0x06U)
    {
        return HAL_ERROR;
    }

    if (ModbusCheckCrc(rx, 8U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    for (uint8_t i = 0U; i < 6U; i++)
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
    uint32_t raw;
    uint16_t low_word;
    uint16_t high_word;
    HAL_StatusTypeDef st;

    if (huart == NULL)
    {
        motor_state.last_hal = HAL_ERROR;
        return HAL_ERROR;
    }

    /*
     * 중요:
     * H11_12 같은 32bit 파라미터는 06H 두 번으로 쓰면
     * 모터가 응답하지 않는 경우가 있다.
     *
     * 따라서 10H Multiple Register Write로 2개 register를 한 번에 쓴다.
     *
     * 현재 pos read 결과 기준:
     *   reg     = low word
     *   reg + 1 = high word
     */
    raw = (uint32_t)value;
    low_word  = (uint16_t)(raw & 0xFFFFU);
    high_word = (uint16_t)((raw >> 16U) & 0xFFFFU);

    motor_debug.last_reg = reg;
    motor_debug.exception_code = 0U;
    motor_debug.crc_ok = 0U;

    tx[0]  = (uint8_t)MOTOR_ID;
    tx[1]  = 0x10U;
    tx[2]  = (uint8_t)((reg >> 8U) & 0xFFU);
    tx[3]  = (uint8_t)(reg & 0xFFU);
    tx[4]  = 0x00U;
    tx[5]  = 0x02U;
    tx[6]  = 0x04U;

    /*
     * low word first
     */
    tx[7]  = (uint8_t)((low_word >> 8U) & 0xFFU);
    tx[8]  = (uint8_t)(low_word & 0xFFU);

    /*
     * high word second
     */
    tx[9]  = (uint8_t)((high_word >> 8U) & 0xFFU);
    tx[10] = (uint8_t)(high_word & 0xFFU);

    crc = ModbusCrc(tx, 11U);
    tx[11] = (uint8_t)(crc & 0xFFU);
    tx[12] = (uint8_t)((crc >> 8U) & 0xFFU);

    st = MotorSendAndReceive(huart, tx, 13U, rx, 8U);

    if (st != HAL_OK)
    {
        motor_state.last_hal = st;

        printf("[BUS_DBG] write32 timeout/fail reg=0x%04X value=%ld st=%d uart=0x%08lX\r\n",
               reg,
               (long)value,
               st,
               (unsigned long)motor_debug.uart_error);

        return st;
    }

    /*
     * Modbus exception response
     */
    if (rx[0] == (uint8_t)MOTOR_ID && rx[1] == 0x90U)
    {
        motor_debug.exception_code = rx[2];

        printf("[BUS_DBG] write32 exception reg=0x%04X ex=%u raw=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
               reg,
               motor_debug.exception_code,
               rx[0], rx[1], rx[2], rx[3],
               rx[4], rx[5], rx[6], rx[7]);

        return HAL_ERROR;
    }

    if (rx[0] != (uint8_t)MOTOR_ID || rx[1] != 0x10U)
    {
        printf("[BUS_DBG] write32 bad response reg=0x%04X raw=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
               reg,
               rx[0], rx[1], rx[2], rx[3],
               rx[4], rx[5], rx[6], rx[7]);

        return HAL_ERROR;
    }

    if (ModbusCheckCrc(rx, 8U) != HAL_OK)
    {
        printf("[BUS_DBG] write32 crc fail reg=0x%04X calc=0x%04X recv=0x%04X raw=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
               reg,
               motor_debug.crc_calc,
               motor_debug.crc_recv,
               rx[0], rx[1], rx[2], rx[3],
               rx[4], rx[5], rx[6], rx[7]);

        return HAL_ERROR;
    }

    /*
     * 표준 10H 응답:
     * ID, 10H, start addr hi, start addr lo, count hi, count lo, CRC lo, CRC hi
     */
    if (rx[2] != tx[2] ||
        rx[3] != tx[3] ||
        rx[4] != 0x00U ||
        rx[5] != 0x02U)
    {
        printf("[BUS_DBG] write32 addr/count mismatch reg=0x%04X raw=%02X %02X %02X %02X %02X %02X %02X %02X\r\n",
               reg,
               rx[0], rx[1], rx[2], rx[3],
               rx[4], rx[5], rx[6], rx[7]);

        return HAL_ERROR;
    }

    motor_state.last_hal = HAL_OK;
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
    HAL_StatusTypeDef st;

    if (huart == NULL || value == NULL)
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

    crc = ModbusCrc(tx, 6U);
    tx[6] = (uint8_t)(crc & 0xFFU);
    tx[7] = (uint8_t)((crc >> 8U) & 0xFFU);

    st = MotorSendAndReceive(huart, tx, 8U, rx, 9U);

    if (st != HAL_OK)
    {
        return st;
    }

    if (rx[0] != (uint8_t)MOTOR_ID)
    {
        return HAL_ERROR;
    }

    if (rx[1] == 0x83U)
    {
        motor_debug.exception_code = rx[2];
        return HAL_ERROR;
    }

    if (rx[1] != 0x03U || rx[2] != 0x04U)
    {
        return HAL_ERROR;
    }

    if (ModbusCheckCrc(rx, 9U) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /*
     * AIMotor 32-bit monitor order:
     * first word = low word
     * second word = high word
     */
    low_word = ((uint16_t)rx[3] << 8U) | rx[4];
    high_word = ((uint16_t)rx[5] << 8U) | rx[6];

    raw = ((uint32_t)high_word << 16U) | low_word;

    *value = (int32_t)raw;
    motor_debug.last_pos = *value;

    return HAL_OK;
}

/* ============================================================
 * Common init section
 * ============================================================ */

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
