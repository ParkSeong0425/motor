#include "motor_bus.h"
#include "motor.h"

#include "gpio.h"
#include "cmsis_os.h"

#include <string.h>

/*
 * Motor RS485
 *
 * USART6 : AIMotor Modbus RTU
 * PF12   : RS485_3 direction pin
 *
 * SET   = TX
 * RESET = RX
 */

#define MOTOR_TX_TIMEOUT_MS       100U
#define MOTOR_RX_TIMEOUT_MS       500U
#define MOTOR_TC_TIMEOUT_MS       100U
#define MOTOR_FRAME_GAP_MS        10U

__weak osMutexId_t MotorBusMutexHandle = NULL;

static uint8_t NeedMutex(void)
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

static void MotorBus_SetTx(void)
{
  HAL_GPIO_WritePin(RS485_2_GPIO_Port, RS485_2_Pin, GPIO_PIN_SET);
}

void MotorBus_SetRx(void)
{
  HAL_GPIO_WritePin(RS485_2_GPIO_Port, RS485_2_Pin, GPIO_PIN_RESET);
}

static void ClearUartError(UART_HandleTypeDef *huart)
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

static void ClearRxBuffer(UART_HandleTypeDef *huart)
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

static uint16_t MakeCrc(const uint8_t *buf, uint16_t len)
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

static HAL_StatusTypeDef CheckCrc(const uint8_t *rx, uint16_t len)
{
  uint16_t calc;
  uint16_t recv;

  if (rx == NULL || len < 2U)
  {
    motor_debug.crc_ok = 0U;
    return HAL_ERROR;
  }

  calc = MakeCrc(rx, (uint16_t)(len - 2U));
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

static HAL_StatusTypeDef WaitTxDone(UART_HandleTypeDef *huart)
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

static HAL_StatusTypeDef SendAndReceive(UART_HandleTypeDef *huart,
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

  if (NeedMutex() != 0U)
  {
    if (osMutexAcquire(MotorBusMutexHandle, osWaitForever) == osOK)
    {
      locked = 1U;
    }
  }

  ClearRxBuffer(huart);
  ClearUartError(huart);

  memset(rx, 0, rx_len);

  MotorBus_SetTx();
  HAL_Delay(1U);

  st = HAL_UART_Transmit(huart, tx, tx_len, MOTOR_TX_TIMEOUT_MS);

  if (st == HAL_OK)
  {
    st = WaitTxDone(huart);
  }

  MotorBus_SetRx();

  if (st == HAL_OK)
  {
    st = HAL_UART_Receive(huart, rx, rx_len, MOTOR_RX_TIMEOUT_MS);
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

  crc = MakeCrc(tx, 6U);
  tx[6] = (uint8_t)(crc & 0xFFU);
  tx[7] = (uint8_t)((crc >> 8U) & 0xFFU);

  st = SendAndReceive(huart, tx, 8U, rx, 8U);

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

  if (CheckCrc(rx, 8U) != HAL_OK)
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
   * AIMotor 32-bit parameter order:
   *   reg     = low word
   *   reg + 1 = high word
   *
   * 예전처럼 06H 두 번으로 나눠 쓰지 말고,
   * 10H로 2개 register를 한 번에 쓴다.
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

  crc = MakeCrc(tx, 11U);
  tx[11] = (uint8_t)(crc & 0xFFU);
  tx[12] = (uint8_t)((crc >> 8U) & 0xFFU);

  st = SendAndReceive(huart, tx, 13U, rx, 8U);

  if (st != HAL_OK)
  {
    return st;
  }

  if (rx[0] != (uint8_t)MOTOR_ID)
  {
    return HAL_ERROR;
  }

  if (rx[1] == 0x90U)
  {
    motor_debug.exception_code = rx[2];
    return HAL_ERROR;
  }

  if (rx[1] != 0x10U)
  {
    return HAL_ERROR;
  }

  if (CheckCrc(rx, 8U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (rx[2] != tx[2] ||
      rx[3] != tx[3] ||
      rx[4] != 0x00U ||
      rx[5] != 0x02U)
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

  crc = MakeCrc(tx, 6U);
  tx[6] = (uint8_t)(crc & 0xFFU);
  tx[7] = (uint8_t)((crc >> 8U) & 0xFFU);

  st = SendAndReceive(huart, tx, 8U, rx, 9U);

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

  if (CheckCrc(rx, 9U) != HAL_OK)
  {
    return HAL_ERROR;
  }

  /*
   * AIMotor 32-bit monitor value order:
   *   first word  = low word
   *   second word = high word
   *
   * 기존 코드처럼 rx[3]부터 바로 32-bit big endian으로 만들면
   * pos=64487424 같은 65536 배수 값이 나올 수 있다.
   */
  low_word  = ((uint16_t)rx[3] << 8U) | rx[4];
  high_word = ((uint16_t)rx[5] << 8U) | rx[6];

  raw = ((uint32_t)high_word << 16U) | low_word;

  *value = (int32_t)raw;
  motor_debug.last_pos = *value;

  return HAL_OK;
}
