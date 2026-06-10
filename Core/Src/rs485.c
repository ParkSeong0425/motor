/*
 * rs485.c
 *
 *  Created on: Jun 8, 2026
 *      Author: HWNOT
 */
#include "rs485.h"

#include "usart.h"
#include "gpio.h"


#include "rs485.h"

#include "usart.h"
#include "gpio.h"
#include "cmsis_os.h"

#define CLI_UART_HANDLE       huart6

/*
 * CLI RS485
 *
 * UART5  : CLI terminal
 * PD13   : RS485_2 direction pin
 *
 * SET   = TX
 * RESET = RX
 *
 * If your RS485 transceiver direction is reversed,
 * swap GPIO_PIN_SET and GPIO_PIN_RESET in RS485_SetTx/Rx.
 */

__weak osMutexId_t CliPrintMutexHandle = NULL;

static void ShortDelay(void)
{
  for (volatile uint32_t i = 0; i < 300U; i++)
  {
    __NOP();
  }
}

static uint8_t NeedMutex(void)
{
  if (CliPrintMutexHandle == NULL)
  {
    return 0U;
  }

  if (osKernelGetState() != osKernelRunning)
  {
    return 0U;
  }

  return 1U;
}

void RS485_Init(void)
{
  RS485_SetRx();

  __HAL_UART_CLEAR_OREFLAG(&CLI_UART_HANDLE);
  __HAL_UART_CLEAR_NEFLAG(&CLI_UART_HANDLE);
  __HAL_UART_CLEAR_FEFLAG(&CLI_UART_HANDLE);
  __HAL_UART_CLEAR_PEFLAG(&CLI_UART_HANDLE);
}

void RS485_SetTx(void)
{
  HAL_GPIO_WritePin(RS485_3_GPIO_Port, RS485_3_Pin, GPIO_PIN_SET);
  ShortDelay();
}

void RS485_SetRx(void)
{
  HAL_GPIO_WritePin(RS485_3_GPIO_Port, RS485_3_Pin, GPIO_PIN_RESET);
  ShortDelay();
}

HAL_StatusTypeDef RS485_Transmit(const uint8_t *data,
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

  if (NeedMutex() != 0U)
  {
    if (osMutexAcquire(CliPrintMutexHandle, timeout) == osOK)
    {
      locked = 1U;
    }
  }

  RS485_SetTx();

  /*
   * RS485 transceiver가 TX 모드로 완전히 바뀔 시간을 준다.
   * 앞 글자 깨짐 방지.
   */
  HAL_Delay(1U);

  st = HAL_UART_Transmit(&CLI_UART_HANDLE, (uint8_t *)data, len, timeout);

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
   * 마지막 stop bit까지 라인에 완전히 나가도록 약간 대기.
   */
  HAL_Delay(1U);

  RS485_SetRx();

  if (locked != 0U)
  {
    (void)osMutexRelease(CliPrintMutexHandle);
  }

  return st;
}
