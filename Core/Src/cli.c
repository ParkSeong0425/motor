#include "cli.h"

#include "main.h"
#include "usart.h"
#include "i2c.h"
#include "spi.h"

#include "mac_eeprom.h"
#include "fram.h"
#include "motor.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define CLI_UART          huart6

#define LINE_SIZE         128U
#define MAX_WORDS         24U

static char line_buf[LINE_SIZE];
static uint16_t line_len = 0U;

static void ShowHelp(void);
static void ShowPrompt(void);
static void RunCommand(char *line);
static int SplitWords(char *line, char *word[], int max_words);

static uint8_t ReadU32(const char *text, uint32_t *out);
static uint8_t ReadI32(const char *text, int32_t *out);
static uint8_t ReadHexByte(const char *text, uint8_t *out);

static void ShowMac(void);
static void ScanI2c(void);

static void RunFram(int count, char *word[]);
static void ShowFram(void);
static void DumpFram(uint16_t addr, uint16_t len);
static HAL_StatusTypeDef ClearFram(uint16_t addr, uint16_t len);

static void RunMotor(int count, char *word[]);

void CLI_Init(void)
{
  ShowHelp();
  ShowPrompt();
}

void CLI_Poll(void)
{
  uint8_t ch;
  HAL_StatusTypeDef st;

  static uint8_t esc_skip = 0U;
  static uint8_t prev_cr = 0U;

  st = HAL_UART_Receive(&CLI_UART, &ch, 1U, 20U);

  if (st != HAL_OK)
  {
    if (HAL_UART_GetError(&CLI_UART) != HAL_UART_ERROR_NONE)
    {
      __HAL_UART_CLEAR_OREFLAG(&CLI_UART);
      __HAL_UART_CLEAR_NEFLAG(&CLI_UART);
      __HAL_UART_CLEAR_FEFLAG(&CLI_UART);
      __HAL_UART_CLEAR_PEFLAG(&CLI_UART);
      CLI_UART.ErrorCode = HAL_UART_ERROR_NONE;
    }

    return;
  }

  /*
   * 방향키는 보통 ESC [ A 같은 3바이트로 들어온다.
   * ESC 이후 2바이트를 버려서 [A, [B 같은 문자가 명령에 섞이지 않게 한다.
   */
  if (esc_skip > 0U)
  {
    esc_skip--;
    return;
  }

  if (ch == 0x1BU)
  {
    esc_skip = 2U;
    return;
  }

  /*
   * Tera Term이 CR+LF를 보내면 '\r' 처리 후 바로 '\n'이 한 번 더 들어온다.
   * 이때 프롬프트가 두 번 찍히는 것을 막는다.
   */
  if (ch == '\n' && prev_cr != 0U)
  {
    prev_cr = 0U;
    return;
  }

  if (ch == '\r' || ch == '\n')
  {
    prev_cr = (ch == '\r') ? 1U : 0U;

    printf("\r\n");

    if (line_len > 0U)
    {
      line_buf[line_len] = '\0';
      RunCommand(line_buf);
      line_len = 0U;
    }

    ShowPrompt();
    return;
  }

  prev_cr = 0U;

  if (ch == 0x08U || ch == 0x7FU)
  {
    if (line_len > 0U)
    {
      line_len--;
    }

    return;
  }

  if (ch >= 32U && ch <= 126U)
  {
    if (line_len < (LINE_SIZE - 1U))
    {
      line_buf[line_len++] = (char)ch;
    }
  }
}

static void ShowHelp(void)
{
  printf("\r\n");
  printf("CLI commands\r\n");
  printf("  help\r\n");
  printf("  info\r\n");
  printf("  mac\r\n");
  printf("  i2c scan\r\n");
  printf("\r\n");
  printf("FRAM commands\r\n");
  printf("  fram status\r\n");
  printf("  fram dump <addr> <len>\r\n");
  printf("  fram write <addr> <hex bytes...>\r\n");
  printf("  fram clear <addr> <len>\r\n");
  printf("\r\n");
  printf("Motor commands\r\n");
  printf("  motor status\r\n");
  printf("  motor setup\r\n");
  printf("  motor sethome\r\n");
  printf("  motor clearhome\r\n");
  printf("  motor home\r\n");
  printf("  motor move <mm> <speed_percent> [acc_ms] [start_ms] [wait_ms]\r\n");
  printf("  motor stop\r\n");
  printf("  motor estop\r\n");
  printf("  motor release\r\n");
  printf("  motor pos\r\n");
  printf("\r\n");
}

static void ShowPrompt(void)
{
  printf("> ");
}

static void RunCommand(char *line)
{
  char *word[MAX_WORDS];
  int count;

  count = SplitWords(line, word, MAX_WORDS);

  if (count <= 0)
  {
    return;
  }

  if (strcmp(word[0], "help") == 0)
  {
    ShowHelp();
  }
  else if (strcmp(word[0], "info") == 0)
  {
    ShowMac();
    ShowFram();
  }
  else if (strcmp(word[0], "mac") == 0)
  {
    ShowMac();
  }
  else if (strcmp(word[0], "i2c") == 0)
  {
    if (count >= 2 && strcmp(word[1], "scan") == 0)
    {
      ScanI2c();
    }
    else
    {
      printf("Usage: i2c scan\r\n");
    }
  }
  else if (strcmp(word[0], "fram") == 0)
  {
    RunFram(count, word);
  }
  else if (strcmp(word[0], "motor") == 0)
  {
    RunMotor(count, word);
  }
  else if (strcmp(word[0], "reboot") == 0)
  {
    printf("Rebooting...\r\n");
    HAL_Delay(100U);
    NVIC_SystemReset();
  }
  else
  {
    printf("Unknown command: %s\r\n", word[0]);
  }
}

static int SplitWords(char *line, char *word[], int max_words)
{
  int count = 0;
  char *token;

  token = strtok(line, " \t");

  while (token != NULL && count < max_words)
  {
    word[count++] = token;
    token = strtok(NULL, " \t");
  }

  return count;
}

static uint8_t ReadU32(const char *text, uint32_t *out)
{
  char *end;
  unsigned long value;

  if (text == NULL || out == NULL)
  {
    return 0U;
  }

  value = strtoul(text, &end, 0);

  if (*end != '\0')
  {
    return 0U;
  }

  *out = (uint32_t)value;
  return 1U;
}

static uint8_t ReadI32(const char *text, int32_t *out)
{
  char *end;
  long value;

  if (text == NULL || out == NULL)
  {
    return 0U;
  }

  value = strtol(text, &end, 0);

  if (*end != '\0')
  {
    return 0U;
  }

  *out = (int32_t)value;
  return 1U;
}

static uint8_t ReadHexByte(const char *text, uint8_t *out)
{
  char *end;
  unsigned long value;

  if (text == NULL || out == NULL)
  {
    return 0U;
  }

  value = strtoul(text, &end, 16);

  if (*end != '\0' || value > 0xFFUL)
  {
    return 0U;
  }

  *out = (uint8_t)value;
  return 1U;
}

static void ShowMac(void)
{
  uint8_t mac[MAC_EEPROM_EUI48_LEN] = {0};
  HAL_StatusTypeDef st;

  st = MacEeprom_ReadMac(&hi2c1, mac);

  if (st == HAL_OK)
  {
    printf("[MAC] %02X:%02X:%02X:%02X:%02X:%02X\r\n",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    if (MacEeprom_IsMacPlausible(mac) != 0U)
    {
      printf("[MAC] OK\r\n");
    }
    else
    {
      printf("[MAC] invalid value\r\n");
    }
  }
  else
  {
    printf("[MAC] read failed, status=%d\r\n", st);
  }
}

static void ScanI2c(void)
{
  uint8_t found = 0U;

  printf("[I2C] scan start\r\n");

  for (uint8_t addr = 1U; addr < 128U; addr++)
  {
    if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 2U, 20U) == HAL_OK)
    {
      found++;
      printf("[I2C] found 0x%02X\r\n", addr);
    }
  }

  printf("[I2C] scan end, found=%u\r\n", found);
}

static void ShowFram(void)
{
  uint8_t before = 0U;
  uint8_t after_wren = 0U;
  uint8_t after_wrdi = 0U;
  HAL_StatusTypeDef st;

  st = Fram_CheckWriteEnableLatch(&hspi3,
                                  &before,
                                  &after_wren,
                                  &after_wrdi);

  printf("[FRAM] status=%d before=0x%02X WREN=0x%02X WRDI=0x%02X\r\n",
         st, before, after_wren, after_wrdi);
}

static void DumpFram(uint16_t addr, uint16_t len)
{
  uint8_t buf[16];

  while (len > 0U)
  {
    uint16_t chunk;

    chunk = (len > sizeof(buf)) ? sizeof(buf) : len;
    memset(buf, 0, sizeof(buf));

    if (Fram_Read(&hspi3, addr, buf, chunk) != HAL_OK)
    {
      printf("[FRAM] read failed at 0x%04X\r\n", addr);
      return;
    }

    printf("0x%04X: ", addr);

    for (uint16_t i = 0U; i < chunk; i++)
    {
      printf("%02X ", buf[i]);
    }

    printf("\r\n");

    addr += chunk;
    len -= chunk;
  }
}

static HAL_StatusTypeDef ClearFram(uint16_t addr, uint16_t len)
{
  uint8_t zero[16] = {0};

  while (len > 0U)
  {
    uint16_t chunk;

    chunk = (len > sizeof(zero)) ? sizeof(zero) : len;

    if (Fram_Write(&hspi3, addr, zero, chunk) != HAL_OK)
    {
      return HAL_ERROR;
    }

    addr += chunk;
    len -= chunk;
  }

  return HAL_OK;
}

static void RunFram(int count, char *word[])
{
  if (count < 2)
  {
    printf("Usage: fram status | dump | write | clear\r\n");
    return;
  }

  if (strcmp(word[1], "status") == 0)
  {
    ShowFram();
  }
  else if (strcmp(word[1], "dump") == 0)
  {
    uint32_t addr;
    uint32_t len;

    if (count < 4 ||
        ReadU32(word[2], &addr) == 0U ||
        ReadU32(word[3], &len) == 0U)
    {
      printf("Usage: fram dump <addr> <len>\r\n");
      return;
    }

    if (addr >= FRAM_MB85RS64_SIZE_BYTES ||
        len == 0U ||
        (addr + len) > FRAM_MB85RS64_SIZE_BYTES)
    {
      printf("Invalid FRAM range\r\n");
      return;
    }

    DumpFram((uint16_t)addr, (uint16_t)len);
  }
  else if (strcmp(word[1], "write") == 0)
  {
    uint32_t addr;
    uint8_t data[32];
    uint16_t len = 0U;

    if (count < 4 || ReadU32(word[2], &addr) == 0U)
    {
      printf("Usage: fram write <addr> <hex bytes...>\r\n");
      return;
    }

    for (int i = 3; i < count && len < sizeof(data); i++)
    {
      if (ReadHexByte(word[i], &data[len]) == 0U)
      {
        printf("Invalid byte: %s\r\n", word[i]);
        return;
      }

      len++;
    }

    if (addr >= FRAM_MB85RS64_SIZE_BYTES ||
        len == 0U ||
        (addr + len) > FRAM_MB85RS64_SIZE_BYTES)
    {
      printf("Invalid FRAM range\r\n");
      return;
    }

    if (Fram_Write(&hspi3, (uint16_t)addr, data, len) == HAL_OK)
    {
      printf("[FRAM] write OK\r\n");
      DumpFram((uint16_t)addr, len);
    }
    else
    {
      printf("[FRAM] write failed\r\n");
    }
  }
  else if (strcmp(word[1], "clear") == 0)
  {
    uint32_t addr;
    uint32_t len;

    if (count < 4 ||
        ReadU32(word[2], &addr) == 0U ||
        ReadU32(word[3], &len) == 0U)
    {
      printf("Usage: fram clear <addr> <len>\r\n");
      return;
    }

    if (addr >= FRAM_MB85RS64_SIZE_BYTES ||
        len == 0U ||
        (addr + len) > FRAM_MB85RS64_SIZE_BYTES)
    {
      printf("Invalid FRAM range\r\n");
      return;
    }

    if (ClearFram((uint16_t)addr, (uint16_t)len) == HAL_OK)
    {
      printf("[FRAM] clear OK\r\n");
      DumpFram((uint16_t)addr, (uint16_t)len);
    }
    else
    {
      printf("[FRAM] clear failed\r\n");
    }
  }
  else
  {
    printf("Unknown fram command\r\n");
  }
}

static void RunMotor(int count, char *word[])
{
  if (count < 2)
  {
    printf("Usage: motor status | setup | sethome | clearhome | home | move | stop | estop | release | pos\r\n");
    return;
  }

  if (strcmp(word[1], "status") == 0)
  {
    printf("[MOTOR] setup=%u home=%u running=%u error=%u estop=%u servo=%u enable=%u\r\n",
           motor_state.setup_done,
           motor_state.home_done,
           motor_state.running,
           motor_state.error,
           motor_state.estop_on,
           motor_state.servo_on,
           motor_state.enable_on);

    printf("[MOTOR] pos=%ld home_offset=%ld target=%ld speed=%u rpm=%u acc=%u last_hal=%d\r\n",
           (long)motor_state.cur_pos,
           (long)motor_state.home_offset,
           (long)motor_state.last_target,
           motor_state.last_speed,
           motor_state.last_rpm,
           motor_state.last_acc_ms,
           motor_state.last_hal);
  }
  else if (strcmp(word[1], "setup") == 0)
  {
    HAL_StatusTypeDef st;

    if (Motor_IsBusy() != 0U)
    {
      printf("[MOTOR] busy\r\n");
      return;
    }

    st = Motor_Setup(&huart5);
    printf("[MOTOR] setup status=%d\r\n", st);
  }
  else if (strcmp(word[1], "sethome") == 0)
  {
    HAL_StatusTypeDef st;

    if (Motor_IsBusy() != 0U)
    {
      printf("[MOTOR] busy\r\n");
      return;
    }

    st = Motor_SetHome();
    printf("[MOTOR] sethome status=%d offset=%ld\r\n",
           st,
           (long)motor_state.home_offset);
  }
  else if (strcmp(word[1], "clearhome") == 0)
  {
    HAL_StatusTypeDef st;

    if (Motor_IsBusy() != 0U)
    {
      printf("[MOTOR] busy\r\n");
      return;
    }

    st = Motor_ClearHome();
    printf("[MOTOR] clearhome status=%d\r\n", st);
  }
  else if (strcmp(word[1], "home") == 0)
  {
    if (Motor_SendHome() == osOK)
    {
      printf("[MOTOR] home queued\r\n");
    }
    else
    {
      printf("[MOTOR] home failed\r\n");
    }
  }
  else if (strcmp(word[1], "move") == 0)
  {
    int32_t mm;
    int32_t unit;
    uint32_t speed;
    uint32_t acc_ms = 1000U;
    uint32_t start_ms = 0U;
    uint32_t wait_ms = 0U;

    if (count < 4 ||
        ReadI32(word[2], &mm) == 0U ||
        ReadU32(word[3], &speed) == 0U)
    {
      printf("Usage: motor move <mm> <speed_percent> [acc_ms] [start_ms] [wait_ms]\r\n");
      return;
    }

    if (count >= 5 && ReadU32(word[4], &acc_ms) == 0U)
    {
      printf("Invalid acc_ms\r\n");
      return;
    }

    if (count >= 6 && ReadU32(word[5], &start_ms) == 0U)
    {
      printf("Invalid start_ms\r\n");
      return;
    }

    if (count >= 7 && ReadU32(word[6], &wait_ms) == 0U)
    {
      printf("Invalid wait_ms\r\n");
      return;
    }

    if (speed > 100U)
    {
      printf("speed must be 0..100\r\n");
      return;
    }

    if (acc_ms > 65535U)
    {
      printf("acc_ms must be 0..65535\r\n");
      return;
    }

    unit = Motor_mmToUnit(mm);

    if (Motor_SendMove(unit, speed, acc_ms, start_ms, wait_ms) == osOK)
    {
      printf("[MOTOR] move queued mm=%ld unit=%ld speed=%lu acc=%lu\r\n",
             (long)mm,
             (long)unit,
             (unsigned long)speed,
             (unsigned long)acc_ms);
    }
    else
    {
      printf("[MOTOR] move failed\r\n");
    }
  }
  else if (strcmp(word[1], "stop") == 0)
  {
    if (Motor_SendStop() == osOK)
    {
      printf("[MOTOR] stop queued\r\n");
    }
    else
    {
      printf("[MOTOR] stop failed\r\n");
    }
  }
  else if (strcmp(word[1], "estop") == 0)
  {
    if (Motor_SendEStop() == osOK)
    {
      printf("[MOTOR] estop queued\r\n");
    }
    else
    {
      printf("[MOTOR] estop failed\r\n");
    }
  }
  else if (strcmp(word[1], "release") == 0)
  {
    if (Motor_SendRelease() == osOK)
    {
      printf("[MOTOR] release queued\r\n");
    }
    else
    {
      printf("[MOTOR] release failed\r\n");
    }
  }
  else if (strcmp(word[1], "pos") == 0)
  {
    int32_t pos = 0;
    int32_t diff = 0;
    HAL_StatusTypeDef st_pos;
    HAL_StatusTypeDef st_diff;

    if (Motor_IsBusy() != 0U)
    {
      printf("[MOTOR] busy. use motor status.\r\n");
      return;
    }

    st_pos = Motor_ReadPos(&huart5, &pos);
    st_diff = Motor_ReadDiff(&huart5, &diff);

    printf("[MOTOR] pos_status=%d pos=%ld diff_status=%d diff=%ld\r\n",
           st_pos,
           (long)pos,
           st_diff,
           (long)diff);
  }
  else
  {
    printf("Unknown motor command\r\n");
  }
}
