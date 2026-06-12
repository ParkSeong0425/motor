#include "cli.h"

#include "main.h"
#include "usart.h"
#include "cmsis_os.h"

#include "motor.h"
#include "motion_protocol.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define CLI_UART      huart6
#define LINE_SIZE     160
#define WORD_MAX      32
#define CMD_SIZE      192
#define REPLY_SIZE    128

void MotionProtocol_Poll(void);
void MotionProtocol_StopLoop(void);
uint8_t MotionProtocol_Run(uint32_t num, uint32_t speed);

static char line_buf[LINE_SIZE];
static uint16_t line_len = 0;

static int Split(char *line, char *word[])
{
    int count = 0;
    char *part = strtok(line, " \t");

    while (part != NULL && count < WORD_MAX)
    {
        word[count++] = part;
        part = strtok(NULL, " \t");
    }

    return count;
}

static uint8_t ReadU32(const char *text, uint32_t *out)
{
    char *end;
    unsigned long value;

    if (text == NULL || out == NULL)
    {
        return 0;
    }

    value = strtoul(text, &end, 0);

    if (*end != '\0')
    {
        return 0;
    }

    *out = (uint32_t)value;
    return 1;
}

static void Proto(const char *cmd)
{
    char reply[REPLY_SIZE];

    MotionProtocol_ProcessCommand(cmd, reply, sizeof(reply));
    printf("%s", reply);
}

static void Help(void)
{
    printf("\r\n");
    printf("set\r\n");
    printf("save in <mm>\r\n");
    printf("save out <mm1> [mm2...]\r\n");
    printf("go <out_no> <speed>\r\n");
    printf("run <out_no> <speed>\r\n");
    printf("stop\r\n");
    printf("estop\r\n");
    printf("release\r\n");
    printf("pos\r\n");
    printf("stat\r\n");
    printf("clear\r\n");
    printf("reboot\r\n");
    printf("\r\n");
}

static void Stat(void)
{
    printf("setup=%u moving=%u err=%u estop=%u run=%u\r\n",
           motor_state.setup_ok,
           motor_state.moving,
           motor_state.error,
           motor_state.estop,
           Motor_IsBusy());
    printf("pos=%ld target=%ld seq=%lu\r\n",
           (long)motor_state.pos,
           (long)motor_state.target,
           (unsigned long)motor_state.seq);
}

static void Pos(void)
{
    int32_t pos = 0;

    if (Motor_ReadPos(&huart5, &pos) == HAL_OK)
    {
        printf("pos=%ld\r\n", (long)pos);
    }
    else
    {
        printf("pos read fail\r\n");
    }
}

static void Save(int count, char *word[])
{
    char cmd[CMD_SIZE];
    int used;
    int add;

    if (count < 3)
    {
        printf("save in <mm> | save out <mm1> [mm2...]\r\n");
        return;
    }

    if (strcmp(word[1], "in") == 0)
    {
        snprintf(cmd, sizeof(cmd), "01FP_1_1_%s", word[2]);
        Proto(cmd);
        return;
    }

    if (strcmp(word[1], "out") == 0)
    {
        used = snprintf(cmd, sizeof(cmd), "01FP_1_2");

        for (int i = 2; i < count; i++)
        {
            add = snprintf(&cmd[used],
                           sizeof(cmd) - (size_t)used,
                           "_%s",
                           word[i]);

            if (add < 0 || (used + add) >= (int)sizeof(cmd))
            {
                printf("too long\r\n");
                return;
            }

            used += add;
        }

        Proto(cmd);
        return;
    }

    printf("save in <mm> | save out <mm1> [mm2...]\r\n");
}

static void Go(int count, char *word[])
{
    char cmd[CMD_SIZE];
    uint32_t num;
    uint32_t speed;

    if (count < 3 ||
        ReadU32(word[1], &num) == 0 ||
        ReadU32(word[2], &speed) == 0)
    {
        printf("go <out_no> <speed>\r\n");
        return;
    }

    snprintf(cmd, sizeof(cmd), "01MO_1_%lu_%lu_0_0",
             (unsigned long)num,
             (unsigned long)speed);

    Proto(cmd);
}

static void RunLoop(int count, char *word[])
{
    uint32_t num;
    uint32_t speed;

    if (count < 3 ||
        ReadU32(word[1], &num) == 0 ||
        ReadU32(word[2], &speed) == 0)
    {
        printf("run <out_no> <speed>\r\n");
        return;
    }

    if (MotionProtocol_Run(num, speed) != 0)
    {
        printf("run start\r\n");
    }
    else
    {
        printf("run fail\r\n");
    }
}

static void Run(char *line)
{
    char *word[WORD_MAX];
    int count = Split(line, word);

    if (count <= 0)
    {
        return;
    }

    if (strcmp(word[0], "help") == 0)
    {
        Help();
    }
    else if (strcmp(word[0], "set") == 0)
    {
        printf("set=%d\r\n", Motor_Setup(&huart5));
    }
    else if (strcmp(word[0], "save") == 0)
    {
        Save(count, word);
    }
    else if (strcmp(word[0], "go") == 0)
    {
        Go(count, word);
    }
    else if (strcmp(word[0], "run") == 0)
    {
        RunLoop(count, word);
    }
    else if (strcmp(word[0], "stop") == 0)
    {
        MotionProtocol_StopLoop();
        printf("stop=%s\r\n", (Motor_SendStop() == osOK) ? "ok" : "fail");
    }
    else if (strcmp(word[0], "estop") == 0)
    {
        MotionProtocol_StopLoop();
        printf("estop=%s\r\n", (Motor_SendEStop() == osOK) ? "ok" : "fail");
    }
    else if (strcmp(word[0], "release") == 0)
    {
        printf("release=%s\r\n", (Motor_SendRelease() == osOK) ? "ok" : "fail");
    }
    else if (strcmp(word[0], "pos") == 0)
    {
        Pos();
    }
    else if (strcmp(word[0], "stat") == 0)
    {
        Stat();
    }
    else if (strcmp(word[0], "clear") == 0)
    {
        Proto("01I");
    }
    else if (strcmp(word[0], "reboot") == 0)
    {
        printf("reboot\r\n");
        HAL_Delay(100);
        NVIC_SystemReset();
    }
    else
    {
        printf("?\r\n");
    }
}

void CLI_Init(void)
{
    Help();
    printf("> ");
}

void CLI_Poll(void)
{
    uint8_t ch;
    HAL_StatusTypeDef result;

    static uint8_t esc_skip = 0;
    static uint8_t prev_cr = 0;

    result = HAL_UART_Receive(&CLI_UART, &ch, 1, 20);

    if (result != HAL_OK)
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

    if (esc_skip > 0)
    {
        esc_skip--;
        return;
    }

    if (ch == 0x1B)
    {
        esc_skip = 2;
        return;
    }

    if (ch == '\n' && prev_cr != 0)
    {
        prev_cr = 0;
        return;
    }

    if (ch == '\r' || ch == '\n')
    {
        prev_cr = (ch == '\r') ? 1 : 0;
        printf("\r\n");

        if (line_len > 0)
        {
            line_buf[line_len] = '\0';
            Run(line_buf);
            line_len = 0;
        }

        printf("> ");
        return;
    }

    prev_cr = 0;

    if (ch == 0x08 || ch == 0x7F)
    {
        if (line_len > 0)
        {
            line_len--;
        }

        return;
    }

    if (ch >= 32 && ch <= 126 && line_len < (LINE_SIZE - 1))
    {
        line_buf[line_len++] = (char)ch;
    }
}

void CLI_TaskRun(void *argument)
{
    (void)argument;

    CLI_Init();

    for (;;)
    {
        CLI_Poll();
        MotionProtocol_Poll();
        osDelay(1);
    }
}
