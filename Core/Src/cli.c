#include "cli.h"
#include "main.h"
#include "usart.h"
#include "cmsis_os.h"
#include "motor.h"
#include "motion_protocol.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define CLI_UART    huart6
#define LINE_SIZE   160
#define WORD_MAX    32
#define CMD_SIZE    192
#define REPLY_SIZE  512

static char line_buf[LINE_SIZE];
static uint16_t line_len = 0;
static volatile uint8_t cli_ready = 0;

/* 공백 기준 단어 분리 */
static int cut(char *line, char *word[])
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

/* uint32 읽기 */
static uint8_t u32(const char *text, uint32_t *out)
{
    char *end;
    unsigned long value = strtoul(text, &end, 0);

    if (*end != '\0') return 0;
    *out = (uint32_t)value;
    return 1;
}

/* protocol 명령 실행 */
static void proto(const char *cmd)
{
    char reply[REPLY_SIZE];

    MotionProtocol_Command(cmd, reply, sizeof(reply));
    printf("%s", reply);
}

/* 도움말 */
static void help(void)
{
    printf("\r\n");
    printf("set\r\n");
    printf("wheel <dia_mm>\r\n");
    printf("zero\r\n");
    printf("home\r\n");
    printf("in <mm>\r\n");
    printf("save <mm1> [mm2...]\r\n");
    printf("show\r\n");
    printf("go <save_num> <speed>\r\n");
    printf("run <save_num> <speed>\r\n");
    printf("do28\r\n");
    printf("power on|off\r\n");
    printf("stop\r\n");
    printf("pos\r\n");
    printf("estop clear\r\n");
    printf("clear\r\n");
    printf("\r\n");
}

/* 현재 위치 표시 */
static void pos(void)
{
    int32_t now = 0;

    if (Motor_ReadPos(&huart5, &now) == HAL_OK) printf("pos=%ld\r\n", (long)now);
    else printf("pos read fail\r\n");
}

/* wheel 저장 */
static void wheel(int count, char *word[])
{
    char cmd[CMD_SIZE];
    uint32_t dia;

    if (count < 2 || u32(word[1], &dia) == 0)
    {
        printf("wheel <dia_mm>\r\n");
        return;
    }

    snprintf(cmd, sizeof(cmd), "01W_%lu", (unsigned long)dia);
    proto(cmd);
}

/* 입고 위치 저장 */
static void inpos(int count, char *word[])
{
    char cmd[CMD_SIZE];

    if (count < 2)
    {
        printf("in <mm>\r\n");
        return;
    }

    snprintf(cmd, sizeof(cmd), "01FP_1_1_%s", word[1]);
    proto(cmd);
}

/* 출고 위치 저장 */
static void save(int count, char *word[])
{
    char cmd[CMD_SIZE];
    int used;
    int add;

    if (count < 2)
    {
        printf("save <mm1> [mm2...]\r\n");
        return;
    }

    used = snprintf(cmd, sizeof(cmd), "01FP_1_2");

    for (int i = 1; i < count; i++)
    {
        add = snprintf(&cmd[used], sizeof(cmd) - (size_t)used, "_%s", word[i]);

        if (add < 0 || (used + add) >= (int)sizeof(cmd))
        {
            printf("too long\r\n");
            return;
        }

        used += add;
    }

    proto(cmd);
}

/* 저장 위치로 1회 이동 */
static void go(int count, char *word[])
{
    char cmd[CMD_SIZE];
    uint32_t num;
    uint32_t speed;

    if (count < 3 || u32(word[1], &num) == 0 || u32(word[2], &speed) == 0)
    {
        printf("go <save_num> <speed>\r\n");
        return;
    }

    snprintf(cmd, sizeof(cmd),
             "01MO_1_%lu_%lu_0_0",
             (unsigned long)num,
             (unsigned long)speed);
    proto(cmd);
}

/* run 왕복 */
static void run_loop(int count, char *word[])
{
    uint32_t num;
    uint32_t speed;

    if (count < 3 || u32(word[1], &num) == 0 || u32(word[2], &speed) == 0)
    {
        printf("run <save_num> <speed>\r\n");
        return;
    }

    printf((MotionProtocol_Run(num, speed) != 0) ? "run start\r\n" : "run fail\r\n");
}

/* power 명령 */
static void power(int count, char *word[])
{
    if (count < 2)
    {
        printf("power on|off\r\n");
        return;
    }

    if (strcmp(word[1], "on") == 0) proto("01P_1");
    else if (strcmp(word[1], "off") == 0) proto("01P_0");
    else printf("power on|off\r\n");
}

/* 한 줄 실행 */
static void run(char *line)
{
    char *word[WORD_MAX];
    int count = cut(line, word);

    if (count <= 0) return;

    if (strcmp(word[0], "help") == 0) help();
    else if (strcmp(word[0], "set") == 0) printf("set=%d\r\n", Motor_Setup(&huart5));
    else if (strcmp(word[0], "wheel") == 0) wheel(count, word);
    else if (strcmp(word[0], "zero") == 0) proto("01ZERO");
    else if (strcmp(word[0], "home") == 0) proto("01HOME");
    else if (strcmp(word[0], "in") == 0) inpos(count, word);
    else if (strcmp(word[0], "save") == 0) save(count, word);
    else if (strcmp(word[0], "show") == 0) proto("01Q");
    else if (strcmp(word[0], "go") == 0) go(count, word);
    else if (strcmp(word[0], "run") == 0) run_loop(count, word);
    else if (strcmp(word[0], "do28") == 0) proto("01D28");
    else if (strcmp(word[0], "power") == 0) power(count, word);
    else if (strcmp(word[0], "stop") == 0) proto("01STOP");
    else if (strcmp(word[0], "pos") == 0) pos();
    else if (strcmp(word[0], "clear") == 0) proto("01I");
    else if (strcmp(word[0], "estop") == 0 &&
             count >= 2 && strcmp(word[1], "clear") == 0) proto("01ECLR");
    else printf("?\r\n");
}

/* NET task가 CLI 초기 출력 완료를 알 수 있게 설정 */
void CLI_SetReady(void)
{
    cli_ready = 1;
}

/* CLI 초기 출력 완료 상태 */
uint8_t CLI_IsReady(void)
{
    return cli_ready;
}

/* CLI 초기 출력: 자동 help는 안 찍고 prompt만 */
void CLI_Init(void)
{
    if (cli_ready != 0) return;
    cli_ready = 1;
    printf("\r\n> ");
}

/* UART 문자 1개씩 받아 한 줄 명령 처리 */
void CLI_Poll(void)
{
    uint8_t ch;
    HAL_StatusTypeDef result;
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
            run(line_buf);
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
            printf("\b \b");
        }
        return;
    }

    if (ch >= 32 && ch <= 126 && line_len < (LINE_SIZE - 1))
    {
        line_buf[line_len++] = (char)ch;
    }
}

/* CLI task */
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
