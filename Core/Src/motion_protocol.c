#include "motion_protocol.h"

#include "motor.h"
#include "cmsis_os.h"
#include "spi.h"
#include "net.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>

#define CMD_SIZE       192
#define TOK_MAX        16
#define POS_MAX        32

#define RACK_ID        1
#define GROUP_IN       1
#define GROUP_OUT      2
#define ACC_MS         1000

#define FRAM_ADDR      0
#define FRAM_MAGIC     0x4B544D50

typedef struct
{
    uint32_t magic;
    int32_t in_pos;
    uint8_t in_ok;
    uint8_t out_ok[POS_MAX];
    int32_t out_pos[POS_MAX];
} PosSave_t;

static int32_t in_pos = 0;
static uint8_t in_ok = 1;

static int32_t out_pos[POS_MAX];
static uint8_t out_ok[POS_MAX];

static uint8_t loaded = 0;

static uint8_t run_on = 0;
static uint8_t run_step = 0;
static uint32_t run_num = 0;
static uint32_t run_speed = 0;
static uint32_t run_next = 0;

static void Trim(char *s)
{
    size_t len;

    if (s == NULL)
    {
        return;
    }

    len = strlen(s);

    while (len > 0)
    {
        char c = s[len - 1];

        if (c == '\r' || c == '\n' || c == ' ' || c == '\t')
        {
            s[len - 1] = '\0';
            len--;
        }
        else
        {
            break;
        }
    }
}

static void Upper(char *s)
{
    if (s == NULL)
    {
        return;
    }

    for (size_t i = 0; s[i] != '\0'; i++)
    {
        s[i] = (char)toupper((unsigned char)s[i]);
    }
}

static size_t Split(char *s, char **w)
{
    size_t n = 0;
    char *p = strtok(s, "_");

    while (p != NULL && n < TOK_MAX)
    {
        w[n++] = p;
        p = strtok(NULL, "_");
    }

    return n;
}

static void ClearPos(void)
{
    in_pos = 0;
    in_ok = 1;

    for (size_t i = 0; i < POS_MAX; i++)
    {
        out_pos[i] = 0;
        out_ok[i] = 0;
    }
}

static HAL_StatusTypeDef SaveFram(void)
{
    PosSave_t d;

    d.magic = FRAM_MAGIC;
    d.in_pos = in_pos;
    d.in_ok = in_ok;

    for (size_t i = 0; i < POS_MAX; i++)
    {
        d.out_pos[i] = out_pos[i];
        d.out_ok[i] = out_ok[i];
    }

    if ((FRAM_ADDR + sizeof(d)) > FRAM_MB85RS64_SIZE_BYTES)
    {
        return HAL_ERROR;
    }

    return Fram_Write(&hspi3, FRAM_ADDR, (uint8_t *)&d, (uint16_t)sizeof(d));
}

static void LoadFram(void)
{
    PosSave_t d;

    if (loaded != 0)
    {
        return;
    }

    loaded = 1;
    ClearPos();

    if ((FRAM_ADDR + sizeof(d)) > FRAM_MB85RS64_SIZE_BYTES)
    {
        return;
    }

    if (Fram_Read(&hspi3, FRAM_ADDR, (uint8_t *)&d, (uint16_t)sizeof(d)) != HAL_OK)
    {
        return;
    }

    if (d.magic != FRAM_MAGIC)
    {
        return;
    }

    in_pos = d.in_pos;
    in_ok = d.in_ok;

    for (size_t i = 0; i < POS_MAX; i++)
    {
        out_pos[i] = d.out_pos[i];
        out_ok[i] = d.out_ok[i];
    }
}

static uint8_t OutOk(uint32_t num)
{
    uint32_t index;

    if (num == 0)
    {
        return 0;
    }

    index = num - 1;

    if (index >= POS_MAX)
    {
        return 0;
    }

    return out_ok[index];
}

static int32_t OutPos(uint32_t num)
{
    return out_pos[num - 1];
}

static void SavePos(char *cmd, char *ans, size_t size)
{
    char *w[TOK_MAX];
    size_t n;
    long rack;
    long group;

    if (Motor_IsBusy() != 0)
    {
        snprintf(ans, size, "01E_BUSY\r\n");
        return;
    }

    n = Split(cmd, w);

    if (n < 4)
    {
        snprintf(ans, size, "01E_FP_ARG\r\n");
        return;
    }

    rack = strtol(w[1], NULL, 10);
    group = strtol(w[2], NULL, 10);

    if (rack != RACK_ID)
    {
        snprintf(ans, size, "01E_RACK\r\n");
        return;
    }

    if (group == GROUP_IN)
    {
        in_pos = Motor_mmToUnit((int32_t)strtol(w[3], NULL, 10));
        in_ok = 1;

        snprintf(ans, size, (SaveFram() == HAL_OK) ? "01FP_1_1_S\r\n" : "01E_FRAM\r\n");
        return;
    }

    if (group == GROUP_OUT)
    {
        size_t save_count = n - 3;

        if (save_count > POS_MAX)
        {
            save_count = POS_MAX;
        }

        for (size_t i = 0; i < POS_MAX; i++)
        {
            out_pos[i] = 0;
            out_ok[i] = 0;
        }

        for (size_t i = 0; i < save_count; i++)
        {
            out_pos[i] = Motor_mmToUnit((int32_t)strtol(w[3 + i], NULL, 10));
            out_ok[i] = 1;
        }

        snprintf(ans, size, (SaveFram() == HAL_OK) ? "01FP_1_2_S\r\n" : "01E_FRAM\r\n");
        return;
    }

    snprintf(ans, size, "01E_GROUP\r\n");
}

static void MoveOut(char *cmd, char *ans, size_t size)
{
    char *w[TOK_MAX];
    size_t n;
    long rack;
    long num;
    long speed;
    long start_ms;
    long wait_ms;
    osStatus_t result;

    n = Split(cmd, w);

    if (n < 6)
    {
        snprintf(ans, size, "01E_MO_ARG\r\n");
        return;
    }

    rack = strtol(w[1], NULL, 10);
    num = strtol(w[2], NULL, 10);
    speed = strtol(w[3], NULL, 10);
    start_ms = strtol(w[4], NULL, 10);
    wait_ms = strtol(w[5], NULL, 10);

    if (rack != RACK_ID || num <= 0 || speed < 0 || speed > 100 ||
        start_ms < 0 || wait_ms < 0 || OutOk((uint32_t)num) == 0)
    {
        snprintf(ans, size, "01E_MO_ARG\r\n");
        return;
    }

    run_on = 0;

    result = Motor_SendMove(OutPos((uint32_t)num),
                            (uint32_t)speed,
                            ACC_MS,
                            (uint32_t)start_ms,
                            (uint32_t)wait_ms);

    snprintf(ans, size, (result == osOK) ? "AO_1_%ld\r\n" : "01E_MO_BUSY\r\n", num);
}

static void MoveIn(char *cmd, char *ans, size_t size)
{
    char *w[TOK_MAX];
    size_t n;
    long rack;
    long num;
    long speed;
    long start_ms;
    long wait_ms;
    osStatus_t result;

    n = Split(cmd, w);

    if (n < 6)
    {
        snprintf(ans, size, "01E_MI_ARG\r\n");
        return;
    }

    rack = strtol(w[1], NULL, 10);
    num = strtol(w[2], NULL, 10);
    speed = strtol(w[3], NULL, 10);
    start_ms = strtol(w[4], NULL, 10);
    wait_ms = strtol(w[5], NULL, 10);

    if (rack != RACK_ID || num != 1 || in_ok == 0 ||
        speed < 0 || speed > 100 || start_ms < 0 || wait_ms < 0)
    {
        snprintf(ans, size, "01E_MI_ARG\r\n");
        return;
    }

    run_on = 0;

    result = Motor_SendMove(in_pos,
                            (uint32_t)speed,
                            ACC_MS,
                            (uint32_t)start_ms,
                            (uint32_t)wait_ms);

    snprintf(ans, size, (result == osOK) ? "AI_1_1\r\n" : "01E_MI_BUSY\r\n");
}

uint8_t MotionProtocol_Run(uint32_t num, uint32_t speed)
{
    LoadFram();

    if (speed > 100 || OutOk(num) == 0 || in_ok == 0 || Motor_IsEStop() != 0)
    {
        return 0;
    }

    run_num = num;
    run_speed = speed;
    run_step = 0;
    run_next = 0;
    run_on = 1;

    return 1;
}

void MotionProtocol_StopLoop(void)
{
    run_on = 0;
}

void MotionProtocol_Poll(void)
{
    uint32_t now;
    int32_t target;

    if (run_on == 0)
    {
        return;
    }

    if (Motor_IsEStop() != 0)
    {
        run_on = 0;
        return;
    }

    if (Motor_IsBusy() != 0)
    {
        return;
    }

    now = osKernelGetTickCount();

    if ((int32_t)(now - run_next) < 0)
    {
        return;
    }

    if (run_step == 0)
    {
        target = OutPos(run_num);
        run_step = 1;
    }
    else
    {
        target = in_pos;
        run_step = 0;
    }

    if (Motor_SendMove(target, run_speed, ACC_MS, 0, 0) == osOK)
    {
        run_next = now + 200;
    }
}

void MotionProtocol_ProcessCommand(const char *cmd_in,
                                   char *response,
                                   size_t response_size)
{
    char cmd[CMD_SIZE];

    if (response == NULL || response_size == 0)
    {
        return;
    }

    if (cmd_in == NULL)
    {
        snprintf(response, response_size, "01E_NULL\r\n");
        return;
    }

    snprintf(cmd, sizeof(cmd), "%s", cmd_in);
    Trim(cmd);
    Upper(cmd);
    LoadFram();

    if (strcmp(cmd, "02C") == 0)
    {
        snprintf(response,
                 response_size,
                 (Motor_IsEStop() != 0) ? "01_S_1_F&S\r\n" : "01_S_1_S&S\r\n");
        return;
    }

    if (strcmp(cmd, "01Q") == 0)
    {
        snprintf(response,
                 response_size,
                 "01Q_%u_%u_%u_%u_%ld_%ld\r\n",
                 run_on,
                 motor_state.moving,
                 motor_state.error,
                 motor_state.estop,
                 (long)motor_state.pos,
                 (long)motor_state.target);
        return;
    }

    if (strcmp(cmd, "01I") == 0)
    {
        run_on = 0;
        ClearPos();
        snprintf(response,
                 response_size,
                 (SaveFram() == HAL_OK) ? "01I_S\r\n" : "01E_FRAM\r\n");
        return;
    }

    if (strncmp(cmd, "01FP", 4) == 0)
    {
        run_on = 0;
        SavePos(cmd, response, response_size);
        return;
    }

    if (strncmp(cmd, "01MO", 4) == 0)
    {
        MoveOut(cmd, response, response_size);
        return;
    }

    if (strncmp(cmd, "01MI", 4) == 0)
    {
        MoveIn(cmd, response, response_size);
        return;
    }

    if (strcmp(cmd, "01STOP") == 0)
    {
        run_on = 0;
        snprintf(response,
                 response_size,
                 (Motor_SendStop() == osOK) ? "01STOP_S\r\n" : "01E_STOP\r\n");
        return;
    }

    if (strcmp(cmd, "01S") == 0)
    {
        run_on = 0;
        snprintf(response,
                 response_size,
                 (Motor_SendEStop() == osOK) ? "01_S_1_F&S\r\n" : "01E_S_QUEUE\r\n");
        return;
    }

    if (strcmp(cmd, "01D") == 0)
    {
        snprintf(response,
                 response_size,
                 (Motor_SendRelease() == osOK) ? "01D_S\r\n" : "01E_D_QUEUE\r\n");
        return;
    }

    snprintf(response, response_size, "01E_UNKNOWN\r\n");
}
