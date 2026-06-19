#include "motion_protocol.h"
#include "motor.h"
#include "storage.h"
#include "usart.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#define CMD_SIZE      192
#define WORD_MAX      20
#define RACK_ID       1
#define G_IN          1
#define G_OUT         2
#define ACC_MS        1000
#define RUN_START_MS  0
#define RUN_WAIT_MS   200
#define HOLD_RETRY_MS 300  // ?
static int32_t in_mm = 0;
static int32_t out_mm[ST_MAX];
static uint8_t in_ok = 0;
static uint8_t out_ok[ST_MAX];
static uint8_t loaded = 0;
static uint8_t run_on = 0;
static uint8_t run_step = 0;
static uint8_t run_wait = 0;
static uint32_t run_no = 0;
static uint32_t run_speed = 0;
static uint32_t run_next = 0;
static uint32_t run_seq = 0;
static int32_t run_target = 0;
static uint8_t run_next_step = 0;
static char run_msg[64], tcp_msg[128];
static volatile uint8_t tcp_on = 0;
static void trim(char *s)
{
    size_t n = strlen(s);
    while (n > 0)
    {
        char c = s[n - 1];
        if (c == '\r' || c == '\n' || c == ' ' || c == '\t') s[--n] = '\0';
        else break;
    }
}
static void up(char *s)
{
    for (uint32_t i = 0; s[i] != '\0'; i++)
        s[i] = (char)toupper((unsigned char)s[i]);
}
static size_t cut(char *s, char *w[])
{
    size_t n = 0;
    char *p = strtok(s, "_");
    while (p != NULL && n < WORD_MAX)
    {
        w[n++] = p;
        p = strtok(NULL, "_");
    }
    return n;
}
static void push(const char *s)
{
    if (s[0] == '\0') return;
    snprintf(tcp_msg, sizeof(tcp_msg), "%s", s);
    tcp_on = 1;
}
uint8_t MotionProtocol_TakeTcpLog(char *out, size_t size)
{
    if (tcp_on == 0 || size == 0) return 0;
    snprintf(out, size, "%s", tcp_msg);
    tcp_on = 0;
    return 1;
}
static void clr(void)
{
    in_mm = 0;
    in_ok = 0;
    for (uint32_t i = 0; i < ST_MAX; i++) { out_mm[i] = 0; out_ok[i] = 0; }
}
static void load(void)
{
    uint32_t count;
    if (loaded != 0) return;
    loaded = 1;
    clr();
    (void)St_Load();
    if (St_InOk() != 0) { in_mm = St_In(); in_ok = 1; }
    count = St_Count();
    if (count > ST_MAX) count = ST_MAX;
    for (uint32_t i = 0; i < count; i++)
    {
        out_mm[i] = St_Out(i);
        out_ok[i] = 1;
    }
}
static uint8_t ok(uint32_t no)
{
    if (no == 0 || no > ST_MAX) return 0;
    return out_ok[no - 1];
}
static int32_t pos(uint32_t no)
{
    return out_mm[no - 1];
}
static void wheel(char *cmd, char *res, size_t size)
{
    char *w[WORD_MAX];
    uint32_t dia;
    if (cut(cmd, w) < 2) { snprintf(res, size, "01E_WHEEL_ARG\r\n"); return; }
    dia = (uint32_t)strtoul(w[1], NULL, 10);
    if (dia == 0) { snprintf(res, size, "01E_WHEEL_VAL\r\n"); return; }
    St_SetDia(dia);
    snprintf(res, size,
             (St_Save() == HAL_OK) ? "01W_S_%lu\r\n" : "01E_FRAM\r\n",
             (unsigned long)dia);
}
static void save(char *cmd, char *res, size_t size)
{
    char *w[WORD_MAX];
    size_t n;
    long rack, group;
    if (Motor_IsBusy() != 0) { snprintf(res, size, "01E_BUSY\r\n"); return; }
    n = cut(cmd, w);
    if (n < 4) { snprintf(res, size, "01E_FP_ARG\r\n"); return; }
    rack = strtol(w[1], NULL, 10);
    group = strtol(w[2], NULL, 10);
    if (rack != RACK_ID) { snprintf(res, size, "01E_RACK\r\n"); return; }
    if (group == G_IN)
    {
        in_mm = (int32_t)strtol(w[3], NULL, 10);
        in_ok = 1;
        St_SetIn(in_mm, 1);
        snprintf(res, size,
                 (St_Save() == HAL_OK) ? "01FP_1_1_S\r\n" : "01E_FRAM\r\n");
        return;
    }
    if (group == G_OUT)
    {
        size_t count = n - 3;
        if (count > ST_MAX) count = ST_MAX;
        for (uint32_t i = 0; i < ST_MAX; i++) { out_mm[i] = 0; out_ok[i] = 0; }
        for (size_t i = 0; i < count; i++)
        {
            out_mm[i] = (int32_t)strtol(w[i + 3], NULL, 10);
            out_ok[i] = 1;
        }
        St_SetOut(out_mm, (uint32_t)count);
        snprintf(res, size,
                 (St_Save() == HAL_OK) ? "01FP_1_2_S\r\n" : "01E_FRAM\r\n");
        return;
    }
    snprintf(res, size, "01E_GROUP\r\n");
}
static void show(char *res, size_t size)
{
    uint32_t used, count;
    loaded = 0;
    load();
    count = St_Count();
    if (count > ST_MAX) count = ST_MAX;
    used = (uint32_t)snprintf(res, size,
        "FRAM_S\r\nwheel=%lumm\r\nzero_ok=%lu\r\nzero=%ld\r\nin_ok=%lu\r\nin=%ldmm\r\nout_count=%lu\r\n",
        (unsigned long)St_Dia(), (unsigned long)St_ZeroOk(), (long)St_Zero(),
        (unsigned long)St_InOk(), (long)St_In(), (unsigned long)count);
    for (uint32_t i = 0; i < count && used < size; i++)
    {
        int n = snprintf(&res[used], size - used,
                         "out%lu=%ldmm\r\n",
                         (unsigned long)(i + 1), (long)St_Out(i));
        if (n < 0) break;
        used += (uint32_t)n;
    }
}
static void mo(char *cmd, char *res, size_t size)
{
    char *w[WORD_MAX];
    long rack, no, speed, start_ms, wait_ms;
    int32_t target;
    if (cut(cmd, w) < 6) { snprintf(res, size, "01E_MO_ARG\r\n"); return; }
    rack = strtol(w[1], NULL, 10);
    no = strtol(w[2], NULL, 10);
    speed = strtol(w[3], NULL, 10);
    start_ms = strtol(w[4], NULL, 10);
    wait_ms = strtol(w[5], NULL, 10);
    if (rack != RACK_ID || no <= 0 || speed < 0 || speed > 100 ||
        start_ms < 0 || wait_ms < 0 || ok((uint32_t)no) == 0)
    {
        snprintf(res, size, "01E_MO_ARG\r\n");
        return;
    }
    run_on = 0;
    target = Motor_TargetUnit(pos((uint32_t)no));
    snprintf(res, size,
             (Motor_SendMove(target, (uint32_t)speed, ACC_MS,
                             (uint32_t)start_ms, (uint32_t)wait_ms) == osOK) ?
             "AO_1_%ld\r\n" : "01E_MO_BUSY\r\n", no);
}
static void mi(char *cmd, char *res, size_t size)
{
    char *w[WORD_MAX];
    long rack, no, speed, start_ms, wait_ms;
    int32_t target;
    if (cut(cmd, w) < 6) { snprintf(res, size, "01E_MI_ARG\r\n"); return; }
    rack = strtol(w[1], NULL, 10);
    no = strtol(w[2], NULL, 10);
    speed = strtol(w[3], NULL, 10);
    start_ms = strtol(w[4], NULL, 10);
    wait_ms = strtol(w[5], NULL, 10);
    if (rack != RACK_ID || no != 1 || in_ok == 0 ||
        speed < 0 || speed > 100 || start_ms < 0 || wait_ms < 0)
    {
        snprintf(res, size, "01E_MI_ARG\r\n");
        return;
    }
    run_on = 0;
    target = Motor_TargetUnit(in_mm);
    snprintf(res, size,
             (Motor_SendMove(target, (uint32_t)speed, ACC_MS,
                             (uint32_t)start_ms, (uint32_t)wait_ms) == osOK) ?
             "AI_1_1\r\n" : "01E_MI_BUSY\r\n");
}
/* run 1스텝 시작 */
static void step(void)
{
    int32_t target;
    uint8_t next;

    if (run_step == 0)
    {
        target = Motor_TargetUnit(pos(run_no));
        next = 1;

        snprintf(run_msg, sizeof(run_msg),
                 "01MO_1_%lu_%lu_%lu_%lu\r\n",
                 (unsigned long)run_no,
                 (unsigned long)run_speed,
                 (unsigned long)RUN_START_MS,
                 (unsigned long)RUN_WAIT_MS);
    }
    else
    {
        target = Motor_TargetUnit(in_mm);
        next = 0;

        snprintf(run_msg, sizeof(run_msg),
                 "01MI_1_1_%lu_%lu_%lu\r\n",
                 (unsigned long)run_speed,
                 (unsigned long)RUN_START_MS,
                 (unsigned long)RUN_WAIT_MS);
    }

    if (Motor_SendMove(target, run_speed, ACC_MS,
                       RUN_START_MS, RUN_WAIT_MS) == osOK)
    {
        run_target = target;
        run_next_step = next;
        run_seq = motor_state.seq;
        run_wait = 1;
    }
}
static void loop(char *cmd, char *res, size_t size)
{
    char *w[WORD_MAX];
    long rack, no, speed;
    if (cut(cmd, w) < 4) { snprintf(res, size, "01E_RUN_ARG\r\n"); return; }
    rack = strtol(w[1], NULL, 10);
    no = strtol(w[2], NULL, 10);
    speed = strtol(w[3], NULL, 10);
    if (rack != RACK_ID || no <= 0 || speed < 0 || speed > 100)
    {
        snprintf(res, size, "01E_RUN_ARG\r\n");
        return;
    }
    snprintf(res, size,
             (MotionProtocol_Run((uint32_t)no, (uint32_t)speed) != 0) ?
             "01RUN_S\r\n" : "01E_RUN\r\n");
}
/* run 시작 */
uint8_t MotionProtocol_Run(uint32_t no, uint32_t speed)
{
    load();

    if (speed > 100 || ok(no) == 0 || in_ok == 0 ||
        Motor_IsBusy() != 0 || Motor_IsEStop() != 0)
    {
        return 0;
    }

    run_no = no;
    run_speed = speed;
    run_step = 0;
    run_wait = 0;
    run_next = 0;
    run_seq = motor_state.seq;
    run_target = 0;
    run_next_step = 0;
    run_on = 1;

    Motor_ClearHold();

    return 1;
}
void MotionProtocol_StopLoop(void)
{
    run_on = 0;
    run_wait = 0;
    Motor_ClearHold();
}
/* run 반복 처리 */
void MotionProtocol_Poll(void)
{
    static uint8_t estop_printed = 0;
    uint32_t now = osKernelGetTickCount();

    if (Motor_IsEStop() != 0)
    {
        if (run_on != 0)
        {
            MotionProtocol_StopLoop();

            if (estop_printed == 0)
            {
                push("01STOP_S\r\n");
                printf("01STOP_S\r\n");
                estop_printed = 1;
            }
        }
        return;
    }

    estop_printed = 0;

    if (run_on == 0) return;

    if (run_wait != 0)
    {
        if (Motor_IsBusy() == 0 && motor_state.seq != run_seq)
        {
            if (Motor_IsHold() != 0)
            {
                /*
                 * 광센서로 목표 전 중간 정지.
                 *
                 * run_step == 1이면 return 방향이었다는 뜻.
                 * 즉 원래 목표는 FRAM zero였지만, 광센서 때문에 중간에서 멈춘 것.
                 * 이 위치를 임시 return endpoint처럼 인정하고 out 방향으로 넘긴다.
                 */
                if (run_step == 1)
                {
                    Motor_ClearHold();

                    run_step = 0;
                    run_wait = 0;
                    run_next = now + RUN_WAIT_MS;

                    push("01HOLD_S\r\n");
                    printf("01HOLD_S\r\n");
                    return;
                }

                /*
                 * out 방향에서 목표 전에 멈춘 것은 의도한 임시 zero 동작이 아님.
                 * 이 경우는 안전하게 run을 멈춘다.
                 */
                MotionProtocol_StopLoop();
                push("01HOLD_S\r\n");
                printf("01HOLD_S\r\n");
                return;
            }

            /*
             * 정상 도착.
             * 이때만 다음 step으로 넘어간다.
             */
            push(run_msg);

            run_step = run_next_step;
            run_wait = 0;
            run_next = now + RUN_WAIT_MS;
            return;
        }

        if (Motor_IsBusy() == 0 && motor_state.error != 0)
        {
            MotionProtocol_StopLoop();
            push("01STOP_S\r\n");
            printf("01STOP_S\r\n");
            return;
        }

        return;
    }

    if ((int32_t)(now - run_next) >= 0)
    {
        step();
    }
}
void MotionProtocol_ProcessCommand(const char *cmd_in, char *res, size_t size)
{
    char cmd[CMD_SIZE];
    if (res == NULL || size == 0) return;
    res[0] = '\0';
    if (cmd_in == NULL) { snprintf(res, size, "01E_NULL\r\n"); return; }
    snprintf(cmd, sizeof(cmd), "%s", cmd_in);
    trim(cmd);
    up(cmd);
    load();
    if (strncmp(cmd, "01W", 3) == 0) wheel(cmd, res, size);
    else if (strncmp(cmd, "01FP", 4) == 0)
    {
        MotionProtocol_StopLoop();
        save(cmd, res, size);
    }
    else if (strncmp(cmd, "01MO", 4) == 0) mo(cmd, res, size);
    else if (strncmp(cmd, "01MI", 4) == 0) mi(cmd, res, size);
    else if (strncmp(cmd, "01RUN", 5) == 0) loop(cmd, res, size);
    else if (strcmp(cmd, "01STOP") == 0)
    {
        MotionProtocol_StopLoop();
        snprintf(res, size,
                 (Motor_SendStop() == osOK) ? "01STOP_S\r\n" : "01E_STOP\r\n");
        push(res);
    }
    else if (strcmp(cmd, "01I") == 0)
    {
        MotionProtocol_StopLoop();
        clr();
        St_SetZero(0, 0);
        St_SetIn(0, 0);
        St_SetOut(out_mm, 0);
        snprintf(res, size,
                 (St_Save() == HAL_OK) ? "01I_S\r\n" : "01E_FRAM\r\n");
    }
    else if (strcmp(cmd, "01HOME") == 0)
    {
        MotionProtocol_StopLoop();
        snprintf(res, size,
                 (Motor_Home(&huart5) == HAL_OK) ? "01HOME_S\r\n" : "01E_HOME\r\n");
    }
    else if (strcmp(cmd, "01ZERO") == 0)
    {
        MotionProtocol_StopLoop();
        snprintf(res, size,
                 (Motor_Zero(&huart5) == HAL_OK) ? "01ZERO_S\r\n" : "01E_ZERO\r\n");
        loaded = 0;
        load();
    }
    else if (strncmp(cmd, "01P", 3) == 0)
    {
        char *w[WORD_MAX];
        char tmp[CMD_SIZE];
        snprintf(tmp, sizeof(tmp), "%s", cmd);
        if (cut(tmp, w) < 2) snprintf(res, size, "01E_PWR_ARG\r\n");
        else if (strcmp(w[1], "1") == 0 || strcmp(w[1], "ON") == 0)
            snprintf(res, size, (Motor_Power(&huart5, 1) == HAL_OK) ?
                     "01P_ON_S\r\n" : "01E_PWR\r\n");
        else if (strcmp(w[1], "0") == 0 || strcmp(w[1], "OFF") == 0)
            snprintf(res, size, (Motor_Power(&huart5, 0) == HAL_OK) ?
                     "01P_OFF_S\r\n" : "01E_PWR\r\n");
        else snprintf(res, size, "01E_PWR_ARG\r\n");
    }
    else if (strcmp(cmd, "01D28") == 0)
        snprintf(res, size,
                 (Motor_Do28(&huart5) == HAL_OK) ? "01D28_S\r\n" : "01E_D28\r\n");
    else if (strcmp(cmd, "01ECLR") == 0)
    {
        MotionProtocol_StopLoop();
        Motor_ClearEStop();
        snprintf(res, size, "01ECLR_S\r\n");
    }
    else if (strcmp(cmd, "01Q") == 0) show(res, size);
    else ;
}
void MotionProtocol_Command(const char *cmd_in, char *res, size_t size)
{
    MotionProtocol_ProcessCommand(cmd_in, res, size);
}
