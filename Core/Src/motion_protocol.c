#include "motion_protocol.h"

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>

#include "motor.h"
#include "usart.h"

/*
 * motion_protocol.c
 *
 * 역할:
 *   TCP로 받은 문자열 명령을 해석해서 MotorQueue에 명령을 넣는다.
 *
 * 현재 구조:
 *   PC TCP 명령
 *   -> MotionProtocol_ProcessCommand()
 *   -> Motor_SendMove() / Motor_SendEStop() / Motor_SendRelease()
 *   -> MotorQueue
 *   -> StartMotorTask()
 *   -> motor.c
 *   -> motor_bus.c
 *   -> AIMotor
 *
 * 중요:
 *   여기서는 모터를 직접 움직이지 않는다.
 *   이동 명령은 Queue에 넣기만 한다.
 */

#define CMD_BUF_SIZE           512U
#define TOKEN_MAX              64U
#define POS_MAX                32U

#define RACK_ID                1L

#define GROUP_IN               1L
#define GROUP_OUT              2L

#define DEFAULT_ACC_MS         1000U

/*
 * 입고/복귀 위치.
 * 현재는 1번 위치만 사용한다.
 */
static int32_t in_pos[POS_MAX];
static uint8_t in_ok[POS_MAX];

/*
 * 출고 위치.
 * 01FP_1_2_1000_2000_3000 처럼 저장된다.
 */
static int32_t out_pos[POS_MAX];
static uint8_t out_ok[POS_MAX];

/*
 * 문자열 끝 공백 제거.
 *
 * 예:
 *   "02C\r\n" -> "02C"
 */
static void Trim(char *text)
{
    size_t len;

    if (text == NULL)
    {
        return;
    }

    len = strlen(text);

    while (len > 0U)
    {
        char c;

        c = text[len - 1U];

        if ((c == '\r') || (c == '\n') || (c == ' ') || (c == '\t'))
        {
            text[len - 1U] = '\0';
            len--;
        }
        else
        {
            break;
        }
    }
}

/*
 * 문자열을 대문자로 변경.
 *
 * 예:
 *   "02c" -> "02C"
 */
static void Upper(char *text)
{
    size_t i;

    if (text == NULL)
    {
        return;
    }

    for (i = 0U; text[i] != '\0'; i++)
    {
        text[i] = (char)toupper((unsigned char)text[i]);
    }
}

/*
 * '_' 기준으로 문자열 나누기.
 *
 * 예:
 *   "01MO_1_3_80_500_500"
 *
 * 결과:
 *   word[0] = "01MO"
 *   word[1] = "1"
 *   word[2] = "3"
 *   word[3] = "80"
 *   word[4] = "500"
 *   word[5] = "500"
 */
static size_t Split(char *text, char **word, size_t max_count)
{
    size_t count;
    char *part;

    count = 0U;

    if ((text == NULL) || (word == NULL))
    {
        return 0U;
    }

    part = strtok(text, "_");

    while (part != NULL)
    {
        if (count >= max_count)
        {
            break;
        }

        word[count] = part;
        count++;

        part = strtok(NULL, "_");
    }

    return count;
}

/*
 * 02C 상태 확인.
 */
static void Status(char *ans, size_t size)
{
    if (Motor_IsEStop() != 0U)
    {
        snprintf(ans, size, "01_S_1_F&S\r\n");
    }
    else
    {
        snprintf(ans, size, "01_S_1_S&S\r\n");
    }
}

/*
 * 01I 위치 저장값 초기화.
 */
static void Init(char *ans, size_t size)
{
    size_t i;

    if (Motor_IsBusy() != 0U)
    {
        snprintf(ans, size, "01E_BUSY\r\n");
        return;
    }

    for (i = 0U; i < POS_MAX; i++)
    {
        in_pos[i] = 0L;
        in_ok[i] = 0U;

        out_pos[i] = 0L;
        out_ok[i] = 0U;
    }

    Motor_ClearHome();

    snprintf(ans, size, "01I_S\r\n");
}

/*
 * 01FP_1_1_0_0 Home 위치 저장.
 *
 * 현재는 실제 원점복귀가 아니라 STM 내부 software home이다.
 */
static void Home(char *ans, size_t size)
{
    if (Motor_IsBusy() != 0U)
    {
        snprintf(ans, size, "01E_BUSY\r\n");
        return;
    }

#if (USE_HOME_SENSOR != 0U)
    if (HomeSensor_IsDetected() == 0U)
    {
        snprintf(ans, size, "01E_HOME_SENSOR\r\n");
        return;
    }
#endif

    Motor_SetHome();

    in_pos[0] = 0L;
    in_ok[0] = 1U;

    snprintf(ans, size, "01FP_1_1_S\r\n");
}

/*
 * 01FP 위치 저장.
 *
 * 예:
 *   01FP_1_1_0_0
 *     -> Home 저장
 *
 *   01FP_1_2_1000_2000_3000
 *     -> 출고 위치 1번 = 1000
 *     -> 출고 위치 2번 = 2000
 *     -> 출고 위치 3번 = 3000
 */
static void Save(char *cmd, char *ans, size_t size)
{
    char *word[TOKEN_MAX];
    size_t count;
    long rack;
    long group;

    count = Split(cmd, word, TOKEN_MAX);

    if (count < 4U)
    {
        snprintf(ans, size, "01E_FP_ARG\r\n");
        return;
    }

    rack = strtol(word[1], NULL, 10);
    group = strtol(word[2], NULL, 10);

    if (rack != RACK_ID)
    {
        snprintf(ans, size, "01E_RACK\r\n");
        return;
    }

    /*
     * group 1:
     *   입고/복귀 위치.
     *   현재는 0이면 home 저장으로 사용한다.
     */
    if (group == GROUP_IN)
    {
        long first_value;

        first_value = strtol(word[3], NULL, 10);

        if (first_value == 0L)
        {
            Home(ans, size);
            return;
        }

        snprintf(ans, size, "01E_GROUP1_VAL\r\n");
        return;
    }

    /*
     * group 2:
     *   출고 위치 여러 개 저장.
     */
    if (group == GROUP_OUT)
    {
        size_t i;
        size_t save_count;

        if (Motor_IsBusy() != 0U)
        {
            snprintf(ans, size, "01E_BUSY\r\n");
            return;
        }

        save_count = count - 3U;

        if (save_count > POS_MAX)
        {
            save_count = POS_MAX;
        }

        for (i = 0U; i < POS_MAX; i++)
        {
            out_pos[i] = 0L;
            out_ok[i] = 0U;
        }

        for (i = 0U; i < save_count; i++)
        {
            out_pos[i] = (int32_t)strtol(word[3U + i], NULL, 10);
            out_ok[i] = 1U;
        }

        snprintf(ans, size, "01FP_1_2_S\r\n");
        return;
    }

    snprintf(ans, size, "01E_GROUP\r\n");
}

/*
 * 01MO 출고 위치로 이동.
 *
 * 예:
 *   01MO_1_3_80_500_500
 *
 * 의미:
 *   rack 번호       = 1
 *   출고 위치 번호  = 3
 *   속도           = 80%
 *   출발 전 대기    = 500ms
 *   이동 후 대기    = 500ms
 *
 * 정상 응답:
 *   AO_1_3
 *
 * 주의:
 *   이 응답은 이동 완료가 아니라 Queue 접수 성공이다.
 */
static void Out(char *cmd, char *ans, size_t size)
{
    char *word[TOKEN_MAX];
    size_t count;
    long rack;
    long num;
    long speed;
    long start_ms;
    long wait_ms;
    size_t pos_index;
    int32_t target_pos;
    osStatus_t q;

    count = Split(cmd, word, TOKEN_MAX);

    if (count < 6U)
    {
        snprintf(ans, size, "01E_MO_ARG\r\n");
        return;
    }

    rack = strtol(word[1], NULL, 10);
    num = strtol(word[2], NULL, 10);
    speed = strtol(word[3], NULL, 10);
    start_ms = strtol(word[4], NULL, 10);
    wait_ms = strtol(word[5], NULL, 10);

    if (rack != RACK_ID)
    {
        snprintf(ans, size, "01E_RACK\r\n");
        return;
    }

    if (num <= 0L)
    {
        snprintf(ans, size, "01E_MO_IDX\r\n");
        return;
    }

    if ((speed < 0L) || (speed > 100L))
    {
        snprintf(ans, size, "01E_MO_SPEED\r\n");
        return;
    }

    if ((start_ms < 0L) || (wait_ms < 0L))
    {
        snprintf(ans, size, "01E_MO_TIME\r\n");
        return;
    }

    if (Motor_IsEStop() != 0U)
    {
        snprintf(ans, size, "01E_MO_ESTOP\r\n");
        return;
    }

    if (Motor_HomeOk() == 0U)
    {
        snprintf(ans, size, "01E_MO_HOME\r\n");
        return;
    }

    pos_index = (size_t)(num - 1L);

    if ((pos_index >= POS_MAX) || (out_ok[pos_index] == 0U))
    {
        snprintf(ans, size, "01E_MO_IDX\r\n");
        return;
    }

    target_pos = out_pos[pos_index];

    q = Motor_SendMove(
        target_pos,
        (uint32_t)speed,
        DEFAULT_ACC_MS,
        (uint32_t)start_ms,
        (uint32_t)wait_ms
    );

    if (q == osOK)
    {
        snprintf(ans, size, "AO_1_%ld\r\n", num);
    }
    else
    {
        snprintf(ans, size, "01E_MO_BUSY\r\n");
    }
}

/*
 * 01MI 입고/복귀 위치로 이동.
 *
 * 예:
 *   01MI_1_1_80_500_500
 *
 * 현재 구조:
 *   입고/복귀 위치는 1번만 사용한다.
 *
 * 정상 응답:
 *   AI_1_1
 *
 * 주의:
 *   이 응답은 이동 완료가 아니라 Queue 접수 성공이다.
 */
static void In(char *cmd, char *ans, size_t size)
{
    char *word[TOKEN_MAX];
    size_t count;
    long rack;
    long num;
    long speed;
    long start_ms;
    long wait_ms;
    int32_t target_pos;
    osStatus_t q;

    count = Split(cmd, word, TOKEN_MAX);

    if (count < 6U)
    {
        snprintf(ans, size, "01E_MI_ARG\r\n");
        return;
    }

    rack = strtol(word[1], NULL, 10);
    num = strtol(word[2], NULL, 10);
    speed = strtol(word[3], NULL, 10);
    start_ms = strtol(word[4], NULL, 10);
    wait_ms = strtol(word[5], NULL, 10);

    if (rack != RACK_ID)
    {
        snprintf(ans, size, "01E_RACK\r\n");
        return;
    }

    if ((num != 1L) || (in_ok[0] == 0U))
    {
        snprintf(ans, size, "01E_MI_IDX\r\n");
        return;
    }

    if ((speed < 0L) || (speed > 100L))
    {
        snprintf(ans, size, "01E_MI_SPEED\r\n");
        return;
    }

    if ((start_ms < 0L) || (wait_ms < 0L))
    {
        snprintf(ans, size, "01E_MI_TIME\r\n");
        return;
    }

    if (Motor_IsEStop() != 0U)
    {
        snprintf(ans, size, "01E_MI_ESTOP\r\n");
        return;
    }

    if (Motor_HomeOk() == 0U)
    {
        snprintf(ans, size, "01E_MI_HOME\r\n");
        return;
    }

    target_pos = in_pos[0];

    q = Motor_SendMove(
        target_pos,
        (uint32_t)speed,
        DEFAULT_ACC_MS,
        (uint32_t)start_ms,
        (uint32_t)wait_ms
    );

    if (q == osOK)
    {
        snprintf(ans, size, "AI_1_1\r\n");
    }
    else
    {
        snprintf(ans, size, "01E_MI_BUSY\r\n");
    }
}

/*
 * 01S 비상정지.
 */
static void Stop(char *ans, size_t size)
{
    if (Motor_SendEStop() == osOK)
    {
        snprintf(ans, size, "01_S_1_F&S\r\n");
    }
    else
    {
        snprintf(ans, size, "01E_S_QUEUE\r\n");
    }
}

/*
 * 01D 비상정지 해제.
 */
static void Release(char *ans, size_t size)
{
    if (Motor_SendRelease() == osOK)
    {
        snprintf(ans, size, "01D_S\r\n");
    }
    else
    {
        snprintf(ans, size, "01E_D_QUEUE\r\n");
    }
}

/*
 * TCP 명령 처리 진입점.
 */
void MotionProtocol_ProcessCommand(
    const char *cmd_in,
    char *response,
    size_t response_size
)
{
    char cmd[CMD_BUF_SIZE];

    if ((response == NULL) || (response_size == 0U))
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

    if (strcmp(cmd, "02C") == 0)
    {
        Status(response, response_size);
    }
    else if (strcmp(cmd, "01I") == 0)
    {
        Init(response, response_size);
    }
    else if (strncmp(cmd, "01FP", 4U) == 0)
    {
        Save(cmd, response, response_size);
    }
    else if (strncmp(cmd, "01MO", 4U) == 0)
    {
        Out(cmd, response, response_size);
    }
    else if (strncmp(cmd, "01MI", 4U) == 0)
    {
        In(cmd, response, response_size);
    }

    else if (strcmp(cmd, "01S") == 0)
    {
        Stop(response, response_size);
    }
    else if (strcmp(cmd, "01D") == 0)
    {
        Release(response, response_size);
    }
    else
    {
        snprintf(response, response_size, "01E_UNKNOWN\r\n");
    }

}


