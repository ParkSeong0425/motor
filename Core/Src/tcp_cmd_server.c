/*
 * tcp_cmd_server.c
 *
 * 이 파일의 역할
 * ------------------------------------------------------------
 * W6100의 TCP socket을 사용해서 PC 명령을 받는 서버입니다.
 *
 * 현재 구조:
 * ------------------------------------------------------------
 * PC PowerShell
 *   ↓ TCP 5000
 * W6100 socket 0
 *   ↓ recv()
 * TcpCmdServer_Process()
 *   ↓
 * MotionProtocol_ProcessCommand()
 *   ↓
 * send()
 *   ↓
 * PC PowerShell
 *
 * 중요:
 * ------------------------------------------------------------
 * 실제 IP 주소는 main.c의 netinfo_set에서 관리합니다.
 *
 * 예:
 *   .ip = {172, 20, 0, 192}
 *
 * 이 파일에서는 실제 IP 주소를 정하지 않습니다.
 * 이 파일은 W6100 socket을 IPv4 TCP / IPv6 TCP / Dual TCP 중
 * 어떤 mode로 열지만 결정합니다.
 */

#include "tcp_cmd_server.h"
#include "motion_protocol.h"

#include "socket.h"
#include "wizchip_conf.h"

#include <string.h>
#include <stdio.h>

/*
 * TCP 응답 문자열 버퍼 크기입니다.
 *
 * STATUS 응답은 짧지만,
 * MOTOR_SETUP_SIM,LIFT 같은 응답은 길어질 수 있으므로
 * 512 정도로 넉넉하게 잡습니다.
 */
#define TCP_RESPONSE_BUF_SIZE 512U

/*
 * 아래 변수들은 디버깅용 전역 변수입니다.
 *
 * STM32CubeIDE Expressions 창에 추가하면
 * TCP 서버가 어떤 상태인지 확인할 수 있습니다.
 */
volatile uint8_t tcp_state_debug = 0U;          /* 현재 socket 상태 */
volatile int32_t tcp_ret_debug = 0;             /* 마지막 socket/recv/send 반환값 */
volatile uint16_t tcp_rx_size_debug = 0U;       /* W6100 RX 버퍼에 쌓인 수신 크기 */
volatile uint32_t tcp_rx_count_debug = 0U;      /* recv 성공 횟수 */
volatile uint32_t tcp_tx_count_debug = 0U;      /* send 성공 횟수 */
volatile uint32_t tcp_socket_open_count = 0U;   /* socket 생성 성공 횟수 */
volatile uint32_t tcp_listen_count = 0U;        /* listen 호출 횟수 */
volatile uint32_t tcp_close_count = 0U;         /* close/disconnect 횟수 */

/**
 * @brief TCP_CMD_MODE_IPV4 / IPV6 / DUAL 값을 W6100 socket mode로 변환
 *
 * W6100에서 socket()을 열 때는 아래 mode가 필요합니다.
 *
 * IPv4 TCP  -> Sn_MR_TCP4
 * IPv6 TCP  -> Sn_MR_TCP6
 * Dual TCP  -> Sn_MR_TCPD
 *
 * 현재 현장 기본은 IPv4이므로 freertos.c에서는 TCP_CMD_MODE_IPV4를 넘깁니다.
 */
static uint8_t TcpModeToSocketMode(uint8_t ip_mode)
{
    if (ip_mode == TCP_CMD_MODE_IPV4)
    {
        return Sn_MR_TCP4;
    }
    else if (ip_mode == TCP_CMD_MODE_IPV6)
    {
        return Sn_MR_TCP6;
    }
    else if (ip_mode == TCP_CMD_MODE_DUAL)
    {
        return Sn_MR_TCPD;
    }
    else
    {
        /*
         * 잘못된 값이 들어오면 안전하게 IPv4로 처리합니다.
         */
        return Sn_MR_TCP4;
    }
}

/**
 * @brief TCP 명령 서버 처리 함수
 *
 * 이 함수는 한 번 호출한다고 끝나는 함수가 아닙니다.
 * FreeRTOS에서는 TcpTask 안에서 계속 반복 호출해야 합니다.
 *
 * 내부 동작 흐름:
 * ------------------------------------------------------------
 * SOCK_CLOSED:
 *   socket()으로 TCP socket 생성
 *
 * SOCK_INIT:
 *   listen()으로 PC 접속 대기 상태 진입
 *
 * SOCK_LISTEN:
 *   PC 접속 대기
 *
 * SOCK_ESTABLISHED:
 *   PC와 연결된 상태
 *   getSn_RX_RSR()로 받은 데이터가 있는지 확인
 *   recv()로 데이터를 읽고
 *   MotionProtocol_ProcessCommand()로 명령을 해석한 뒤
 *   send()로 응답 전송
 *
 * SOCK_CLOSE_WAIT:
 *   PC가 연결 종료 요청을 보낸 상태
 *   disconnect()로 socket 정리
 */
int32_t TcpCmdServer_Process(
    uint8_t sn,
    uint8_t *buf,
    uint16_t port,
    uint8_t ip_mode
)
{
    int32_t ret;
    uint16_t size;
    uint16_t send_len;
    char response[TCP_RESPONSE_BUF_SIZE];
    uint8_t status;
    uint8_t socket_mode;

    status = getSn_SR(sn);

    tcp_state_debug = status;
    tcp_rx_size_debug = getSn_RX_RSR(sn);

    switch (status)
    {
    case SOCK_ESTABLISHED:
        /*
         * PC와 TCP 연결이 된 상태입니다.
         * Sn_IR_CON 비트가 켜져 있으면 새 연결 이벤트가 발생했다는 뜻입니다.
         */
        if ((getSn_IR(sn) & Sn_IR_CON) != 0U)
        {
            setSn_IR(sn, Sn_IR_CON);
        }

        size = getSn_RX_RSR(sn);
        tcp_rx_size_debug = size;

        if (size > 0U)
        {
            /*
             * tcp_buf 크기가 2048이므로 마지막에 '\0'을 넣기 위해
             * 최대 2047까지만 받습니다.
             */
            if (size > 2047U)
            {
                size = 2047U;
            }

            ret = recv(sn, buf, size);
            tcp_ret_debug = ret;

            if (ret <= 0)
            {
                return ret;
            }

            tcp_rx_count_debug++;

            /*
             * C 문자열로 처리하기 위해 NULL 종료를 넣습니다.
             */
            buf[ret] = '\0';

            /*
             * TCP 명령 처리
             */
            MotionProtocol_ProcessCommand(
                (const char *)buf,
                response,
                sizeof(response)
            );

            send_len = (uint16_t)strlen(response);

            ret = send(sn, (uint8_t *)response, send_len);
            tcp_ret_debug = ret;

            if (ret < 0)
            {
                close(sn);
                tcp_close_count++;
                return ret;
            }

            tcp_tx_count_debug++;
        }
        break;

    case SOCK_CLOSE_WAIT:
        /*
         * PC 쪽에서 연결 종료를 요청한 상태입니다.
         */
        disconnect(sn);
        tcp_close_count++;
        break;

    case SOCK_INIT:
        /*
         * socket은 생성되었지만 아직 listen 상태가 아닙니다.
         */
        ret = listen(sn);
        tcp_ret_debug = ret;

        if (ret == SOCK_OK)
        {
            tcp_listen_count++;
        }
        else
        {
            return ret;
        }
        break;

    case SOCK_CLOSED:
        /*
         * socket이 닫혀 있는 상태입니다.
         * 전달받은 ip_mode에 따라 IPv4 / IPv6 / Dual TCP socket을 엽니다.
         *
         * 현재 기본값:
         *   freertos.c -> TCP_CMD_MODE_IPV4
         *
         * 실제 IP 주소:
         *   main.c -> netinfo_set.ip
         */
        socket_mode = TcpModeToSocketMode(ip_mode);

        ret = socket(sn, socket_mode, port, SOCK_IO_NONBLOCK);
        tcp_ret_debug = ret;

        if (ret != sn)
        {
            return ret;
        }

        tcp_socket_open_count++;
        break;

    default:
        /*
         * SOCK_LISTEN 등은 특별히 할 일이 없습니다.
         * 다음 호출에서 상태가 바뀌면 처리됩니다.
         */
        break;
    }

    return 1;
}
