#ifndef TCP_CMD_SERVER_H
#define TCP_CMD_SERVER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*
 * TCP server IP mode
 *
 * 주의:
 * 아래 값은 실제 IP 주소가 아니다.
 *
 * 실제 IP 주소:
 *   main.c의 netinfo_set.ip = {172, 20, 0, 192}
 *
 * 아래 값:
 *   W6100 socket을 IPv4 / IPv6 / Dual 중 어떤 TCP mode로 열지 정하는 값
 */
#define TCP_CMD_MODE_IPV4      0U
#define TCP_CMD_MODE_IPV6      1U
#define TCP_CMD_MODE_DUAL      2U

/**
 * @brief TCP 명령 서버 처리 함수
 *
 * 이 함수는 한 번 호출한다고 끝나는 함수가 아닙니다.
 * FreeRTOS에서는 TcpTask 안에서 계속 반복 호출해야 합니다.
 *
 * 역할:
 * ------------------------------------------------------------
 * 1. socket이 닫혀 있으면 socket을 생성
 * 2. socket이 초기화 상태이면 listen 수행
 * 3. PC가 접속하면 수신 데이터 확인
 * 4. 수신 데이터가 있으면 명령 처리 후 응답 전송
 * 5. 연결 종료 요청이 있으면 disconnect 수행
 *
 * @param sn       W6100 socket 번호. 보통 0 사용.
 * @param buf      TCP 수신 데이터를 저장할 버퍼.
 * @param port     TCP 서버 포트 번호. 현재 5000 사용.
 * @param ip_mode  TCP_CMD_MODE_IPV4 / TCP_CMD_MODE_IPV6 / TCP_CMD_MODE_DUAL
 *
 * @return 1 이상이면 기본적으로 정상 처리.
 *         음수이면 WIZnet socket/send/recv 계열 에러 가능.
 */
int32_t TcpCmdServer_Process(
    uint8_t sn,
    uint8_t *buf,
    uint16_t port,
    uint8_t ip_mode
);

#ifdef __cplusplus
}
#endif

#endif /* TCP_CMD_SERVER_H */
