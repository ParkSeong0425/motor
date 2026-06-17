#include "net.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "cli.h"
#include "storage.h"
#include "motion_protocol.h"

#include "wizchip_conf.h"
#include "wiz6100_port.h"
#include "socket.h"

#include <stdio.h>
#include <string.h>

/* W6100 socket 번호 */
static uint8_t sock = 0;

/* PC TCP 포트 */
static uint16_t port = 5000;

/* TCP RX 버퍼 */
static uint8_t buf[2048];

/* TCP 로그/응답 버퍼 */
static char log_buf[160];
static char rx_cmd[192];
static char tx_res[192];
static uint16_t rx_len = 0;
static uint32_t rx_tick = 0;

/* W6100 네트워크 정보 */
static wiz_NetInfo info = {
    .mac = {0, 0, 0, 0, 0, 0},
    .ip = {172, 20, 0, 192},
    .sn = {255, 255, 0, 0},
    .gw = {0, 0, 0, 0},
    .dns = {0, 0, 0, 0},
    .ipmode = NETINFO_STATIC_V4
};

/* MAC 주소가 00:00... 또는 FF:FF... 인지 확인 */
static uint8_t mac_ok(void)
{
    uint8_t all_00 = 1;
    uint8_t all_ff = 1;

    for (uint8_t i = 0; i < 6; i++)
    {
        if (info.mac[i] != 0x00) all_00 = 0;
        if (info.mac[i] != 0xFF) all_ff = 0;
    }

    return (uint8_t)((all_00 == 0 && all_ff == 0) ? 1u : 0u);
}

/* MAC EEPROM에서 MAC 주소 6byte 읽기 */
static HAL_StatusTypeDef mac(void)
{
    return HAL_I2C_Mem_Read(&hi2c1,
                            MAC_ADDR,
                            MAC_POS,
                            I2C_MEMADD_SIZE_8BIT,
                            info.mac,
                            MAC_LEN,
                            100);
}

/* W6100 MAC, IP, Subnet 설정 */
static void set(void)
{
    uint8_t lock = SYS_NET_LOCK;

    (void)ctlwizchip(CW_SYS_UNLOCK, &lock);
    (void)ctlnetwork(CN_SET_NETINFO, &info);
}

/* TCP socket open */
static void open(void)
{
    if (socket(sock, Sn_MR_TCP4, port, 0) != sock)
    {
        close(sock);
    }
}

/* 문자열을 TCP로 전송 */
static void tx(const char *s)
{
    int32_t ret;
    uint16_t len;

    if (s == NULL) return;

    len = (uint16_t)strlen(s);
    if (len == 0) return;

    ret = send(sock, (uint8_t *)s, len);
    if (ret <= 0) close(sock);
}

/* 수신된 명령 1개 실행 */
static void run_cmd(void)
{
    if (rx_len == 0) return;

    rx_cmd[rx_len] = '\0';
    MotionProtocol_Command(rx_cmd, tx_res, sizeof(tx_res));
    tx(tx_res);
    rx_len = 0;
}

/* TCP 수신 데이터를 MotionProtocol 명령으로 처리 */
static void rxcmd(void)
{
    uint16_t size;
    int32_t ret;

    size = getSn_RX_RSR(sock);

    while (size > 0)
    {
        uint16_t n = size;

        if (n > sizeof(buf)) n = sizeof(buf);

        ret = recv(sock, buf, n);
        if (ret <= 0)
        {
            close(sock);
            return;
        }

        for (int32_t i = 0; i < ret; i++)
        {
            uint8_t c = buf[i];

            if (c == '\r' || c == '\n')
            {
                run_cmd();
            }
            else if (rx_len < (sizeof(rx_cmd) - 1u))
            {
                rx_cmd[rx_len++] = (char)c;
                rx_tick = osKernelGetTickCount();
            }
            else
            {
                rx_len = 0;
                tx("01E_LONG\r\n");
            }
        }

        size = getSn_RX_RSR(sock);
    }
}

/* 개행 없이 들어온 명령도 잠깐 대기 후 실행 */
static void rxidle(void)
{
    if (rx_len == 0) return;

    if ((int32_t)(osKernelGetTickCount() - rx_tick) >= 20)
    {
        run_cmd();
    }
}

/* MotionProtocol TCP 로그를 PC로 전송 */
static void txlog(void)
{
    if (MotionProtocol_TakeTcpLog(log_buf, sizeof(log_buf)) == 0) return;
    tx(log_buf);
}

/* TCP socket 상태별 open, listen, receive, send 처리 */
static void tcp(void)
{
    uint8_t state;

    state = getSn_SR(sock);

    if (state == SOCK_CLOSED)
    {
        rx_len = 0;
        open();
        return;
    }

    if (state == SOCK_INIT)
    {
        (void)listen(sock);
        return;
    }

    if (state == SOCK_ESTABLISHED)
    {
        rxcmd();
        rxidle();
        txlog();
        return;
    }

    if (state == SOCK_CLOSE_WAIT)
    {
        rx_len = 0;
        close(sock);
        return;
    }
}

/* W6100 초기화 후 TCP 명령 처리 */
void Net_TaskRun(void *argument)
{
    uint8_t mem[16] = {
        2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2
    };
    uint8_t status = 0;

    (void)argument;

    /*
     * CLI 메뉴 출력 후 NET 로그를 출력하기 위해 대기.
     * 순서: CLI 메뉴 -> > 프롬프트 -> NET 로그
     */
    while (CLI_IsReady() == 0)
    {
        osDelay(1);
    }

    printf("[NET] start\r\n");

    if (St_Stat(&status) == HAL_OK)
    {
        printf("[NET] FRAM status=0x%02X\r\n", status);
    }

    if (mac() != HAL_OK || mac_ok() == 0)
    {
        printf("[NET] MAC fail\r\n");

        for (;;)
        {
            osDelay(1000);
        }
    }

    printf("[NET] MAC %02X:%02X:%02X:%02X:%02X:%02X\r\n",
           info.mac[0],
           info.mac[1],
           info.mac[2],
           info.mac[3],
           info.mac[4],
           info.mac[5]);

    W6100_RegisterCallback();
    W6100_Reset();

    if (ctlwizchip(CW_INIT_WIZCHIP, mem) != 0)
    {
        printf("[NET] W6100 init fail\r\n");
    }

    set();

    printf("[NET] IP %u.%u.%u.%u PORT %u\r\n",
           info.ip[0],
           info.ip[1],
           info.ip[2],
           info.ip[3],
           (unsigned int)port);

    for (;;)
    {
        tcp();
        osDelay(1);
    }
}
