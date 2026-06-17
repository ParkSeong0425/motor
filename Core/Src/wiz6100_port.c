#include "main.h"
#include "spi.h"
#include "wizchip_conf.h"
#include "wiz6100_port.h"

extern SPI_HandleTypeDef hspi1;

/*
 * W6100 burst read 때 보낼 dummy 데이터.
 * SPI read는 실제로는 transmit + receive가 같이 일어나기 때문에 필요하다.
 */
static uint8_t dummy_tx[2048];

/* W6100 SPI 선택, CS LOW */
static void sel(void)
{
    HAL_GPIO_WritePin(W610_CS_GPIO_Port, W610_CS_Pin, GPIO_PIN_RESET);
}

/* W6100 SPI 해제, CS HIGH */
static void desel(void)
{
    HAL_GPIO_WritePin(W610_CS_GPIO_Port, W610_CS_Pin, GPIO_PIN_SET);
}

/* W6100에서 1바이트 읽기 */
static uint8_t rd(void)
{
    uint8_t tx = 0xFF;
    uint8_t rx = 0x00;

    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 100);

    return rx;
}

/* W6100에 1바이트 쓰기 */
static void wr(uint8_t tx)
{
    HAL_SPI_Transmit(&hspi1, &tx, 1, 100);
}

/* W6100에서 여러 바이트 읽기 */
static void rds(uint8_t *buf, datasize_t len)
{
    if (len <= 0)
    {
        return;
    }

    if (len > 2048)
    {
        len = 2048;
    }

    for (datasize_t i = 0; i < len; i++)
    {
        dummy_tx[i] = 0xFF;
    }

    HAL_SPI_TransmitReceive(&hspi1, dummy_tx, buf, len, 100);
}

/* W6100에 여러 바이트 쓰기 */
static void wrs(uint8_t *buf, datasize_t len)
{
    if (len <= 0)
    {
        return;
    }

    HAL_SPI_Transmit(&hspi1, buf, len, 100);
}

/* W6100 하드웨어 리셋 */
void W6100_Reset(void)
{
    HAL_GPIO_WritePin(W610_RST_GPIO_Port, W610_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    HAL_GPIO_WritePin(W610_RST_GPIO_Port, W610_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
}

/* WIZnet 드라이버에 SPI/CS 함수 연결 */
void W6100_RegisterCallback(void)
{
    reg_wizchip_cs_cbfunc(sel, desel);
    reg_wizchip_spi_cbfunc(rd, wr, rds, wrs);
}
