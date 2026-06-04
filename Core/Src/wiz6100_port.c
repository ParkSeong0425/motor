#include "main.h"
#include "spi.h"
#include "wizchip_conf.h"
#include "wiz6100_port.h"

volatile uint32_t spi_read_count = 0;
volatile uint32_t spi_write_count = 0;
volatile uint32_t spi_read_burst_count = 0;
volatile uint32_t spi_write_burst_count = 0;
static uint8_t spi_dummy_tx[2048];

extern SPI_HandleTypeDef hspi1;

static void wizchip_select(void)
{
    HAL_GPIO_WritePin(W610_CS_GPIO_Port, W610_CS_Pin, GPIO_PIN_RESET);
}

static void wizchip_deselect(void)
{
    HAL_GPIO_WritePin(W610_CS_GPIO_Port, W610_CS_Pin, GPIO_PIN_SET);
}

static uint8_t wizchip_read_byte(void)
{
    spi_read_count++;

    uint8_t tx = 0xFF;
    uint8_t rx = 0x00;

    HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 100);

    return rx;
}

static void wizchip_write_byte(uint8_t tx)
{
    spi_write_count++;

    HAL_SPI_Transmit(&hspi1, &tx, 1, 100);
}

static void wizchip_read_burst(uint8_t* buf, datasize_t len)
{
    spi_read_burst_count++;

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
        spi_dummy_tx[i] = 0xFF;
    }

    HAL_SPI_TransmitReceive(&hspi1, spi_dummy_tx, buf, len, 100);
}
static void wizchip_write_burst(uint8_t* buf, datasize_t len)
{
    spi_write_burst_count++;

    if (len <= 0)
    {
        return;
    }

    HAL_SPI_Transmit(&hspi1, buf, len, 100);
}
void W6100_Reset(void)
{
    HAL_GPIO_WritePin(W610_RST_GPIO_Port, W610_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);

    HAL_GPIO_WritePin(W610_RST_GPIO_Port, W610_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(100);
}

void W6100_RegisterCallback(void)
{
    reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);

    reg_wizchip_spi_cbfunc(
        wizchip_read_byte,
        wizchip_write_byte,
        wizchip_read_burst,
        wizchip_write_burst
    );
}





