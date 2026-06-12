#include "net.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "spi.h"
#include "gpio.h"

#include "wizchip_conf.h"
#include "wiz6100_port.h"
#include "tcp_cmd_server.h"

#include <stdio.h>

#define TCP_SOCKET   0
#define TCP_PORT     5000

static uint8_t tcp_buf[2048];

static wiz_NetInfo netinfo = {
    .mac = {0, 0, 0, 0, 0, 0},
    .ip = {172, 20, 0, 192},
    .sn = {255, 255, 0, 0},
    .gw = {0, 0, 0, 0},
    .dns = {0, 0, 0, 0},
    .ipmode = NETINFO_STATIC_V4
};

HAL_StatusTypeDef MacEeprom_IsReady(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_IsDeviceReady(hi2c,
                                 MAC_EEPROM_I2C_ADDR_HAL,
                                 3,
                                 100);
}

HAL_StatusTypeDef MacEeprom_Read(I2C_HandleTypeDef *hi2c,
                                 uint8_t addr,
                                 uint8_t *data,
                                 uint16_t len)
{
    if (hi2c == NULL || data == NULL || len == 0)
    {
        return HAL_ERROR;
    }

    if (((uint16_t)addr + len) > MAC_EEPROM_SIZE_BYTES)
    {
        return HAL_ERROR;
    }

    return HAL_I2C_Mem_Read(hi2c,
                            MAC_EEPROM_I2C_ADDR_HAL,
                            addr,
                            I2C_MEMADD_SIZE_8BIT,
                            data,
                            len,
                            100);
}

HAL_StatusTypeDef MacEeprom_ReadByte(I2C_HandleTypeDef *hi2c,
                                     uint8_t addr,
                                     uint8_t *value)
{
    return MacEeprom_Read(hi2c, addr, value, 1);
}

HAL_StatusTypeDef MacEeprom_ReadMac(I2C_HandleTypeDef *hi2c,
                                    uint8_t mac[MAC_EEPROM_EUI48_LEN])
{
    return MacEeprom_Read(hi2c,
                          MAC_EEPROM_EUI48_OFFSET,
                          mac,
                          MAC_EEPROM_EUI48_LEN);
}

uint8_t MacEeprom_IsMacPlausible(const uint8_t mac[MAC_EEPROM_EUI48_LEN])
{
    uint8_t all_00 = 1;
    uint8_t all_ff = 1;

    if (mac == NULL)
    {
        return 0;
    }

    for (uint8_t i = 0; i < MAC_EEPROM_EUI48_LEN; i++)
    {
        if (mac[i] != 0x00)
        {
            all_00 = 0;
        }

        if (mac[i] != 0xFF)
        {
            all_ff = 0;
        }
    }

    if (all_00 != 0 || all_ff != 0)
    {
        return 0;
    }

    return 1;
}

#define FRAM_CMD_WREN   0x06
#define FRAM_CMD_WRDI   0x04
#define FRAM_CMD_RDSR   0x05
#define FRAM_CMD_READ   0x03
#define FRAM_CMD_WRITE  0x02

static void Fram_CsLow(void)
{
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);
}

static void Fram_CsHigh(void)
{
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
}

HAL_StatusTypeDef Fram_InitPins(void)
{
    Fram_CsHigh();
    return HAL_OK;
}

static HAL_StatusTypeDef Fram_Cmd(SPI_HandleTypeDef *hspi, uint8_t cmd)
{
    HAL_StatusTypeDef result;

    if (hspi == NULL)
    {
        return HAL_ERROR;
    }

    Fram_CsLow();
    result = HAL_SPI_Transmit(hspi, &cmd, 1, 100);
    Fram_CsHigh();

    return result;
}

HAL_StatusTypeDef Fram_WriteEnable(SPI_HandleTypeDef *hspi)
{
    return Fram_Cmd(hspi, FRAM_CMD_WREN);
}

HAL_StatusTypeDef Fram_WriteDisable(SPI_HandleTypeDef *hspi)
{
    return Fram_Cmd(hspi, FRAM_CMD_WRDI);
}

HAL_StatusTypeDef Fram_ReadStatus(SPI_HandleTypeDef *hspi, uint8_t *status)
{
    HAL_StatusTypeDef result;
    uint8_t cmd = FRAM_CMD_RDSR;
    uint8_t rx = 0;

    if (hspi == NULL || status == NULL)
    {
        return HAL_ERROR;
    }

    Fram_CsLow();

    result = HAL_SPI_Transmit(hspi, &cmd, 1, 100);

    if (result == HAL_OK)
    {
        result = HAL_SPI_Receive(hspi, &rx, 1, 100);
    }

    Fram_CsHigh();

    if (result == HAL_OK)
    {
        *status = rx;
    }

    return result;
}

HAL_StatusTypeDef Fram_CheckWriteEnableLatch(SPI_HandleTypeDef *hspi,
                                             uint8_t *before,
                                             uint8_t *after_wren,
                                             uint8_t *after_wrdi)
{
    HAL_StatusTypeDef result;

    if (hspi == NULL || before == NULL ||
        after_wren == NULL || after_wrdi == NULL)
    {
        return HAL_ERROR;
    }

    result = Fram_ReadStatus(hspi, before);

    if (result == HAL_OK)
    {
        result = Fram_WriteEnable(hspi);
    }

    if (result == HAL_OK)
    {
        result = Fram_ReadStatus(hspi, after_wren);
    }

    if (result == HAL_OK)
    {
        result = Fram_WriteDisable(hspi);
    }

    if (result == HAL_OK)
    {
        result = Fram_ReadStatus(hspi, after_wrdi);
    }

    if (result != HAL_OK)
    {
        return result;
    }

    if (((*after_wren & FRAM_STATUS_WEL) == 0) ||
        ((*after_wrdi & FRAM_STATUS_WEL) != 0))
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

HAL_StatusTypeDef Fram_Read(SPI_HandleTypeDef *hspi,
                            uint16_t addr,
                            uint8_t *data,
                            uint16_t len)
{
    HAL_StatusTypeDef result;
    uint8_t cmd[3];

    if (hspi == NULL || data == NULL || len == 0)
    {
        return HAL_ERROR;
    }

    if (((uint32_t)addr + len) > FRAM_MB85RS64_SIZE_BYTES)
    {
        return HAL_ERROR;
    }

    cmd[0] = FRAM_CMD_READ;
    cmd[1] = (uint8_t)(addr >> 8);
    cmd[2] = (uint8_t)addr;

    Fram_CsLow();

    result = HAL_SPI_Transmit(hspi, cmd, sizeof(cmd), 100);

    if (result == HAL_OK)
    {
        result = HAL_SPI_Receive(hspi, data, len, 100);
    }

    Fram_CsHigh();

    return result;
}

HAL_StatusTypeDef Fram_Write(SPI_HandleTypeDef *hspi,
                             uint16_t addr,
                             const uint8_t *data,
                             uint16_t len)
{
    HAL_StatusTypeDef result;
    uint8_t cmd[3];

    if (hspi == NULL || data == NULL || len == 0)
    {
        return HAL_ERROR;
    }

    if (((uint32_t)addr + len) > FRAM_MB85RS64_SIZE_BYTES)
    {
        return HAL_ERROR;
    }

    result = Fram_WriteEnable(hspi);

    if (result != HAL_OK)
    {
        return result;
    }

    cmd[0] = FRAM_CMD_WRITE;
    cmd[1] = (uint8_t)(addr >> 8);
    cmd[2] = (uint8_t)addr;

    Fram_CsLow();

    result = HAL_SPI_Transmit(hspi, cmd, sizeof(cmd), 100);

    if (result == HAL_OK)
    {
        result = HAL_SPI_Transmit(hspi, (uint8_t *)data, len, 100);
    }

    Fram_CsHigh();

    (void)Fram_WriteDisable(hspi);

    return result;
}

static void Net_SetInfo(void)
{
    uint8_t lock = SYS_NET_LOCK;

    (void)ctlwizchip(CW_SYS_UNLOCK, &lock);
    (void)ctlnetwork(CN_SET_NETINFO, &netinfo);
}

void Net_TaskRun(void *argument)
{
    uint8_t mem[16] = {
        2, 2, 2, 2, 2, 2, 2, 2,
        2, 2, 2, 2, 2, 2, 2, 2
    };
    uint8_t status = 0;

    (void)argument;

    osDelay(500);

    printf("[NET] start\r\n");

    (void)Fram_InitPins();

    if (Fram_ReadStatus(&hspi3, &status) == HAL_OK)
    {
        printf("[NET] FRAM status=0x%02X\r\n", status);
    }

    if (MacEeprom_ReadMac(&hi2c1, netinfo.mac) != HAL_OK ||
        MacEeprom_IsMacPlausible(netinfo.mac) == 0)
    {
        printf("[NET] MAC fail\r\n");

        for (;;)
        {
            osDelay(1000);
        }
    }

    printf("[NET] MAC %02X:%02X:%02X:%02X:%02X:%02X\r\n",
           netinfo.mac[0],
           netinfo.mac[1],
           netinfo.mac[2],
           netinfo.mac[3],
           netinfo.mac[4],
           netinfo.mac[5]);

    W6100_RegisterCallback();
    W6100_Reset();

    if (ctlwizchip(CW_INIT_WIZCHIP, mem) != 0)
    {
        printf("[NET] W6100 init fail\r\n");
    }

    Net_SetInfo();

    printf("[NET] IP %u.%u.%u.%u PORT %u\r\n",
           netinfo.ip[0],
           netinfo.ip[1],
           netinfo.ip[2],
           netinfo.ip[3],
           TCP_PORT);

    for (;;)
    {
        (void)TcpCmdServer_Process(TCP_SOCKET,
                                   tcp_buf,
                                   TCP_PORT,
                                   TCP_CMD_MODE_IPV4);

        osDelay(1);
    }
}
