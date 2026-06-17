#include "storage.h"
#include "spi.h"
#include "gpio.h"
#include <string.h>

#define FR_ADDR     0
#define FR_MAGIC    0x4B465241u
#define FR_VER      4u
#define FR_VER_OLD  3u
#define FR_SIZE     8192u

#define C_WREN      0x06
#define C_WRDI      0x04
#define C_RDSR      0x05
#define C_READ      0x03
#define C_WRITE     0x02

typedef struct
{
    uint32_t magic;
    uint16_t ver;
    uint16_t size;
    uint32_t dia_mm;
    uint32_t in_ok;
    int32_t in_pos;
    uint32_t out_count;
    int32_t out_pos[ST_MAX];
    uint32_t sum;
} StRec_t;

typedef struct
{
    uint32_t magic;
    uint16_t ver;
    uint16_t size;
    uint32_t dia_mm;
    uint32_t out_count;
    int32_t out_pos[ST_MAX];
    uint32_t sum;
} StOldRec_t;

static StRec_t st;

/* FRAM CS low */
static void cs0(void)
{
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);
}

/* FRAM CS high */
static void cs1(void)
{
    HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
}

/* 저장 데이터 확인용 checksum */
static uint32_t sum_bytes(const uint8_t *p, uint32_t n)
{
    uint32_t s = 0;

    for (uint32_t i = 0; i < n; i++)
    {
        s = (s * 33u) + p[i];
    }

    return s;
}

/* 현재 레코드 checksum */
static uint32_t sum(const StRec_t *r)
{
    return sum_bytes((const uint8_t *)r, sizeof(StRec_t) - sizeof(uint32_t));
}

/* 구버전(v3) 레코드 checksum */
static uint32_t sum_old(const StOldRec_t *r)
{
    return sum_bytes((const uint8_t *)r, sizeof(StOldRec_t) - sizeof(uint32_t));
}

/* 현재 FRAM 레코드가 정상인지 확인 */
static uint8_t good(const StRec_t *r)
{
    if (r->magic != FR_MAGIC) return 0;
    if (r->ver != FR_VER) return 0;
    if (r->size != sizeof(StRec_t)) return 0;
    if (r->dia_mm == 0) return 0;
    if (r->out_count > ST_MAX) return 0;
    if (r->in_ok > 1u) return 0;
    if (r->sum != sum(r)) return 0;
    return 1;
}

/* 구버전(v3) FRAM 레코드가 정상인지 확인 */
static uint8_t good_old(const StOldRec_t *r)
{
    if (r->magic != FR_MAGIC) return 0;
    if (r->ver != FR_VER_OLD) return 0;
    if (r->size != sizeof(StOldRec_t)) return 0;
    if (r->dia_mm == 0) return 0;
    if (r->out_count > ST_MAX) return 0;
    if (r->sum != sum_old(r)) return 0;
    return 1;
}

/* FRAM 명령 1byte 전송 */
static HAL_StatusTypeDef cmd(uint8_t c)
{
    HAL_StatusTypeDef r;

    cs0();
    r = HAL_SPI_Transmit(&hspi3, &c, 1, 100);
    cs1();

    return r;
}

/* FRAM 상태 레지스터 읽기 */
HAL_StatusTypeDef St_Stat(uint8_t *status)
{
    HAL_StatusTypeDef r;
    uint8_t c = C_RDSR;
    uint8_t v = 0;

    if (status == NULL) return HAL_ERROR;

    cs0();
    r = HAL_SPI_Transmit(&hspi3, &c, 1, 100);
    if (r == HAL_OK) r = HAL_SPI_Receive(&hspi3, &v, 1, 100);
    cs1();

    if (r == HAL_OK) *status = v;
    return r;
}

/* FRAM에서 len byte 읽기 */
static HAL_StatusTypeDef rd(uint16_t addr, uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef r;
    uint8_t c[3];

    if (((uint32_t)addr + len) > FR_SIZE) return HAL_ERROR;

    c[0] = C_READ;
    c[1] = (uint8_t)(addr >> 8);
    c[2] = (uint8_t)addr;

    cs0();
    r = HAL_SPI_Transmit(&hspi3, c, sizeof(c), 100);
    if (r == HAL_OK) r = HAL_SPI_Receive(&hspi3, data, len, 100);
    cs1();

    return r;
}

/* FRAM에 len byte 쓰기 */
static HAL_StatusTypeDef wr(uint16_t addr, const uint8_t *data, uint16_t len)
{
    HAL_StatusTypeDef r;
    uint8_t c[3];

    if (((uint32_t)addr + len) > FR_SIZE) return HAL_ERROR;

    r = cmd(C_WREN);
    if (r != HAL_OK) return r;

    c[0] = C_WRITE;
    c[1] = (uint8_t)(addr >> 8);
    c[2] = (uint8_t)addr;

    cs0();
    r = HAL_SPI_Transmit(&hspi3, c, sizeof(c), 100);
    if (r == HAL_OK) r = HAL_SPI_Transmit(&hspi3, (uint8_t *)data, len, 100);
    cs1();

    (void)cmd(C_WRDI);
    return r;
}

/* 기본값 설정 */
static void def(void)
{
    memset(&st, 0, sizeof(st));
    st.magic = FR_MAGIC;
    st.ver = FR_VER;
    st.size = sizeof(StRec_t);
    st.dia_mm = ST_DIA_DEF;
    st.in_ok = 0;
    st.in_pos = 0;
    st.out_count = 0;
    st.sum = sum(&st);
}

/* 구버전(v3) 저장값을 현재(v4) 구조로 변환 */
static void migrate_old(const StOldRec_t *old)
{
    def();
    st.dia_mm = old->dia_mm;
    st.out_count = old->out_count;

    if (st.out_count > ST_MAX) st.out_count = ST_MAX;

    for (uint32_t i = 0; i < ST_MAX; i++)
    {
        st.out_pos[i] = old->out_pos[i];
    }

    st.sum = sum(&st);
}

/* RAM 값을 FRAM에 저장 후 다시 읽어 검증 */
HAL_StatusTypeDef St_Save(void)
{
    HAL_StatusTypeDef r;
    StRec_t chk;

    st.magic = FR_MAGIC;
    st.ver = FR_VER;
    st.size = sizeof(StRec_t);
    if (st.dia_mm == 0) st.dia_mm = ST_DIA_DEF;
    if (st.in_ok > 1u) st.in_ok = 1u;
    if (st.out_count > ST_MAX) st.out_count = ST_MAX;
    st.sum = sum(&st);

    r = wr(FR_ADDR, (const uint8_t *)&st, sizeof(st));
    if (r != HAL_OK) return r;

    r = rd(FR_ADDR, (uint8_t *)&chk, sizeof(chk));
    if (r != HAL_OK) return r;
    if (good(&chk) == 0) return HAL_ERROR;
    if (memcmp(&chk, &st, sizeof(st)) != 0) return HAL_ERROR;

    return HAL_OK;
}

/* FRAM에서 저장값 읽기 */
HAL_StatusTypeDef St_Load(void)
{
    HAL_StatusTypeDef r;
    StOldRec_t old;

    r = rd(FR_ADDR, (uint8_t *)&st, sizeof(st));
    if (r == HAL_OK && good(&st) != 0)
    {
        return HAL_OK;
    }

    r = rd(FR_ADDR, (uint8_t *)&old, sizeof(old));
    if (r == HAL_OK && good_old(&old) != 0)
    {
        migrate_old(&old);
        return St_Save();
    }

    def();
    if (r != HAL_OK) return r;
    return St_Save();
}

/* FRAM 초기화 후 저장값 로드 */
HAL_StatusTypeDef St_Init(void)
{
    cs1();
    return St_Load();
}

/* 바퀴 지름(mm) 읽기 */
uint32_t St_Dia(void)
{
    return st.dia_mm;
}

/* 바퀴 지름(mm) 설정 */
void St_SetDia(uint32_t dia_mm)
{
    if (dia_mm != 0) st.dia_mm = dia_mm;
}

/* 저장된 입고 위치가 있는지 확인 */
uint8_t St_InOk(void)
{
    return (uint8_t)((st.in_ok != 0u) ? 1u : 0u);
}

/* 입고 거리(mm) 읽기 */
int32_t St_In(void)
{
    return st.in_pos;
}

/* 입고 위치(mm) 설정 */
void St_SetIn(int32_t mm, uint8_t ok)
{
    st.in_pos = mm;
    st.in_ok = (ok != 0u) ? 1u : 0u;
}

/* 저장된 출고 위치 개수 */
uint32_t St_Count(void)
{
    return st.out_count;
}

/* index 위치의 출고 거리(mm) */
int32_t St_Out(uint32_t index)
{
    if (index >= st.out_count) return 0;
    return st.out_pos[index];
}

/* 출고 위치 목록(mm) 설정 */
void St_SetOut(const int32_t *list, uint32_t count)
{
    if (count > ST_MAX) count = ST_MAX;
    st.out_count = count;

    for (uint32_t i = 0; i < ST_MAX; i++)
    {
        st.out_pos[i] = (i < count && list != NULL) ? list[i] : 0;
    }
}
