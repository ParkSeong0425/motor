#include "storage.h"
#include "spi.h"
#include "gpio.h"
#include <string.h>

#define FR_ADDR     0
#define FR_MAGIC    0x4B465241u
#define FR_VER      5u
#define FR_VER_OLD  4u
#define FR_SIZE     8192u

#define C_WREN      0x06
#define C_WRDI      0x04
#define C_RDSR      0x05
#define C_READ      0x03
#define C_WRITE     0x02

typedef struct {
	uint32_t magic;
	uint16_t ver;
	uint16_t size;
	uint32_t dia_mm;
	uint32_t zero_ok;
	int32_t zero_unit;
	uint32_t in_ok;
	int32_t in_pos;
	uint32_t out_count;
	int32_t out_pos[ST_MAX];
	uint32_t sum;
} StRec_t;

typedef struct {
	uint32_t magic;
	uint16_t ver;
	uint16_t size;
	uint32_t dia_mm;
	uint32_t in_ok;
	int32_t in_pos;
	uint32_t out_count;
	int32_t out_pos[ST_MAX];
	uint32_t sum;
} StOldRec_t;

static StRec_t st;

/* FRAM CS low/high */
static void cs0(void) {
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_RESET);
}
static void cs1(void) {
	HAL_GPIO_WritePin(FRAM_CS_GPIO_Port, FRAM_CS_Pin, GPIO_PIN_SET);
}

/* checksum */
static uint32_t sum_bytes(const uint8_t *p, uint32_t n) {
	uint32_t s = 0;
	for (uint32_t i = 0; i < n; i++)
		s = (s * 33u) + p[i];
	return s;
}

static uint32_t sum(const StRec_t *r) {
	return sum_bytes((const uint8_t*) r, sizeof(StRec_t) - sizeof(uint32_t));
}

static uint32_t sum_old(const StOldRec_t *r) {
	return sum_bytes((const uint8_t*) r, sizeof(StOldRec_t) - sizeof(uint32_t));
}

static uint8_t good(const StRec_t *r) {
	if (r->magic != FR_MAGIC)
		return 0;
	if (r->ver != FR_VER)
		return 0;
	if (r->size != sizeof(StRec_t))
		return 0;
	if (r->dia_mm == 0)
		return 0;
	if (r->zero_ok > 1u || r->in_ok > 1u)
		return 0;
	if (r->out_count > ST_MAX)
		return 0;
	if (r->sum != sum(r))
		return 0;
	return 1;
}

static uint8_t good_old(const StOldRec_t *r) {
	if (r->magic != FR_MAGIC)
		return 0;
	if (r->ver != FR_VER_OLD)
		return 0;
	if (r->size != sizeof(StOldRec_t))
		return 0;
	if (r->dia_mm == 0)
		return 0;
	if (r->in_ok > 1u || r->out_count > ST_MAX)
		return 0;
	if (r->sum != sum_old(r))
		return 0;
	return 1;
}

/* FRAM command */
static HAL_StatusTypeDef cmd(uint8_t c) {
	HAL_StatusTypeDef r;
	cs0();
	r = HAL_SPI_Transmit(&hspi3, &c, 1, 100);
	cs1();
	return r;
}

/* FRAM status */
HAL_StatusTypeDef St_Stat(uint8_t *status) {
	HAL_StatusTypeDef r;
	uint8_t c = C_RDSR, v = 0;

	if (status == NULL)
		return HAL_ERROR;

	cs0();
	r = HAL_SPI_Transmit(&hspi3, &c, 1, 100);
	if (r == HAL_OK)
		r = HAL_SPI_Receive(&hspi3, &v, 1, 100);
	cs1();

	if (r == HAL_OK)
		*status = v;
	return r;
}

/* read/write */
static HAL_StatusTypeDef rd(uint16_t addr, uint8_t *data, uint16_t len) {
	HAL_StatusTypeDef r;
	uint8_t c[3];

	if (((uint32_t) addr + len) > FR_SIZE)
		return HAL_ERROR;

	c[0] = C_READ;
	c[1] = (uint8_t) (addr >> 8);
	c[2] = (uint8_t) addr;

	cs0();
	r = HAL_SPI_Transmit(&hspi3, c, sizeof(c), 100);
	if (r == HAL_OK)
		r = HAL_SPI_Receive(&hspi3, data, len, 100);
	cs1();

	return r;
}

static HAL_StatusTypeDef wr(uint16_t addr, const uint8_t *data, uint16_t len) {
	HAL_StatusTypeDef r;
	uint8_t c[3];

	if (((uint32_t) addr + len) > FR_SIZE)
		return HAL_ERROR;

	r = cmd(C_WREN);
	if (r != HAL_OK)
		return r;

	c[0] = C_WRITE;
	c[1] = (uint8_t) (addr >> 8);
	c[2] = (uint8_t) addr;

	cs0();
	r = HAL_SPI_Transmit(&hspi3, c, sizeof(c), 100);
	if (r == HAL_OK)
		r = HAL_SPI_Transmit(&hspi3, (uint8_t*) data, len, 100);
	cs1();

	(void) cmd(C_WRDI);
	return r;
}

/* default */
static void def(void) {
	memset(&st, 0, sizeof(st));
	st.magic = FR_MAGIC;
	st.ver = FR_VER;
	st.size = sizeof(StRec_t);
	st.dia_mm = ST_DIA_DEF;
	st.zero_ok = 0;
	st.zero_unit = 0;
	st.in_ok = 0;
	st.in_pos = 0;
	st.out_count = 0;
	st.sum = sum(&st);
}

/* v4 -> v5 migration */
static void migrate_old(const StOldRec_t *old) {
	def();
	st.dia_mm = old->dia_mm;
	st.in_ok = old->in_ok;
	st.in_pos = old->in_pos;
	st.out_count = old->out_count;
	if (st.out_count > ST_MAX)
		st.out_count = ST_MAX;

	for (uint32_t i = 0; i < ST_MAX; i++)
		st.out_pos[i] = old->out_pos[i];
	st.sum = sum(&st);
}

/* save/load */
HAL_StatusTypeDef St_Save(void) {
	HAL_StatusTypeDef r;
	StRec_t chk;

	st.magic = FR_MAGIC;
	st.ver = FR_VER;
	st.size = sizeof(StRec_t);
	if (st.dia_mm == 0)
		st.dia_mm = ST_DIA_DEF;
	if (st.zero_ok > 1u)
		st.zero_ok = 1u;
	if (st.in_ok > 1u)
		st.in_ok = 1u;
	if (st.out_count > ST_MAX)
		st.out_count = ST_MAX;
	st.sum = sum(&st);

	r = wr(FR_ADDR, (const uint8_t*) &st, sizeof(st));
	if (r != HAL_OK)
		return r;

	r = rd(FR_ADDR, (uint8_t*) &chk, sizeof(chk));
	if (r != HAL_OK)
		return r;
	if (good(&chk) == 0)
		return HAL_ERROR;
	if (memcmp(&chk, &st, sizeof(st)) != 0)
		return HAL_ERROR;

	return HAL_OK;
}

HAL_StatusTypeDef St_Load(void) {
	HAL_StatusTypeDef r;
	StOldRec_t old;

	r = rd(FR_ADDR, (uint8_t*) &st, sizeof(st));
	if (r == HAL_OK && good(&st) != 0)
		return HAL_OK;

	r = rd(FR_ADDR, (uint8_t*) &old, sizeof(old));
	if (r == HAL_OK && good_old(&old) != 0) {
		migrate_old(&old);
		return St_Save();
	}

	def();
	if (r != HAL_OK)
		return r;
	return St_Save();
}

HAL_StatusTypeDef St_Init(void) {
	cs1();
	return St_Load();
}

/* accessors */
uint32_t St_Dia(void) {
	return st.dia_mm;
}
void St_SetDia(uint32_t dia_mm) {
	if (dia_mm != 0)
		st.dia_mm = dia_mm;
}

uint8_t St_ZeroOk(void) {
	return (uint8_t) ((st.zero_ok != 0u) ? 1u : 0u);
}
int32_t St_Zero(void) {
	return st.zero_unit;
}
void St_SetZero(int32_t unit, uint8_t ok) {
	st.zero_unit = unit;
	st.zero_ok = (ok != 0u) ? 1u : 0u;
}

uint8_t St_InOk(void) {
	return (uint8_t) ((st.in_ok != 0u) ? 1u : 0u);
}
int32_t St_In(void) {
	return st.in_pos;
}
void St_SetIn(int32_t mm, uint8_t ok) {
	st.in_pos = mm;
	st.in_ok = (ok != 0u) ? 1u : 0u;
}

uint32_t St_Count(void) {
	return st.out_count;
}
int32_t St_Out(uint32_t index) {
	if (index >= st.out_count)
		return 0;
	return st.out_pos[index];
}

void St_SetOut(const int32_t *list, uint32_t count) {
	if (count > ST_MAX)
		count = ST_MAX;
	st.out_count = count;

	for (uint32_t i = 0; i < ST_MAX; i++) {
		st.out_pos[i] = (i < count && list != NULL) ? list[i] : 0;
	}
}
HAL_StatusTypeDef St_ClearAll(void) {
	/*
	 * FRAM 전체 설정을 기본값으로 초기화한다.
	 *
	 * 결과:
	 *   wheel     = ST_DIA_DEF
	 *   zero_ok   = 0
	 *   zero      = 0
	 *   in_ok     = 0
	 *   in        = 0
	 *   out_count = 0
	 *   out list  = 0
	 *
	 * 중요:
	 *   현재 모터 위치를 zero로 저장하지 않는다.
	 *   즉, St_SetZero(now, 1) 같은 동작은 하지 않는다.
	 */
	def();

	return St_Save();
}
