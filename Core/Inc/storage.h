#ifndef STORAGE_H
#define STORAGE_H

#include "main.h"
#include <stdint.h>

/* FRAM에 저장할 출고 위치 최대 개수 */
#define ST_MAX      16

/* 기본 바퀴 지름(mm). wheel 명령으로 변경 가능 */
#define ST_DIA_DEF  100

HAL_StatusTypeDef St_Init(void);
HAL_StatusTypeDef St_Load(void);
HAL_StatusTypeDef St_Save(void);
HAL_StatusTypeDef St_Stat(uint8_t *status);

uint32_t St_Dia(void);
void St_SetDia(uint32_t dia_mm);

uint8_t St_InOk(void);
int32_t St_In(void);
void St_SetIn(int32_t mm, uint8_t ok);

uint32_t St_Count(void);
int32_t St_Out(uint32_t index);
void St_SetOut(const int32_t *list, uint32_t count);

#endif
