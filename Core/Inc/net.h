#ifndef NET_H
#define NET_H

#include <stdint.h>

/* MAC EEPROM의 I2C 주소, HAL 함수용이라 0x50을 왼쪽으로 1칸 민 값 */
#define MAC_ADDR    (0x50 << 1)

/* EEPROM 안에서 MAC 주소가 시작되는 위치 */
#define MAC_POS     0xFA

/* MAC 주소 길이, MAC은 항상 6바이트 */
#define MAC_LEN     6

/*
 * W6100 / TCP 네트워크 태스크.
 * FreeRTOS NetTask에서 이 함수 하나만 실행하면 된다.
 */
void Net_TaskRun(void *argument);

#endif
