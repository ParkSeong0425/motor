#ifndef CLI_H
#define CLI_H

#include <stdint.h>

void CLI_Init(void);
void CLI_Poll(void);
void CLI_TaskRun(void *argument);

void CLI_SetReady(void);
uint8_t CLI_IsReady(void);

#endif
