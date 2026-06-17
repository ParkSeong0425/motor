#ifndef MOTION_PROTOCOL_H
#define MOTION_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>

void MotionProtocol_ProcessCommand(const char *cmd_in,
                                   char *response,
                                   size_t response_size);
void MotionProtocol_Command(const char *cmd_in,
                            char *response,
                            size_t response_size);
uint8_t MotionProtocol_Run(uint32_t num, uint32_t speed);
void MotionProtocol_StopLoop(void);
void MotionProtocol_Poll(void);
uint8_t MotionProtocol_TakeTcpLog(char *out, size_t size);

#ifdef __cplusplus
}
#endif

#endif
