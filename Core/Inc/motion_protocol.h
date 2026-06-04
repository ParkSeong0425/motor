#ifndef MOTION_PROTOCOL_H
#define MOTION_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

/*
 * TCP 명령 처리 진입 함수.
 */
void MotionProtocol_ProcessCommand(
    const char *cmd_in,
    char *response,
    size_t response_size
);

#ifdef __cplusplus
}
#endif

#endif /* MOTION_PROTOCOL_H */
