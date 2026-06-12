#ifndef CLI_H
#define CLI_H

#ifdef __cplusplus
extern "C" {
#endif

void CLI_Init(void);
void CLI_Poll(void);
void CLI_TaskRun(void *argument);

#ifdef __cplusplus
}
#endif

#endif
