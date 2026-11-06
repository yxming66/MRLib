#pragma once

#include <stdint.h>

typedef void* osMessageQueueId_t;
typedef int32_t osStatus_t;

enum { osOK = 0 };

#ifdef __cplusplus
extern "C" {
#endif

osStatus_t osMessageQueueGet(osMessageQueueId_t queue, void* message,
                             uint8_t* priority, uint32_t timeout);

#ifdef __cplusplus
}
#endif
