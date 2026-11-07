#pragma once
#include <stdint.h>
#include <stddef.h>
typedef void* osMutexId_t;
typedef void* osMessageQueueId_t;
typedef enum { osOK, osError = -1 } osStatus_t;
typedef enum { osKernelInactive, osKernelRunning } osKernelState_t;
#define osWaitForever UINT32_MAX
osKernelState_t osKernelGetState(void);
osMutexId_t osMutexNew(const void*);
osStatus_t osMutexAcquire(osMutexId_t, uint32_t);
osStatus_t osMutexRelease(osMutexId_t);
osMessageQueueId_t osMessageQueueNew(uint32_t, uint32_t, const void*);
osStatus_t osMessageQueueDelete(osMessageQueueId_t);
osStatus_t osMessageQueuePut(osMessageQueueId_t, const void*, uint8_t, uint32_t);
osStatus_t osMessageQueueGet(osMessageQueueId_t, void*, uint8_t*, uint32_t);
uint32_t osMessageQueueGetCount(osMessageQueueId_t);
