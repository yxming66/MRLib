#pragma once
#include <stdint.h>
typedef enum {
  osKernelInactive,
  osKernelReady,
  osKernelRunning,
  osKernelLocked,
  osKernelSuspended,
  osKernelError = -1,
  osKernelReserved = 0x7fffffff
} osKernelState_t;
typedef enum { osOK, osError = -1 } osStatus_t;
osKernelState_t osKernelGetState(void);
uint32_t osKernelGetTickFreq(void);
osStatus_t osDelay(uint32_t ticks);
