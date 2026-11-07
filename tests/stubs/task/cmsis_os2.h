#pragma once
#include <stdint.h>
typedef void* osThreadId_t;
typedef void (*osThreadFunc_t)(void*);
typedef enum { osOK = 0, osError = -1 } osStatus_t;
typedef enum { osKernelInactive, osKernelReady, osKernelRunning } osKernelState_t;
typedef enum { osPriorityLow = 8, osPriorityNormal = 24, osPriorityHigh = 40 } osPriority_t;
typedef struct {
  const char* name;
  uint32_t attr_bits;
  void* cb_mem;
  uint32_t cb_size;
  void* stack_mem;
  uint32_t stack_size;
  osPriority_t priority;
} osThreadAttr_t;
osKernelState_t osKernelGetState(void);
uint32_t osKernelGetTickFreq(void);
uint32_t osKernelGetTickCount(void);
osThreadId_t osThreadNew(osThreadFunc_t, void*, const osThreadAttr_t*);
osStatus_t osDelay(uint32_t);
osStatus_t osDelayUntil(uint32_t);
void osThreadExit(void);
