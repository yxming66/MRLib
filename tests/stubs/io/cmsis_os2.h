#pragma once
#include <stdint.h>
typedef void* osThreadId_t;
#define osFlagsWaitAll 1u
#define osFlagsWaitAny 0u
#define osFlagsErrorResource 0xfffffffdU
#define osFlagsErrorTimeout 0xfffffffeU
osThreadId_t osThreadGetId(void);
uint32_t osThreadFlagsSet(osThreadId_t, uint32_t);
uint32_t osThreadFlagsClear(uint32_t);
uint32_t osThreadFlagsWait(uint32_t, uint32_t, uint32_t);
