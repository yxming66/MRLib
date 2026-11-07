#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "bsp/bsp.h"
#ifdef __cplusplus
extern "C" {
#endif

// Required project implementations; linked directly, no runtime registry/heap.
// A single monotonic microsecond epoch shared by tasks and supported ISRs.
// Board documents counter resolution, IRQ latency, sleep and clock restrictions.
bool Board_TimeIsReady(void);
uint64_t Board_TimeNowUs(void);
// Pre-scheduler millisecond delay, returning BSP_OK/BSP_ERR (no RTOS call).
int8_t Board_TimeDelayMs(uint32_t ms);
#ifdef __cplusplus
}
#endif
