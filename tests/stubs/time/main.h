#pragma once
#include <stdint.h>
uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t ms);
extern uint32_t time_primask;
static inline uint32_t __get_PRIMASK(void) { return time_primask; }
static inline void __disable_irq(void) { time_primask = 1; }
static inline void __enable_irq(void) { time_primask = 0; }
