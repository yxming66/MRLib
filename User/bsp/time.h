#pragma once
#include <stdint.h>
#include "bsp/bsp.h"
#ifdef __cplusplus
extern "C" {
#endif

// Board must initialize its time source before use; unavailable reads return 0.
// Both units use the SAME epoch. Milliseconds wrap modulo 2^32 (~49.7 days);
// use unsigned subtraction for bounded elapsed times. Microseconds retain 64 bits.
uint32_t BSP_TIME_Get_ms(void);
uint64_t BSP_TIME_Get_us(void);
uint64_t BSP_TIME_Get(void);
int8_t BSP_TIME_Delay_ms(uint32_t ms);
// Short hardware timing only, outside ISR/IRQ-masked sections. No task waiting.
int8_t BSP_TIME_Delay_us(uint32_t us);
int8_t BSP_TIME_Delay(uint32_t ms);
#ifdef __cplusplus
}
#endif
