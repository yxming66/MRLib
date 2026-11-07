#pragma once
#include <stdint.h>
extern uint32_t io_primask;
static inline uint32_t __get_PRIMASK(void) { return io_primask; }
static inline void __disable_irq(void) { io_primask = 1; }
static inline void __enable_irq(void) { io_primask = 0; }
