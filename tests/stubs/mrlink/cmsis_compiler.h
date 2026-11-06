#pragma once

#include <stdint.h>

static inline uint32_t __get_PRIMASK(void) { return 0U; }
static inline void __disable_irq(void) {}
static inline void __enable_irq(void) {}
static inline void __DMB(void) {}

