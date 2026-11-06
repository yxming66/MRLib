#pragma once
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
extern uint32_t mr_test_primask;
#ifdef __cplusplus
}
#endif
static inline uint32_t __get_PRIMASK(void) { return mr_test_primask; }
static inline void __disable_irq(void) { mr_test_primask = 1; }
static inline void __enable_irq(void) { mr_test_primask = 0; }
static inline void __DMB(void) {}
