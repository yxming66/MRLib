#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "bsp/bsp.h"

#ifdef __cplusplus
extern "C" {
#endif

// Opaque slots, unrelated to timer numbers or robot functions.
typedef enum {
  BSP_PWM_0, BSP_PWM_1, BSP_PWM_2, BSP_PWM_3,
  BSP_PWM_4, BSP_PWM_5, BSP_PWM_6, BSP_PWM_7,
  BSP_PWM_8, BSP_PWM_9, BSP_PWM_10, BSP_PWM_11,
  BSP_PWM_12, BSP_PWM_13, BSP_PWM_14, BSP_PWM_15,
  BSP_PWM_NUM, BSP_PWM_ERR,
} BSP_PWM_Channel_t;

typedef struct {
  const char* name;
  TIM_HandleTypeDef* timer;
  uint32_t channel;            // TIM_CHANNEL_1..4, normal PWM1, up-counting.
  uint32_t input_clock_hz;     // Actual timer kernel clock, supplied by board.
  uint8_t counter_bits;        // 16 or 32; board verifies the timer capability.
  bool allow_frequency_change; // Exclusive timer ownership, including other users.
} BSP_PWM_Endpoint_t;

// Startup only, after HAL PWM configuration and before any output starts.
// Copies the table, but names/handles must have static lifetime. Validate first,
// publish once; no runtime rebinding. Shared timers must use the SAME handle,
// clock and width, and disallow frequency changes on every registered channel.
int8_t BSP_PWM_Configure(const BSP_PWM_Endpoint_t* endpoints, uint8_t count);
BSP_PWM_Channel_t BSP_PWM_FindEndpoint(const char* name);

// Single task/owner per timer; these calls do not lock or schedule access.
int8_t BSP_PWM_Start(BSP_PWM_Channel_t ch);
int8_t BSP_PWM_Stop(BSP_PWM_Channel_t ch);
// Finite duty, clamped to [0,1]. Full duty fails if ARR+1 cannot fit in CCR.
int8_t BSP_PWM_SetComp(BSP_PWM_Channel_t ch, float duty_cycle);
// Explicitly permitted exclusive timers only. Preserves duty and sets PSC/ARR;
// the update event resets phase. PWM/encoder/capture/DMA users must not share it.
int8_t BSP_PWM_SetFreq(BSP_PWM_Channel_t ch, float freq);
// Uses current PSC/ARR, rounds down to a timer tick and clamps to the period.
int8_t BSP_PWM_SetPulseUs(BSP_PWM_Channel_t ch, uint32_t pulse_us);
uint32_t BSP_PWM_GetAutoReloadPreload(BSP_PWM_Channel_t ch); // Invalid returns 0.
uint32_t BSP_PWM_GetChannel(BSP_PWM_Channel_t ch); // Invalid returns UINT32_MAX.
TIM_HandleTypeDef* BSP_PWM_GetHandle(BSP_PWM_Channel_t ch); // Invalid returns NULL.

#ifdef __cplusplus
}
#endif
