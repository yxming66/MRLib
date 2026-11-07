#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"
#include "bsp/bsp.h"

#ifdef __cplusplus
extern "C" {
#endif

// Neutral slots, independent of pin numbers and device functions.
typedef enum {
  BSP_GPIO_0, BSP_GPIO_1, BSP_GPIO_2, BSP_GPIO_3,
  BSP_GPIO_4, BSP_GPIO_5, BSP_GPIO_6, BSP_GPIO_7,
  BSP_GPIO_8, BSP_GPIO_9, BSP_GPIO_10, BSP_GPIO_11,
  BSP_GPIO_12, BSP_GPIO_13, BSP_GPIO_14, BSP_GPIO_15,
  BSP_GPIO_16, BSP_GPIO_17, BSP_GPIO_18, BSP_GPIO_19,
  BSP_GPIO_20, BSP_GPIO_21, BSP_GPIO_22, BSP_GPIO_23,
  BSP_GPIO_24, BSP_GPIO_25, BSP_GPIO_26, BSP_GPIO_27,
  BSP_GPIO_28, BSP_GPIO_29, BSP_GPIO_30, BSP_GPIO_31,
  BSP_GPIO_NUM, BSP_GPIO_ERR,
} BSP_GPIO_t;

typedef enum { BSP_GPIO_INPUT, BSP_GPIO_OUTPUT, BSP_GPIO_EXTI } BSP_GPIO_Mode_t;
typedef struct {
  const char* name;
  GPIO_TypeDef* port;
  uint16_t pin;                  // Exactly one GPIO_PIN_x bit.
  BSP_GPIO_Mode_t mode;           // Board verifies actual Cube configuration.
} BSP_GPIO_Endpoint_t;

// Startup only, after Cube GPIO init. Copies the table; names/ports stay valid.
// Registered EXTI lines start masked. Cube owns routing, edges, NVIC and IRQ handlers.
// H7 Cortex-M7 implementation; no output level or GPIO mode changes here.
int8_t BSP_GPIO_Configure(const BSP_GPIO_Endpoint_t* endpoints, uint8_t count);
BSP_GPIO_t BSP_GPIO_FindEndpoint(const char* name);
const BSP_GPIO_Endpoint_t* BSP_GPIO_GetEndpoint(BSP_GPIO_t gpio);

// One callback per EXTI endpoint, installed/removed while its line is masked.
// Never overwrites another callback. Removal must present the same function.
int8_t BSP_GPIO_RegisterCallback(BSP_GPIO_t gpio, void (*callback)(void));
int8_t BSP_GPIO_UnregisterCallback(BSP_GPIO_t gpio, void (*callback)(void));
// Masks/unmasks this EXTI line only; never disables a shared NVIC IRQ group.
// Enabling discards pending events accumulated while disabled, and needs a callback.
int8_t BSP_GPIO_EnableIRQ(BSP_GPIO_t gpio);
int8_t BSP_GPIO_DisableIRQ(BSP_GPIO_t gpio);

int8_t BSP_GPIO_WritePin(BSP_GPIO_t gpio, bool value);
int8_t BSP_GPIO_TogglePin(BSP_GPIO_t gpio);
bool BSP_GPIO_ReadPin(BSP_GPIO_t gpio); // Invalid returns false.

#ifdef __cplusplus
}
#endif
