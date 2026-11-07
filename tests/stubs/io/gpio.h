#pragma once
#include <stdint.h>
#include "cmsis_compiler.h"
typedef enum { GPIO_PIN_RESET, GPIO_PIN_SET } GPIO_PinState;
typedef struct { uint16_t levels; } GPIO_TypeDef;
typedef struct { uint32_t IMR1, PR1; } EXTI_Core_TypeDef;
extern EXTI_Core_TypeDef io_exti;
#define EXTI_D1 (&io_exti)
#define __HAL_GPIO_EXTI_CLEAR_IT(pins) (io_exti.PR1 &= ~(uint32_t)(pins))
void HAL_GPIO_WritePin(GPIO_TypeDef*, uint16_t, GPIO_PinState);
void HAL_GPIO_TogglePin(GPIO_TypeDef*, uint16_t);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef*, uint16_t);
void HAL_GPIO_EXTI_Callback(uint16_t);
