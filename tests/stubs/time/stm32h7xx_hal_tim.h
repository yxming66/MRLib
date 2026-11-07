#pragma once
#include <stdint.h>
typedef struct { uint32_t counter, pending; } TIM_HandleTypeDef;
#define __HAL_TIM_GET_COUNTER(p) ((p)->counter)
#define __HAL_TIM_GET_FLAG(p, flag) ((void)(flag), (p)->pending)
#define TIM_FLAG_UPDATE 1u
#define RESET 0u
