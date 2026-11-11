#pragma once
#include "stm32h7xx_hal.h"
#include "bsp/time_source.h"
#ifdef __cplusplus
extern "C" {
#endif
int Board_TimeInit(void);
/* Called before Cube's HAL_IncTick; does not increment HAL tick itself. */
void Board_TimeOnUpdate(TIM_HandleTypeDef* timer);
#ifdef __cplusplus
}
#endif
