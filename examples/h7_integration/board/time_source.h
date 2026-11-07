#pragma once
#include "bsp/time_source.h"
#include "stm32h7xx_hal.h"
#ifdef __cplusplus
extern "C" {
#endif
int Board_TimeInit(void);
void Board_TimeOnUpdate(TIM_HandleTypeDef* timer);
#ifdef __cplusplus
}
#endif
