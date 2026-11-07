#pragma once
#include <stdint.h>
#include "main.h"
typedef struct { uint32_t PSC, ARR, CNT, CR1, DIER, SR; } TIM_TypeDef;
typedef struct { TIM_TypeDef* Instance; struct { uint32_t CounterMode; } Init; } TIM_HandleTypeDef;
typedef struct { uint32_t APB1CLKDivider; } RCC_ClkInitTypeDef;
extern TIM_TypeDef time_timer2;
#define TIM2 (&time_timer2)
#define TIM_COUNTERMODE_UP 0u
#define TIM_CR1_CEN 1u
#define TIM_IT_UPDATE 1u
#define TIM_FLAG_UPDATE 1u
#define TIM2_IRQn 28u
#define RCC_APB1_DIV2 2u
#define RESET 0u
uint32_t Time_ReadCounter(TIM_HandleTypeDef*);
#define __HAL_TIM_GET_COUNTER(p) Time_ReadCounter(p)
#define __HAL_TIM_GET_FLAG(p,f) ((p)->Instance->SR & (f))
#define __HAL_TIM_GET_AUTORELOAD(p) ((p)->Instance->ARR)
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef*, uint32_t*);
uint32_t HAL_RCC_GetHCLKFreq(void);
void HAL_NVIC_SetPriority(unsigned irq, unsigned priority, unsigned subpriority);
uint32_t __get_IPSR(void);
void HAL_IncTick(void);
