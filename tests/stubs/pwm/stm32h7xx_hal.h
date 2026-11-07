#pragma once
#include <stdint.h>
typedef enum { HAL_OK, HAL_ERROR } HAL_StatusTypeDef;
typedef struct { uint32_t PSC, ARR, CNT, EGR, CCR[4], CR1; } TIM_TypeDef;
typedef struct {
  TIM_TypeDef* Instance;
  struct { uint32_t Prescaler, Period, CounterMode, AutoReloadPreload; } Init;
  unsigned State;
} TIM_HandleTypeDef;
#define TIM_CHANNEL_1 0u
#define TIM_CHANNEL_2 4u
#define TIM_CHANNEL_3 8u
#define TIM_CHANNEL_4 12u
#define TIM_COUNTERMODE_UP 0u
#define HAL_TIM_STATE_READY 1u
#define TIM_EGR_UG 1u
#define TIM_CR1_CEN 1u
#define __HAL_TIM_GET_AUTORELOAD(h) ((h)->Instance->ARR)
#define __HAL_TIM_GET_COMPARE(h,c) ((h)->Instance->CCR[(c)/4u])
#define __HAL_TIM_SET_AUTORELOAD(h,v) ((h)->Instance->ARR = (v))
#define __HAL_TIM_SET_PRESCALER(h,v) ((h)->Instance->PSC = (v))
#define __HAL_TIM_SET_COUNTER(h,v) ((h)->Instance->CNT = (v))
#define __HAL_TIM_SET_COMPARE(h,c,v) ((h)->Instance->CCR[(c)/4u] = (v))
HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*, uint32_t);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef*, uint32_t);
