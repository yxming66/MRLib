#include "bsp/pwm.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static BSP_PWM_Endpoint_t endpoints[BSP_PWM_NUM];
static uint8_t endpoint_count;
static bool configured;

static const BSP_PWM_Endpoint_t* Get(BSP_PWM_Channel_t ch) {
  return (unsigned)ch < endpoint_count ? &endpoints[ch] : NULL;
}
static uint32_t CounterMax(const BSP_PWM_Endpoint_t* endpoint) {
  return endpoint->counter_bits == 32 ? UINT32_MAX : UINT16_MAX;
}
static bool ValidChannel(uint32_t channel) {
  return channel == TIM_CHANNEL_1 || channel == TIM_CHANNEL_2 ||
         channel == TIM_CHANNEL_3 || channel == TIM_CHANNEL_4;
}

int8_t BSP_PWM_Configure(const BSP_PWM_Endpoint_t* table, uint8_t count) {
  if (configured || !table || !count || count > BSP_PWM_NUM) return BSP_ERR;
  for (uint8_t i = 0; i < count; ++i) {
    const BSP_PWM_Endpoint_t* p = &table[i];
    if (!p->name || !p->name[0] || !p->timer || !p->timer->Instance ||
        p->timer->State != HAL_TIM_STATE_READY || !ValidChannel(p->channel) ||
        !p->input_clock_hz || (p->counter_bits != 16 && p->counter_bits != 32) ||
        p->timer->Init.CounterMode != TIM_COUNTERMODE_UP ||
        (p->timer->Instance->CR1 & TIM_CR1_CEN) != 0 ||
        p->timer->Instance->PSC > UINT16_MAX ||
        __HAL_TIM_GET_AUTORELOAD(p->timer) > CounterMax(p)) return BSP_ERR;
    for (uint8_t j = 0; j < i; ++j) {
      const BSP_PWM_Endpoint_t* q = &table[j];
      if (strcmp(p->name, q->name) == 0) return BSP_ERR;
      if (p->timer->Instance != q->timer->Instance) continue;
      if (p->timer != q->timer || p->channel == q->channel ||
          p->input_clock_hz != q->input_clock_hz || p->counter_bits != q->counter_bits ||
          p->allow_frequency_change || q->allow_frequency_change) return BSP_ERR;
    }
  }
  memcpy(endpoints, table, count * sizeof(*table));
  endpoint_count = count;
  configured = true;
  return BSP_OK;
}

BSP_PWM_Channel_t BSP_PWM_FindEndpoint(const char* name) {
  if (name)
    for (uint8_t i = 0; i < endpoint_count; ++i)
      if (strcmp(name, endpoints[i].name) == 0) return (BSP_PWM_Channel_t)i;
  return BSP_PWM_ERR;
}
TIM_HandleTypeDef* BSP_PWM_GetHandle(BSP_PWM_Channel_t ch) {
  const BSP_PWM_Endpoint_t* p = Get(ch);
  return p ? p->timer : NULL;
}
uint32_t BSP_PWM_GetChannel(BSP_PWM_Channel_t ch) {
  const BSP_PWM_Endpoint_t* p = Get(ch);
  return p ? p->channel : UINT32_MAX;
}
uint32_t BSP_PWM_GetAutoReloadPreload(BSP_PWM_Channel_t ch) {
  const BSP_PWM_Endpoint_t* p = Get(ch);
  return p ? p->timer->Init.AutoReloadPreload : 0;
}
int8_t BSP_PWM_Start(BSP_PWM_Channel_t ch) {
  const BSP_PWM_Endpoint_t* p = Get(ch);
  return p && HAL_TIM_PWM_Start(p->timer, p->channel) == HAL_OK ? BSP_OK : BSP_ERR;
}
int8_t BSP_PWM_Stop(BSP_PWM_Channel_t ch) {
  const BSP_PWM_Endpoint_t* p = Get(ch);
  return p && HAL_TIM_PWM_Stop(p->timer, p->channel) == HAL_OK ? BSP_OK : BSP_ERR;
}
int8_t BSP_PWM_SetComp(BSP_PWM_Channel_t ch, float duty) {
  const BSP_PWM_Endpoint_t* p = Get(ch);
  if (!p || !isfinite(duty)) return BSP_ERR;
  if (duty < 0) duty = 0;
  if (duty > 1) duty = 1;
  const double ticks = (double)__HAL_TIM_GET_AUTORELOAD(p->timer) + 1.0;
  const double compare = floor(ticks * (double)duty);
  if (compare > CounterMax(p)) return BSP_ERR;
  __HAL_TIM_SET_COMPARE(p->timer, p->channel, (uint32_t)compare);
  return BSP_OK;
}
int8_t BSP_PWM_SetFreq(BSP_PWM_Channel_t ch, float freq) {
  const BSP_PWM_Endpoint_t* p = Get(ch);
  if (!p || !p->allow_frequency_change || !isfinite(freq) || freq <= 0) return BSP_ERR;
  // Reserve the top ARR value so 100% duty always fits in CCR at the new period.
  const double max_ticks = CounterMax(p);
  const double ideal_ticks = (double)p->input_clock_hz / (double)freq;
  if (!isfinite(ideal_ticks) || ideal_ticks < 2.0 || ideal_ticks > max_ticks * 65536.0)
    return BSP_ERR;
  const double divisor = ceil(ideal_ticks / max_ticks);
  const double ticks = floor(ideal_ticks / divisor + 0.5);
  if (divisor < 1 || divisor > 65536 || ticks < 2 || ticks > max_ticks) return BSP_ERR;
  const double old_ticks = (double)__HAL_TIM_GET_AUTORELOAD(p->timer) + 1.0;
  double duty = (double)__HAL_TIM_GET_COMPARE(p->timer, p->channel) / old_ticks;
  if (duty > 1) duty = 1;
  const uint32_t compare = (uint32_t)floor(ticks * duty);
  const uint32_t prescaler = (uint32_t)divisor - 1;
  const uint32_t period = (uint32_t)ticks - 1;
  // All validation precedes hardware mutation. Caller serializes the timer.
  p->timer->Init.Prescaler = prescaler;
  p->timer->Init.Period = period;
  __HAL_TIM_SET_PRESCALER(p->timer, prescaler);
  __HAL_TIM_SET_AUTORELOAD(p->timer, period);
  __HAL_TIM_SET_COMPARE(p->timer, p->channel, compare);
  __HAL_TIM_SET_COUNTER(p->timer, 0);
  p->timer->Instance->EGR = TIM_EGR_UG;
  return BSP_OK;
}
int8_t BSP_PWM_SetPulseUs(BSP_PWM_Channel_t ch, uint32_t pulse_us) {
  const BSP_PWM_Endpoint_t* p = Get(ch);
  if (!p || p->timer->Instance->PSC > UINT16_MAX) return BSP_ERR;
  const uint64_t denominator = ((uint64_t)p->timer->Instance->PSC + 1u) * 1000000u;
  uint64_t compare = (uint64_t)pulse_us * p->input_clock_hz / denominator;
  const uint64_t ticks = (uint64_t)__HAL_TIM_GET_AUTORELOAD(p->timer) + 1u;
  if (compare > ticks) compare = ticks;
  if (compare > CounterMax(p)) return BSP_ERR;
  __HAL_TIM_SET_COMPARE(p->timer, p->channel, (uint32_t)compare);
  return BSP_OK;
}
