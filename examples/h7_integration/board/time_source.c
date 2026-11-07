// This project's HAL timer: TIM2, 1 MHz, ARR=999; never suspended/reconfigured.
#include "time_source.h"
extern TIM_HandleTypeDef htim2;
static volatile uint64_t elapsed_ms;
static volatile bool ready;

int Board_TimeInit(void) {
  if (ready) return 0;
  RCC_ClkInitTypeDef clocks = {0};
  uint32_t latency;
  HAL_RCC_GetClockConfig(&clocks, &latency);
  const uint32_t timer_hz = HAL_RCC_GetHCLKFreq();
  // At APB1 /2, both H723 TIMPRE choices give HCLK for TIM2.
  if (clocks.APB1CLKDivider != RCC_APB1_DIV2 || timer_hz % 1000000u ||
      !timer_hz || htim2.Instance != TIM2 || htim2.Init.CounterMode != TIM_COUNTERMODE_UP ||
      htim2.Instance->PSC != timer_hz / 1000000u - 1u ||
      __HAL_TIM_GET_AUTORELOAD(&htim2) != 999u ||
      !(htim2.Instance->CR1 & TIM_CR1_CEN) ||
      !(htim2.Instance->DIER & TIM_IT_UPDATE)) return 0;
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  // Timer IRQ must not be preempted by any normal ISR that reads this source:
  // HAL clears UIF before entering its period callback. Priority 0 closes that gap.
  HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
  elapsed_ms = HAL_GetTick();
  ready = true;
  if (!token) __enable_irq();
  return 1;
}
bool Board_TimeIsReady(void) { return ready; }
void Board_TimeOnUpdate(TIM_HandleTypeDef* timer) {
  if (timer != &htim2) return;
  if (ready) ++elapsed_ms;
  HAL_IncTick();
}
uint64_t Board_TimeNowUs(void) {
  if (!ready) return 0;
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  uint64_t ms = elapsed_ms;
  const uint32_t pending = __HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE);
  uint32_t part = __HAL_TIM_GET_COUNTER(&htim2);
  if (pending != RESET) ++ms;
  else if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
    part = __HAL_TIM_GET_COUNTER(&htim2);
    ++ms;
  }
  if (!token) __enable_irq();
  // Counter IRQ latency must remain <1 ms, including PRIMASK/FAULTMASK and
  // priority-0 ISRs; one UIF flag cannot represent multiple missed overflows.
  return ms * 1000u + part;
}
int8_t Board_TimeDelayMs(uint32_t ms) {
  if (!ready || __get_IPSR() || __get_PRIMASK()) return BSP_ERR;
  HAL_Delay(ms);
  return BSP_OK;
}
