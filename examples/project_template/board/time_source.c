#include "board/time_source.h"

/* TIM23 is the Cube HAL time base. TIM2 remains available for PWM. */
extern TIM_HandleTypeDef htim23;
static volatile uint64_t elapsed_ms;
static volatile bool ready;

int Board_TimeInit(void) {
  if (ready) return 0;
  RCC_ClkInitTypeDef clocks = {0};
  uint32_t latency;
  HAL_RCC_GetClockConfig(&clocks, &latency);
  const uint32_t hz = HAL_RCC_GetHCLKFreq();
  /* On this H723/APB2 /2 board, TIM23 clock equals HCLK for either TIMPRE. */
  if (clocks.APB2CLKDivider != RCC_APB2_DIV2 || !hz || hz % 1000000u ||
      htim23.Instance != TIM23 || htim23.Init.CounterMode != TIM_COUNTERMODE_UP ||
      htim23.Instance->PSC != hz / 1000000u - 1u ||
      __HAL_TIM_GET_AUTORELOAD(&htim23) != 999u ||
      !(htim23.Instance->CR1 & TIM_CR1_CEN) ||
      !(htim23.Instance->DIER & TIM_IT_UPDATE) ||
      HAL_GetTickFreq() != HAL_TICK_FREQ_1KHZ) return 0;
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  /* No normal time-reading ISR may preempt HAL between clearing UIF and
     calling Board_TimeOnUpdate. This IRQ must never call FreeRTOS APIs. */
  HAL_NVIC_SetPriority(TIM23_IRQn, 0, 0);
  elapsed_ms = HAL_GetTick();
  ready = true;
  if (!token) __enable_irq();
  return 1;
}
bool Board_TimeIsReady(void) { return ready; }
void Board_TimeOnUpdate(TIM_HandleTypeDef* timer) {
  if (timer == &htim23 && ready) ++elapsed_ms;
  /* The generated main.c callback calls HAL_IncTick exactly once. */
}
uint64_t Board_TimeNowUs(void) {
  if (!ready) return 0;
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  uint64_t ms = elapsed_ms;
  const uint32_t pending = __HAL_TIM_GET_FLAG(&htim23, TIM_FLAG_UPDATE);
  uint32_t part = __HAL_TIM_GET_COUNTER(&htim23);
  if (pending != RESET) ++ms;
  else if (__HAL_TIM_GET_FLAG(&htim23, TIM_FLAG_UPDATE) != RESET) {
    part = __HAL_TIM_GET_COUNTER(&htim23);
    ++ms;
  }
  if (!token) __enable_irq();
  /* Worst-case IRQ latency, including masked sections, must stay <1 ms. */
  return ms * 1000u + part;
}
int8_t Board_TimeDelayMs(uint32_t ms) {
  if (!ready || __get_IPSR() || __get_PRIMASK()) return BSP_ERR;
  HAL_Delay(ms);
  return BSP_OK;
}
