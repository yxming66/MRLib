#include <assert.h>
#include <limits.h>
#include "../../examples/h7_integration/board/time_source.h"
TIM_TypeDef time_timer2 = {239, 999, 25, TIM_CR1_CEN, TIM_IT_UPDATE, 0};
TIM_HandleTypeDef htim2 = {&time_timer2, {TIM_COUNTERMODE_UP}};
uint32_t time_primask;
static uint32_t tick = UINT32_MAX - 1u, hclk = 240000000, divider = RCC_APB1_DIV2;
static unsigned priority = 15, delayed, ipsr;
static bool wrap_during_read;
uint32_t HAL_GetTick(void) { return tick; }
void HAL_IncTick(void) { ++tick; }
void HAL_Delay(uint32_t ms) { delayed = ms; }
uint32_t __get_IPSR(void) { return ipsr; }
void HAL_RCC_GetClockConfig(RCC_ClkInitTypeDef* clocks, uint32_t* latency) {
  clocks->APB1CLKDivider = divider; *latency = 0;
}
uint32_t HAL_RCC_GetHCLKFreq(void) { return hclk; }
void HAL_NVIC_SetPriority(unsigned irq, unsigned level, unsigned sub) {
  assert(irq == TIM2_IRQn && sub == 0); priority = level;
}
uint32_t Time_ReadCounter(TIM_HandleTypeDef* timer) {
  const uint32_t value = timer->Instance->CNT;
  if (wrap_during_read) {
    wrap_during_read = false; timer->Instance->CNT = 7; timer->Instance->SR = 1;
  }
  return value;
}
int main(void) {
  assert(!Board_TimeIsReady() && Board_TimeNowUs() == 0);
  assert(Board_TimeDelayMs(1) == BSP_ERR);
  divider = 1; assert(!Board_TimeInit()); divider = RCC_APB1_DIV2;
  time_timer2.ARR = 1000; assert(!Board_TimeInit()); time_timer2.ARR = 999;
  time_timer2.PSC = 119; assert(!Board_TimeInit()); time_timer2.PSC = 239;
  time_timer2.DIER = 0; assert(!Board_TimeInit()); time_timer2.DIER = TIM_IT_UPDATE;
  assert(Board_TimeInit() && priority == 0 && Board_TimeIsReady());
  assert(!Board_TimeInit());
  const uint64_t epoch = (uint64_t)(UINT32_MAX - 1u) * 1000u;
  assert(Board_TimeNowUs() == epoch + 25);
  time_timer2.SR = TIM_FLAG_UPDATE;
  assert(Board_TimeNowUs() == epoch + 1025); // Pending IRQ, before callback.
  time_timer2.SR = 0;
  Board_TimeOnUpdate(&htim2);
  assert(Board_TimeNowUs() == epoch + 1025); // No double count after callback.
  Board_TimeOnUpdate(&htim2); // HAL tick wraps, 64-bit board epoch keeps increasing.
  assert(tick == 0 && Board_TimeNowUs() == epoch + 2025);
  time_timer2.CNT = 999; wrap_during_read = true;
  assert(Board_TimeNowUs() == epoch + 3007);
  time_timer2.SR = 0; Board_TimeOnUpdate(&htim2);
  assert(Board_TimeNowUs() == epoch + 3007);
  TIM_HandleTypeDef alias = htim2;
  Board_TimeOnUpdate(&alias);
  assert(Board_TimeNowUs() == epoch + 3007);
  time_primask = 1;
  assert(Board_TimeNowUs() == epoch + 3007 && time_primask == 1);
  assert(Board_TimeDelayMs(1) == BSP_ERR);
  time_primask = 0; ipsr = 1;
  assert(Board_TimeDelayMs(1) == BSP_ERR);
  ipsr = 0;
  assert(Board_TimeDelayMs(5) == BSP_OK && delayed == 5);
  return 0;
}
