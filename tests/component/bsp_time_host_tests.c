#include <assert.h>
#include <limits.h>

#include "bsp/time.h"
#include "cmsis_os2.h"
#include "bsp/time_source.h"

static bool ready;
static uint64_t now_us;
static uint32_t increment;
static osKernelState_t state;
static osStatus_t delay_result;
static uint32_t hz, delayed_ticks, delayed_ms, freq_calls;
osKernelState_t osKernelGetState(void) { return state; }
uint32_t osKernelGetTickFreq(void) {
  ++freq_calls;
  return hz;
}
osStatus_t osDelay(uint32_t ticks) {
  delayed_ticks = ticks;
  return delay_result;
}
bool Board_TimeIsReady(void) { return ready; }
uint64_t Board_TimeNowUs(void) { const uint64_t value = now_us; now_us += increment; return value; }
int8_t Board_TimeDelayMs(uint32_t ms) { delayed_ms = ms; return BSP_OK; }

int main(void) {
  assert(BSP_TIME_Get_us() == 0 && BSP_TIME_Get_ms() == 0);
  assert(BSP_TIME_Delay_us(1) == BSP_ERR && BSP_TIME_Delay_ms(1) == BSP_ERR);
  ready = true;
  state = osKernelInactive;
  assert(BSP_TIME_Delay_ms(7) == BSP_OK && delayed_ms == 7 && freq_calls == 0);
  state = osKernelRunning;
  hz = 2000;
  assert(BSP_TIME_Delay_ms(7) == BSP_OK && delayed_ticks == 14);
  hz = 128;
  assert(BSP_TIME_Delay_ms(9) == BSP_OK && delayed_ticks == 2);
  assert(BSP_TIME_Delay_ms(0) == BSP_OK && delayed_ticks == 1);
  hz = 0;
  assert(BSP_TIME_Delay_ms(1) == BSP_ERR);
  hz = UINT32_MAX;
  assert(BSP_TIME_Delay_ms(UINT32_MAX) == BSP_ERR);
  hz = 1000;
  delay_result = osError;
  assert(BSP_TIME_Delay_ms(1) == BSP_ERR);
  state = osKernelLocked;
  assert(BSP_TIME_Delay_ms(1) == BSP_ERR);
  now_us = ((uint64_t)UINT32_MAX + 2u) * 1000u + 25u;
  assert(BSP_TIME_Get_us() == now_us && BSP_TIME_Get_ms() == 1u);
  assert(BSP_TIME_Get() == now_us);
  increment = 3;
  assert(BSP_TIME_Delay_us(10) == BSP_OK);
  assert(now_us >= ((uint64_t)UINT32_MAX + 2u) * 1000u + 35u);
  return 0;
}
