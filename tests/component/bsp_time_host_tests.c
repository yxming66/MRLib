#include <assert.h>
#include <limits.h>
#include "bsp/time.h"
#include "cmsis_os2.h"
#include "stm32h7xx_hal_tim.h"

TIM_HandleTypeDef htim2;
static osKernelState_t state;
static osStatus_t delay_result;
static uint32_t hz, delayed_ticks, delayed_ms, freq_calls, tick;
osKernelState_t osKernelGetState(void) { return state; }
uint32_t osKernelGetTickFreq(void) { ++freq_calls; return hz; }
osStatus_t osDelay(uint32_t ticks) { delayed_ticks = ticks; return delay_result; }
uint32_t HAL_GetTick(void) { return tick; }
void HAL_Delay(uint32_t ms) { delayed_ms = ms; }

int main(void) {
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
  tick = 10; htim2.counter = 25;
  assert(BSP_TIME_Get_us() == 10025 && BSP_TIME_Get_ms() == 10);
  htim2.pending = 1;
  assert(BSP_TIME_Get() == 11025);
  return 0;
}
