#include "bsp/time.h"
#include "bsp/time_source.h"
#include <cmsis_os2.h>

uint64_t BSP_TIME_Get_us(void) { return Board_TimeIsReady() ? Board_TimeNowUs() : 0; }
uint32_t BSP_TIME_Get_ms(void) { return (uint32_t)(BSP_TIME_Get_us() / 1000u); }
uint64_t BSP_TIME_Get(void) { return BSP_TIME_Get_us(); }

int8_t BSP_TIME_Delay_ms(uint32_t ms) {
  if (!Board_TimeIsReady()) return BSP_ERR;
  switch (osKernelGetState()) {
    case osKernelRunning: {
      const uint32_t hz = osKernelGetTickFreq();
      if (hz == 0) return BSP_ERR;
      // Round up without overflow; CMSIS reserves osWaitForever (UINT32_MAX).
      const uint64_t ticks = ((uint64_t)ms * hz + 999u) / 1000u;
      if (ticks >= UINT32_MAX) return BSP_ERR;
      return osDelay(ticks ? (uint32_t)ticks : 1u) == osOK ? BSP_OK : BSP_ERR;
    }
    case osKernelInactive:
    case osKernelReady:
      return Board_TimeDelayMs(ms);
    default:
      return BSP_ERR;
  }
}

/* 微秒忙等延时：只允许极短硬件初始化/临界时序使用，禁止任务流程等待。 */
int8_t BSP_TIME_Delay_us(uint32_t us) {
  if (!Board_TimeIsReady()) return BSP_ERR;
  uint64_t start = BSP_TIME_Get_us();
  while (BSP_TIME_Get_us() - start < us) {
    // 等待us时间
  }
  return BSP_OK;
}

int8_t BSP_TIME_Delay(uint32_t ms) { return BSP_TIME_Delay_ms(ms); }
