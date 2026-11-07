/* Includes ----------------------------------------------------------------- */
#include "bsp/time.h"

#include <cmsis_os2.h>

#include "bsp.h"
#include "main.h"
#include "stm32h7xx_hal_tim.h"

/* USER INCLUDE BEGIN */

/* USER INCLUDE END */
/* Private define ----------------------------------------------------------- */
/* USER DEFINE BEGIN */

/* USER DEFINE END */

/* Private macro ------------------------------------------------------------ */
/* Private typedef ---------------------------------------------------------- */
/* USER STRUCT BEGIN */

/* USER STRUCT END */

/* Private variables -------------------------------------------------------- */
extern TIM_HandleTypeDef htim2;

/* Private function  -------------------------------------------------------- */
/* Exported functions ------------------------------------------------------- */

uint32_t BSP_TIME_Get_ms(void) { return HAL_GetTick(); }

uint64_t BSP_TIME_Get_us(void) {
  // Read the HAL tick, timer counter and pending overflow as one snapshot.
  // TIM2 must be 1 MHz / ARR=999; its IRQ must not remain masked for >=1 ms.
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  uint32_t ms = HAL_GetTick();
  const uint32_t pending = __HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE);
  uint32_t us_in_ms = __HAL_TIM_GET_COUNTER(&htim2);
  if (pending != RESET) {
    ms++;
  } else if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE) != RESET) {
    // The timer wrapped between the first flag read and counter read.
    us_in_ms = __HAL_TIM_GET_COUNTER(&htim2);
    ms++;
  }
  if (token == 0u) __enable_irq();
  return (uint64_t)ms * 1000ULL + (uint64_t)us_in_ms;
}

uint64_t BSP_TIME_Get(void) __attribute__((alias("BSP_TIME_Get_us")));

int8_t BSP_TIME_Delay_ms(uint32_t ms) {
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
      HAL_Delay(ms);
      return BSP_OK;
    default:
      return BSP_ERR;
  }
}

/* 微秒忙等延时：只允许极短硬件初始化/临界时序使用，禁止任务流程等待。 */
int8_t BSP_TIME_Delay_us(uint32_t us) {
  uint64_t start = BSP_TIME_Get_us();
  while (BSP_TIME_Get_us() - start < us) {
    // 等待us时间
  }
  return BSP_OK;
}

int8_t BSP_TIME_Delay(uint32_t ms) __attribute__((alias("BSP_TIME_Delay_ms")));

/* USER FUNCTION BEGIN */

/* USER FUNCTION END */
