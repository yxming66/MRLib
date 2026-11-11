#include "board/pwm_endpoints.h"
#include "bsp/pwm.h"
#include "tim.h"

int Board_RegisterPwmEndpoints(void) {
  // 此 H723 工程 APB1 /2，TIM3/TIM12 内核时钟等于 HCLK。
  // 换时钟树或芯片时必须重新核算，不可直接照抄。
  RCC_ClkInitTypeDef clocks = {0};
  uint32_t latency;
  HAL_RCC_GetClockConfig(&clocks, &latency);
  const uint32_t hz = HAL_RCC_GetHCLKFreq();
  if (!hz || clocks.APB1CLKDivider != RCC_APB1_DIV2) return 0;
  // 仅登记，不启动输出；共享 TIM3 的通道禁止单独改频率。
  const BSP_PWM_Endpoint_t endpoints[] = {
      {"buzzer", &htim12, TIM_CHANNEL_2, hz, 16, false},
      {"tim3_ch2", &htim3, TIM_CHANNEL_2, hz, 16, false},
      {"tim3_ch4", &htim3, TIM_CHANNEL_4, hz, 16, false},
  };
  return BSP_PWM_Configure(endpoints, sizeof(endpoints) / sizeof(endpoints[0])) == BSP_OK;
}
