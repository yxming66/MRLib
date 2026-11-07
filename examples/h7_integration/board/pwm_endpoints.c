// Project hardware inventory; PWM1/up-counting configured by Cube.
#include "bsp/pwm.h"
#include "tim.h"

int Board_RegisterPwmEndpoints(void) {
  RCC_ClkInitTypeDef clocks = {0};
  uint32_t latency;
  HAL_RCC_GetClockConfig(&clocks, &latency);
  // For this H723 board APB1/APB2 divide HCLK by two. Timer kernel clocks equal
  // HCLK for either TIMPRE setting at this divider. Other clock trees need an
  // explicit board calculation; the library must not guess timer domains.
  if (clocks.APB1CLKDivider != RCC_APB1_DIV2 || clocks.APB2CLKDivider != RCC_APB2_DIV2)
    return 0;
  const uint32_t timer_hz = HAL_RCC_GetHCLKFreq();
  if (!timer_hz) return 0;
  const BSP_PWM_Endpoint_t endpoints[] = {
      {"tim12_ch2", &htim12, TIM_CHANNEL_2, timer_hz, 16, true},
      {"tim3_ch4", &htim3, TIM_CHANNEL_4, timer_hz, 16, true},
      // Shared TIM1 period belongs to Cube/board. Neither channel may change it.
      {"tim1_ch1", &htim1, TIM_CHANNEL_1, timer_hz, 16, false},
      {"tim1_ch3", &htim1, TIM_CHANNEL_3, timer_hz, 16, false},
  };
  return BSP_PWM_Configure(endpoints, sizeof(endpoints) / sizeof(endpoints[0])) == BSP_OK;
}
