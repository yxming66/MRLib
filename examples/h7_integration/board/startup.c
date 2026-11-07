#include "cmsis_os2.h"
#include "dma.h"
#include "fdcan.h"
#include "gpio.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

extern void RobotTask(void* argument);
_Static_assert(HSE_VALUE == 24000000UL, "This board clock requires a 24 MHz HSE");

/* Clock parameters match the read-only zzrobot H723 Cube board (24 MHz HSE).
 * They are board configuration, not library defaults. Verify for another PCB. */
static void ConfigureClock(void) {
  RCC_OscInitTypeDef osc = {0};
  RCC_ClkInitTypeDef clk = {0};
  if (HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) != HAL_OK) Error_Handler();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);
  while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
  }
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLM = 2;
  osc.PLL.PLLN = 40;
  osc.PLL.PLLP = 1;
  osc.PLL.PLLQ = 4;
  osc.PLL.PLLR = 2;
  osc.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  osc.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) Error_Handler();
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
                  RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1 | RCC_CLOCKTYPE_D1PCLK1;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.SYSCLKDivider = RCC_SYSCLK_DIV1;
  clk.AHBCLKDivider = RCC_HCLK_DIV2;
  clk.APB1CLKDivider = RCC_APB1_DIV2;
  clk.APB2CLKDivider = RCC_APB2_DIV2;
  clk.APB3CLKDivider = RCC_APB3_DIV2;
  clk.APB4CLKDivider = RCC_APB4_DIV2;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK) Error_Handler();
}

int main(void) {
  /* Explicit baseline: caches off; all data/heap/stacks are in AXI SRAM.
   * Cache-enabled production boards must supply their own MPU/cache policy. */
  SCB_DisableDCache();
  SCB_DisableICache();
  if (HAL_Init() != HAL_OK) Error_Handler();
  ConfigureClock();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_FDCAN3_Init();
  MX_UART5_Init();
  MX_UART7_Init();
  MX_UART9_Init();
  MX_USART10_UART_Init();
  MX_USART1_UART_Init();
  MX_UART8_Init();
  MX_SPI2_Init();
  MX_TIM12_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM1_Init();
  if (osKernelInitialize() != osOK) Error_Handler();
  const osThreadAttr_t task = {.name = "robot", .stack_size = 32768, .priority = osPriorityNormal};
  if (!osThreadNew(RobotTask, NULL, &task)) Error_Handler();
  if (osKernelStart() != osOK) Error_Handler();
  Error_Handler();
  return 0;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* timer) {
  if (timer->Instance == TIM2) HAL_IncTick();
}
void Error_Handler(void) {
  __disable_irq();
  for (;;) {
  }
}
