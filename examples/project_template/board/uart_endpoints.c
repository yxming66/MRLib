#include "board/uart_endpoints.h"
#include "bsp/uart.h"
#include "usart.h"

int Board_RegisterUartEndpoints(void) {
  // 按 Cube 生成的 usart.h 修改句柄；波特率、DMA、NVIC 仍由 Cube 配置。
  const BSP_UART_Endpoint_t endpoints[] = {
      {"usart1", &huart1},
      {"uart5", &huart5},
  };
  return BSP_UART_Configure(endpoints, sizeof(endpoints) / sizeof(endpoints[0])) == BSP_OK;
}
