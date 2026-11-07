// Board-owned hardware inventory. Cube initializes these handles/peripherals.
// This table has no RC/PC/referee roles; config.yaml assigns project uses.
#include "bsp/uart.h"
#include "usart.h"

int Board_RegisterUartEndpoints(void) {
  static const BSP_UART_Endpoint_t endpoints[] = {
      {"uart5", &huart5}, {"uart9", &huart9}, {"usart1", &huart1},
      {"uart8", &huart8}, {"uart7", &huart7}, {"usart10", &huart10},
  };
  return BSP_UART_Configure(endpoints, sizeof(endpoints) / sizeof(endpoints[0])) == BSP_OK;
}
