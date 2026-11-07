// Real uart.c with deterministic HAL calls. No board-global huart symbols are
// defined: all hardware dependencies must arrive via the registration table.
#include <cassert>
#include <cstring>
#include "board_bindings.hpp"

extern "C" {
uint32_t io_primask;
static UART_HandleTypeDef* sent_handle;
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef* h, uint8_t*, uint16_t) {
  sent_handle = h;
  return HAL_OK;
}
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t) { return HAL_ERROR; }
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t) { return HAL_ERROR; }
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef*, uint8_t*, uint16_t) { return HAL_ERROR; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t) { return HAL_ERROR; }
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef*, uint8_t*, uint16_t) { return HAL_ERROR; }
}
static unsigned completions;
static void Completed() { ++completions; }

int main(int argc, char** argv) {
  assert(argc == 2);
  USART_TypeDef instances[3]{};
  UART_HandleTypeDef handles[3]{};
  for (unsigned i = 0; i < 3; ++i) handles[i].Instance = &instances[i];
  // UART1 is deliberately NOT slot 1. Slot allocation is not a hardware ID.
  const BSP_UART_Endpoint_t endpoints[] = {
      {"uart9", &handles[0]}, {"uart5", &handles[1]}, {"usart1", &handles[2]}};
  assert(!mr::generated::ValidateUartBindings());
  assert(BSP_UART_Configure(endpoints, 3) == BSP_OK);
  assert(BSP_UART_GetHandle(BSP_UART_9) == nullptr);
  assert(BSP_UART_RegisterCallback(BSP_UART_9, BSP_UART_TX_CPLT_CB, Completed) != BSP_OK);
  assert(mr::generated::ValidateUartBindings());
  const auto logical = mr::generated::CommunicationUart("telemetry");
  const auto expected = BSP_UART_FindEndpoint(argv[1]);
  assert(logical == expected && expected != BSP_UART_ERR);
  assert(mr::generated::Uart("telemetry_uart") == expected);
  assert(mr::generated::Uart("remote") == BSP_UART_FindEndpoint("uart5"));
  assert(mr::generated::Uart("missing") == BSP_UART_ERR);
  assert(mr::generated::CommunicationUart("missing") == BSP_UART_ERR);
  uint8_t data[] = {1, 2, 3};
  assert(BSP_UART_Transmit(logical, data, 3, false) == BSP_OK);
  assert(sent_handle == BSP_UART_GetHandle(expected));
  assert(BSP_UART_RegisterCallback(logical, BSP_UART_TX_CPLT_CB, Completed) == BSP_OK);
  HAL_UART_TxCpltCallback(BSP_UART_GetHandle(BSP_UART_FindEndpoint("uart5")));
  assert(completions == 0);
  HAL_UART_TxCpltCallback(sent_handle);
  assert(completions == 1);
}
