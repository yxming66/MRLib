#pragma once
#include <stddef.h>
#include <stdint.h>

#include "cmsis_compiler.h"
typedef enum { HAL_OK, HAL_ERROR, HAL_BUSY } HAL_StatusTypeDef;
typedef struct {
  struct {
    unsigned Mode;
    unsigned MemDataAlignment, PeriphDataAlignment;
  } Init;
} DMA_HandleTypeDef;
typedef struct {
  unsigned ISR, CR1, CR3;
} USART_TypeDef;
extern USART_TypeDef io_uart_instances[6];
typedef struct {
  USART_TypeDef* Instance;
  DMA_HandleTypeDef* hdmarx;
  unsigned gState, RxState, ErrorCode, ReceptionType;
} UART_HandleTypeDef;
extern UART_HandleTypeDef huart1, huart5, huart7, huart8, huart9, huart10;
#define UART5 (&io_uart_instances[0])
#define UART9 (&io_uart_instances[1])
#define USART1 (&io_uart_instances[2])
#define UART8 (&io_uart_instances[3])
#define UART7 (&io_uart_instances[4])
#define USART10 (&io_uart_instances[5])
#define DMA_NORMAL 0u
#define DMA_MDATAALIGN_BYTE 0u
#define DMA_PDATAALIGN_BYTE 0u
#define DMA_IT_HT 1u
#define HAL_UART_STATE_READY 0u
#define HAL_UART_ERROR_DMA 0x10u
#define UART_FLAG_IDLE 1u
#define UART_CLEAR_OREF 1u
#define UART_CLEAR_NEF 2u
#define UART_CLEAR_PEF 4u
#define UART_CLEAR_FEF 8u
#define UART_RXDATA_FLUSH_REQUEST 1u
#define __HAL_UART_GET_FLAG(h, f) ((void)(h), (void)(f), 1)
#define __HAL_UART_CLEAR_IDLEFLAG(h) ((void)(h))
#define __HAL_UART_CLEAR_FLAG(h, f) ((void)(h), (void)(f))
#define __HAL_UART_SEND_REQ(h, f) ((void)(h), (void)(f))
#define __HAL_DMA_DISABLE_IT(h, f) ((void)(h), (void)(f))
HAL_StatusTypeDef HAL_UART_Transmit_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_UART_Transmit_IT(UART_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_UART_Receive_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_UART_Receive_IT(UART_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_IT(UART_HandleTypeDef*, uint8_t*, uint16_t);
HAL_StatusTypeDef HAL_UART_AbortReceive(UART_HandleTypeDef*);
HAL_StatusTypeDef HAL_UART_AbortTransmit(UART_HandleTypeDef*);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef*, uint16_t);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef*);
void HAL_UART_ErrorCallback(UART_HandleTypeDef*);
void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef*);

#define UART_CLEAR_IDLEF 16u
#define HAL_UART_ERROR_NONE 0u
#define __HAL_DMA_GET_COUNTER(h) ((void)(h), 0u)
#define HAL_UART_RECEPTION_STANDARD 0u
