#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ----------------------------------------------------------------- */
#include <stdbool.h>
#include <stdint.h>
#include "stm32h7xx_hal.h"

#include "bsp/bsp.h"

/* USER INCLUDE BEGIN */

/* USER INCLUDE END */

/* Exported constants ------------------------------------------------------- */
/* Exported macro ----------------------------------------------------------- */
/* USER DEFINE BEGIN */

/* USER DEFINE END */

/* Exported types ----------------------------------------------------------- */

/* Neutral slots. Board registers hardware; devices never encode RC/PC roles. */
typedef enum {
  BSP_UART_0, BSP_UART_1, BSP_UART_2, BSP_UART_3, BSP_UART_4,
  BSP_UART_5, BSP_UART_6, BSP_UART_7, BSP_UART_8, BSP_UART_9,
  BSP_UART_NUM,
  BSP_UART_ERR,
} BSP_UART_t;

typedef struct {
  const char* name;
  UART_HandleTypeDef* handle;
} BSP_UART_Endpoint_t;

/* Call once at startup, after HAL initialization and before devices/interrupt
 * traffic. Copies the table; names and HAL handles must have static lifetime.
 * Entire table is validated before publication. No runtime rebinding/reset.
 * Array order defines opaque slot IDs; use FindEndpoint instead of assuming IDs. */
int8_t BSP_UART_Configure(const BSP_UART_Endpoint_t* endpoints, uint8_t count);
BSP_UART_t BSP_UART_FindEndpoint(const char* name);

/* UART支持的中断回调函数类型，具体参考HAL中定义 */
typedef enum {
  BSP_UART_TX_HALF_CPLT_CB,
  BSP_UART_TX_CPLT_CB,
  BSP_UART_RX_HALF_CPLT_CB,
  BSP_UART_RX_CPLT_CB,
  BSP_UART_ERROR_CB,
  BSP_UART_ABORT_CPLT_CB,
  BSP_UART_ABORT_TX_CPLT_CB,
  BSP_UART_ABORT_RX_CPLT_CB,

  BSP_UART_IDLE_LINE_CB,
  BSP_UART_CB_NUM,
} BSP_UART_Callback_t;

typedef void (*BSP_UART_RxEventCallback_t)(uint16_t size);

/* Exported functions prototypes -------------------------------------------- */

UART_HandleTypeDef* BSP_UART_GetHandle(BSP_UART_t uart);

/* Claim/release and registration are startup/task operations. Claim rejects any
 * existing callback, preventing silent takeover of another device's UART. */
int8_t BSP_UART_Claim(BSP_UART_t uart, const void* owner);
int8_t BSP_UART_Release(BSP_UART_t uart, const void* owner);
int8_t BSP_UART_RegisterOwnedCallback(BSP_UART_t uart, const void* owner, BSP_UART_Callback_t type,
                                      void (*fn)(void));
int8_t BSP_UART_RegisterOwnedRxEvent(BSP_UART_t uart, const void* owner,
                                     BSP_UART_RxEventCallback_t fn);
void BSP_UART_IRQHandler(UART_HandleTypeDef* huart);

int8_t BSP_UART_RegisterCallback(BSP_UART_t uart, BSP_UART_Callback_t type, void (*callback)(void));
int8_t BSP_UART_RegisterRxEventCallback(BSP_UART_t uart, BSP_UART_RxEventCallback_t callback);

int8_t BSP_UART_Transmit(BSP_UART_t uart, uint8_t* data, uint16_t size, bool dma);
int8_t BSP_UART_Receive(BSP_UART_t uart, uint8_t* data, uint16_t size, bool dma);
int8_t BSP_UART_ReceiveToIdle(BSP_UART_t uart, uint8_t* data, uint16_t size, bool dma);

/* USER FUNCTION BEGIN */

/* USER FUNCTION END */

#ifdef __cplusplus
}
#endif
