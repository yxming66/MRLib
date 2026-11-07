/* Includes ----------------------------------------------------------------- */
#include "bsp/uart.h"

#include <string.h>

#include "bsp/dma.h"

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
static void (*UART_Callback[BSP_UART_NUM][BSP_UART_CB_NUM])(void);
static BSP_UART_RxEventCallback_t UART_RxEventCallback[BSP_UART_NUM];
static const void* UART_Owner[BSP_UART_NUM];
static uint8_t* UART_DmaRx[BSP_UART_NUM];
static uint16_t UART_DmaRxSize[BSP_UART_NUM];
static BSP_UART_Endpoint_t UART_Endpoints[BSP_UART_NUM];
static uint8_t UART_EndpointCount;
static bool UART_Configured;

int8_t BSP_UART_Configure(const BSP_UART_Endpoint_t* endpoints, uint8_t count) {
  if (UART_Configured || !endpoints || !count || count > BSP_UART_NUM) return BSP_ERR;
  for (uint8_t i = 0; i < count; ++i) {
    const BSP_UART_Endpoint_t* endpoint = &endpoints[i];
    if (!endpoint->name || !endpoint->name[0] || !endpoint->handle ||
        !endpoint->handle->Instance || endpoint->handle->RxState != HAL_UART_STATE_READY ||
        endpoint->handle->gState != HAL_UART_STATE_READY) return BSP_ERR;
    for (uint8_t j = 0; j < i; ++j)
      if (strcmp(endpoint->name, endpoints[j].name) == 0 ||
          endpoint->handle->Instance == endpoints[j].handle->Instance) return BSP_ERR;
  }
  memcpy(UART_Endpoints, endpoints, count * sizeof(*endpoints));
  UART_EndpointCount = count;
  UART_Configured = true;
  return BSP_OK;
}

BSP_UART_t BSP_UART_FindEndpoint(const char* name) {
  if (name)
    for (uint8_t i = 0; i < UART_EndpointCount; ++i)
      if (strcmp(name, UART_Endpoints[i].name) == 0) return (BSP_UART_t)i;
  return BSP_UART_ERR;
}
static bool UART_CompleteDma(BSP_UART_t uart, uint16_t size, bool final) {
  uint8_t* data = UART_DmaRx[uart];
  if (!data) return true;
  const uint16_t capacity = UART_DmaRxSize[uart];
  if (final) {
    UART_DmaRx[uart] = NULL;
    UART_DmaRxSize[uart] = 0;
  }
  if (!size || size > capacity) {
    if (UART_Callback[uart][BSP_UART_ERROR_CB]) UART_Callback[uart][BSP_UART_ERROR_CB]();
    return false;
  }
  if (BSP_DMA_Synchronize(BSP_DMA_COMPLETE_RX, data, size)) return true;
  if (UART_Callback[uart][BSP_UART_ERROR_CB]) UART_Callback[uart][BSP_UART_ERROR_CB]();
  return false;
}

/* Private function  -------------------------------------------------------- */
static BSP_UART_t UART_Get(UART_HandleTypeDef* huart) {
  if (huart)
    for (uint8_t i = 0; i < UART_EndpointCount; ++i)
      if (huart == UART_Endpoints[i].handle) return (BSP_UART_t)i;
  return BSP_UART_ERR;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  BSP_UART_t bsp_uart = UART_Get(huart);
  if (bsp_uart != BSP_UART_ERR) {
    if (UART_Callback[bsp_uart][BSP_UART_TX_CPLT_CB]) {
      UART_Callback[bsp_uart][BSP_UART_TX_CPLT_CB]();
    }
  }
}

void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef* huart) {
  BSP_UART_t bsp_uart = UART_Get(huart);
  if (bsp_uart != BSP_UART_ERR) {
    if (UART_Callback[bsp_uart][BSP_UART_TX_HALF_CPLT_CB]) {
      UART_Callback[bsp_uart][BSP_UART_TX_HALF_CPLT_CB]();
    }
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef* huart) {
  BSP_UART_t bsp_uart = UART_Get(huart);
  if (bsp_uart != BSP_UART_ERR && !UART_CompleteDma(bsp_uart, UART_DmaRxSize[bsp_uart], true))
    return;
  if (bsp_uart != BSP_UART_ERR) {
    if (UART_Callback[bsp_uart][BSP_UART_RX_CPLT_CB]) {
      UART_Callback[bsp_uart][BSP_UART_RX_CPLT_CB]();
    }
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
  BSP_UART_t bsp_uart = UART_Get(huart);
  if (bsp_uart != BSP_UART_ERR && !UART_CompleteDma(bsp_uart, Size, true)) return;
  if (bsp_uart != BSP_UART_ERR) {
    if (UART_RxEventCallback[bsp_uart]) {
      UART_RxEventCallback[bsp_uart](Size);
    } else if (UART_Callback[bsp_uart][BSP_UART_RX_CPLT_CB]) {
      UART_Callback[bsp_uart][BSP_UART_RX_CPLT_CB]();
    }
  }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef* huart) {
  BSP_UART_t bsp_uart = UART_Get(huart);
  if (bsp_uart != BSP_UART_ERR && UART_DmaRx[bsp_uart] &&
      !UART_CompleteDma(bsp_uart, UART_DmaRxSize[bsp_uart] / 2u, false))
    return;
  if (bsp_uart != BSP_UART_ERR) {
    if (UART_Callback[bsp_uart][BSP_UART_RX_HALF_CPLT_CB]) {
      UART_Callback[bsp_uart][BSP_UART_RX_HALF_CPLT_CB]();
    }
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef* huart) {
  BSP_UART_t bsp_uart = UART_Get(huart);
  if (bsp_uart != BSP_UART_ERR) {
    if (UART_Callback[bsp_uart][BSP_UART_ERROR_CB]) {
      UART_Callback[bsp_uart][BSP_UART_ERROR_CB]();
    }
  }
}

void HAL_UART_AbortCpltCallback(UART_HandleTypeDef* huart) {
  BSP_UART_t bsp_uart = UART_Get(huart);
  if (bsp_uart != BSP_UART_ERR) {
    UART_DmaRx[bsp_uart] = NULL;
    UART_DmaRxSize[bsp_uart] = 0;
  }
  if (bsp_uart != BSP_UART_ERR) {
    if (UART_Callback[bsp_uart][BSP_UART_ABORT_CPLT_CB]) {
      UART_Callback[bsp_uart][BSP_UART_ABORT_CPLT_CB]();
    }
  }
}

void HAL_UART_AbortTransmitCpltCallback(UART_HandleTypeDef* huart) {
  BSP_UART_t bsp_uart = UART_Get(huart);
  if (bsp_uart != BSP_UART_ERR) {
    if (UART_Callback[bsp_uart][BSP_UART_ABORT_TX_CPLT_CB]) {
      UART_Callback[bsp_uart][BSP_UART_ABORT_TX_CPLT_CB]();
    }
  }
}

void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef* huart) {
  BSP_UART_t bsp_uart = UART_Get(huart);
  if (bsp_uart != BSP_UART_ERR) {
    UART_DmaRx[bsp_uart] = NULL;
    UART_DmaRxSize[bsp_uart] = 0;
  }
  if (bsp_uart != BSP_UART_ERR) {
    if (UART_Callback[bsp_uart][BSP_UART_ABORT_RX_CPLT_CB]) {
      UART_Callback[bsp_uart][BSP_UART_ABORT_RX_CPLT_CB]();
    }
  }
}

/* Exported functions ------------------------------------------------------- */
void BSP_UART_IRQHandler(UART_HandleTypeDef* huart) {
  if (huart == NULL) return;
  const BSP_UART_t uart = UART_Get(huart);
  if (uart == BSP_UART_ERR) return;
  if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    if (UART_Callback[uart][BSP_UART_IDLE_LINE_CB]) {
      UART_Callback[uart][BSP_UART_IDLE_LINE_CB]();
    }
  }
}

UART_HandleTypeDef* BSP_UART_GetHandle(BSP_UART_t uart) {
  return (unsigned)uart < UART_EndpointCount ? UART_Endpoints[uart].handle : NULL;
}

int8_t BSP_UART_RegisterCallback(BSP_UART_t uart, BSP_UART_Callback_t type,
                                 void (*callback)(void)) {
  if ((unsigned)uart >= BSP_UART_NUM || (unsigned)type >= BSP_UART_CB_NUM) return BSP_ERR;
  if (!BSP_UART_GetHandle(uart)) return BSP_ERR;
  if (UART_Owner[uart]) return BSP_ERR;
  if (callback == NULL) {
    UART_Callback[uart][type] = NULL;
    return BSP_OK;
  }
  UART_Callback[uart][type] = callback;
  return BSP_OK;
}

int8_t BSP_UART_RegisterRxEventCallback(BSP_UART_t uart, BSP_UART_RxEventCallback_t callback) {
  if ((unsigned)uart >= BSP_UART_NUM) return BSP_ERR;
  if (!BSP_UART_GetHandle(uart)) return BSP_ERR;
  if (UART_Owner[uart]) return BSP_ERR;
  UART_RxEventCallback[uart] = callback;
  return BSP_OK;
}

int8_t BSP_UART_Transmit(BSP_UART_t uart, uint8_t* data, uint16_t size, bool dma) {
  if ((unsigned)uart >= BSP_UART_NUM || !data || !size) return BSP_ERR;
  UART_HandleTypeDef* h = BSP_UART_GetHandle(uart);
  if (!h) return BSP_ERR;
  if (!dma) return HAL_UART_Transmit_IT(h, data, size);
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  int8_t status = BSP_ERR;
  if (h->gState == HAL_UART_STATE_READY && BSP_DMA_Synchronize(BSP_DMA_PREPARE_TX, data, size))
    status = (int8_t)HAL_UART_Transmit_DMA(h, data, size);
  if (!token) __enable_irq();
  return status;
}
static int8_t UART_StartReceive(BSP_UART_t uart, uint8_t* data, uint16_t size, bool dma,
                                bool idle) {
  if ((unsigned)uart >= BSP_UART_NUM || !data || !size) return BSP_ERR;
  UART_HandleTypeDef* h = BSP_UART_GetHandle(uart);
  if (!h || (dma && (!h->hdmarx || h->hdmarx->Init.Mode != DMA_NORMAL))) return BSP_ERR;
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  int8_t status = BSP_ERR;
  if (h->RxState == HAL_UART_STATE_READY &&
      (!dma || BSP_DMA_Synchronize(BSP_DMA_PREPARE_RX, data, size))) {
    if (dma)
      status = (int8_t)(idle ? HAL_UARTEx_ReceiveToIdle_DMA(h, data, size)
                             : HAL_UART_Receive_DMA(h, data, size));
    else
      status = (int8_t)(idle ? HAL_UARTEx_ReceiveToIdle_IT(h, data, size)
                             : HAL_UART_Receive_IT(h, data, size));
    if (status == HAL_OK) {
      UART_DmaRx[uart] = dma ? data : NULL;
      UART_DmaRxSize[uart] = dma ? size : 0;
      if (dma && idle) __HAL_DMA_DISABLE_IT(h->hdmarx, DMA_IT_HT);
    }
  }
  if (!token) __enable_irq();
  return status;
}
int8_t BSP_UART_Receive(BSP_UART_t uart, uint8_t* data, uint16_t size, bool dma) {
  return UART_StartReceive(uart, data, size, dma, false);
}
int8_t BSP_UART_ReceiveToIdle(BSP_UART_t uart, uint8_t* data, uint16_t size, bool dma) {
  return UART_StartReceive(uart, data, size, dma, true);
}

/* USER FUNCTION BEGIN */

/* USER FUNCTION END */

int8_t BSP_UART_Claim(BSP_UART_t uart, const void* owner) {
  if ((unsigned)uart >= BSP_UART_NUM || !owner) return BSP_ERR;
  if (UART_Owner[uart] || UART_RxEventCallback[uart]) return BSP_ERR;
  UART_HandleTypeDef* h = BSP_UART_GetHandle(uart);
  if (!h || h->RxState != HAL_UART_STATE_READY || h->gState != HAL_UART_STATE_READY) return BSP_ERR;
  for (unsigned i = 0; i < BSP_UART_CB_NUM; ++i)
    if (UART_Callback[uart][i]) return BSP_ERR;
  UART_Owner[uart] = owner;
  return BSP_OK;
}
int8_t BSP_UART_Release(BSP_UART_t uart, const void* owner) {
  if ((unsigned)uart >= BSP_UART_NUM || !owner || UART_Owner[uart] != owner) return BSP_ERR;
  for (unsigned i = 0; i < BSP_UART_CB_NUM; ++i) UART_Callback[uart][i] = NULL;
  UART_RxEventCallback[uart] = NULL;
  UART_DmaRx[uart] = NULL;
  UART_DmaRxSize[uart] = 0;
  UART_Owner[uart] = NULL;
  return BSP_OK;
}
int8_t BSP_UART_RegisterOwnedCallback(BSP_UART_t uart, const void* owner, BSP_UART_Callback_t type,
                                      void (*fn)(void)) {
  if ((unsigned)uart >= BSP_UART_NUM || (unsigned)type >= BSP_UART_CB_NUM || !owner ||
      UART_Owner[uart] != owner)
    return BSP_ERR;
  UART_Callback[uart][type] = fn;
  return BSP_OK;
}
int8_t BSP_UART_RegisterOwnedRxEvent(BSP_UART_t uart, const void* owner,
                                     BSP_UART_RxEventCallback_t fn) {
  if ((unsigned)uart >= BSP_UART_NUM || !owner || UART_Owner[uart] != owner) return BSP_ERR;
  UART_RxEventCallback[uart] = fn;
  return BSP_OK;
}
