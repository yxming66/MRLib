#include <stddef.h>
#include <string.h>

#include "bsp/mrlink/internal.h"
#include "cmsis_compiler.h"

static MrLink_UartBackend_t* s_uart_backends[BSP_UART_NUM];

static int8_t UartDeinit(void* ctx);
static int8_t UartStartRx(void* ctx);
static int8_t UartStartRxInternal(MrLink_UartBackend_t* backend, bool allow_abort);
static uint16_t UartPopRx(void* ctx, uint8_t* dst, uint16_t dst_size);
static int8_t UartSend(void* ctx, const uint8_t* data, uint16_t len);
static int8_t UartRegisterTxCallbacks(void* ctx, MrLink_ChannelNotify_t tx_done,
                                      MrLink_ChannelNotify_t tx_error, void* callback_ctx);
static bool UartIsRxActive(void* ctx);
static uint32_t UartGetRxOverflowCount(const void* ctx);
static uint32_t UartGetRxIrqCount(const void* ctx);
static uint32_t UartGetRxIrqByteCount(const void* ctx);
static uint32_t UartGetRxStartFailCount(const void* ctx);
static void (*UartTxDoneTrampoline(BSP_UART_t uart))(void);
static void (*UartErrorTrampoline(BSP_UART_t uart))(void);
static void (*UartAbortTxTrampoline(BSP_UART_t uart))(void);

static const MrLink_ChannelOps_t kUartOps = {
    .deinit = UartDeinit,
    .start_rx = UartStartRx,
    .pop_rx = UartPopRx,
    .send = UartSend,
    .register_tx_callbacks = UartRegisterTxCallbacks,
    .is_rx_active = UartIsRxActive,
    .get_rx_overflow_count = UartGetRxOverflowCount,
    .get_rx_irq_count = UartGetRxIrqCount,
    .get_rx_irq_byte_count = UartGetRxIrqByteCount,
    .get_rx_start_fail_count = UartGetRxStartFailCount,
};

static MrLink_UartBackend_t* ChannelUart(MrLink_Channel_t* channel) {
  return &channel->backend_storage.uart;
}

static bool UartBackendHasConflict(BSP_UART_t uart, const MrLink_UartBackend_t* backend) {
  if (s_uart_backends[uart] != NULL && s_uart_backends[uart] != backend) {
    return true;
  }

  for (uint8_t i = 0u; i < BSP_UART_NUM; i++) {
    if ((BSP_UART_t)i != uart && s_uart_backends[i] == backend) {
      return true;
    }
  }
  return false;
}

static uint8_t* UartSlotPtr(MrLink_UartBackend_t* backend, uint8_t idx) {
  return &backend->rx_slots[(uint32_t)idx * backend->rx_slot_size];
}

static void UartDropPendingSlot(MrLink_UartBackend_t* backend, uint8_t idx) {
  backend->rx_overflow_count++;
  backend->rx_len[idx] = 0u;
  if (backend->rx_read_idx == idx) {
    backend->rx_read_idx = (uint8_t)((idx + 1u) % backend->rx_slot_count);
    backend->rx_read_offset = 0u;
  }
}

static void UartRxEvent(MrLink_UartBackend_t* backend, uint16_t size) {
  if (backend == NULL) {
    return;
  }

  backend->rx_active = false;
  const uint8_t ready_idx = backend->rx_write_idx;

  if (size > backend->rx_slot_size) {
    backend->rx_start_fail_count++;
    (void)UartStartRxInternal(backend, false);
    return;
  }

  if (size > 0u) {
    backend->rx_len[ready_idx] = size;
    backend->rx_irq_count++;
    backend->rx_irq_byte_count += size;

    const uint8_t next_idx = (uint8_t)((ready_idx + 1u) % backend->rx_slot_count);
    if (backend->rx_len[next_idx] != 0u) {
      UartDropPendingSlot(backend, next_idx);
    }
    backend->rx_write_idx = next_idx;
  }

  if (UartStartRxInternal(backend, false) != BSP_OK) {
    backend->rx_start_fail_count++;
  }

  if (size > 0u && backend->rx_ready != NULL) {
    backend->rx_ready(backend->rx_ready_ctx);
  }
}

#define UART_RX_WRAPPER(Id) \
  static void UartRxEvent##Id(uint16_t n) { UartRxEvent(s_uart_backends[Id], n); }
UART_RX_WRAPPER(BSP_UART_0)
UART_RX_WRAPPER(BSP_UART_1)
UART_RX_WRAPPER(BSP_UART_2)
UART_RX_WRAPPER(BSP_UART_3)
UART_RX_WRAPPER(BSP_UART_4)
UART_RX_WRAPPER(BSP_UART_5)
UART_RX_WRAPPER(BSP_UART_6)
UART_RX_WRAPPER(BSP_UART_7)
UART_RX_WRAPPER(BSP_UART_8)
UART_RX_WRAPPER(BSP_UART_9)
#undef UART_RX_WRAPPER
static BSP_UART_RxEventCallback_t UartRxTrampoline(BSP_UART_t uart) {
  static const BSP_UART_RxEventCallback_t callbacks[BSP_UART_NUM] = {
      UartRxEventBSP_UART_0, UartRxEventBSP_UART_1, UartRxEventBSP_UART_2,
      UartRxEventBSP_UART_3, UartRxEventBSP_UART_4, UartRxEventBSP_UART_5,
      UartRxEventBSP_UART_6, UartRxEventBSP_UART_7, UartRxEventBSP_UART_8,
      UartRxEventBSP_UART_9,
  };
  return (unsigned)uart < BSP_UART_NUM ? callbacks[uart] : NULL;
}

static void UartTxDone(MrLink_UartBackend_t* backend) {
  if (backend != NULL) {
    backend->tx_busy = false;
    backend->tx_abort_pending = false;
  }
  if (backend != NULL && backend->tx_done != NULL) {
    backend->tx_done(backend->tx_callback_ctx);
  }
}

static void UartTxError(MrLink_UartBackend_t* backend) {
  const bool was_busy = (backend != NULL) ? backend->tx_busy : false;
  if (backend != NULL) {
    backend->tx_busy = false;
    backend->tx_abort_pending = false;
  }
  if (was_busy && backend != NULL && backend->tx_error != NULL) {
    backend->tx_error(backend->tx_callback_ctx);
  }
}

static void UartRxErrorRecover(MrLink_UartBackend_t* backend) {
  if (backend == NULL) {
    return;
  }

  backend->rx_active = false;

  UART_HandleTypeDef* huart = BSP_UART_GetHandle(backend->uart);
  if (huart != NULL) {
    __HAL_UART_CLEAR_FLAG(huart,
                          UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
    __HAL_UART_SEND_REQ(huart, UART_RXDATA_FLUSH_REQUEST);
  }

  if (UartStartRxInternal(backend, false) != BSP_OK) {
    backend->rx_start_fail_count++;
  }
}

static void UartError(MrLink_UartBackend_t* backend) {
  if (!backend) return;
  UART_HandleTypeDef* huart = BSP_UART_GetHandle(backend->uart);
  /* Defer blocking HAL cancellation to the task. A receive-only error must
   * never release a buffer still owned by TX DMA. */
  if (backend->tx_busy && huart && (huart->ErrorCode & HAL_UART_ERROR_DMA) != 0u)
    backend->tx_abort_pending = true;
  UartRxErrorRecover(backend);
}

#define UART_TX_WRAPPERS(Id) \
  static void UartTxDone##Id(void) { UartTxDone(s_uart_backends[Id]); } \
  static void UartError##Id(void) { UartError(s_uart_backends[Id]); } \
  static void UartAbortTx##Id(void) { UartTxError(s_uart_backends[Id]); }
UART_TX_WRAPPERS(BSP_UART_0)
UART_TX_WRAPPERS(BSP_UART_1)
UART_TX_WRAPPERS(BSP_UART_2)
UART_TX_WRAPPERS(BSP_UART_3)
UART_TX_WRAPPERS(BSP_UART_4)
UART_TX_WRAPPERS(BSP_UART_5)
UART_TX_WRAPPERS(BSP_UART_6)
UART_TX_WRAPPERS(BSP_UART_7)
UART_TX_WRAPPERS(BSP_UART_8)
UART_TX_WRAPPERS(BSP_UART_9)
#undef UART_TX_WRAPPERS
static void (*UartAbortTxTrampoline(BSP_UART_t uart))(void) {
  static void (*const callbacks[BSP_UART_NUM])(void) = {
      UartAbortTxBSP_UART_0, UartAbortTxBSP_UART_1, UartAbortTxBSP_UART_2,
      UartAbortTxBSP_UART_3, UartAbortTxBSP_UART_4, UartAbortTxBSP_UART_5,
      UartAbortTxBSP_UART_6, UartAbortTxBSP_UART_7, UartAbortTxBSP_UART_8,
      UartAbortTxBSP_UART_9,
  };
  return (unsigned)uart < BSP_UART_NUM ? callbacks[uart] : NULL;
}

static void (*UartTxDoneTrampoline(BSP_UART_t uart))(void) {
  static void (*const callbacks[BSP_UART_NUM])(void) = {
      UartTxDoneBSP_UART_0, UartTxDoneBSP_UART_1, UartTxDoneBSP_UART_2,
      UartTxDoneBSP_UART_3, UartTxDoneBSP_UART_4, UartTxDoneBSP_UART_5,
      UartTxDoneBSP_UART_6, UartTxDoneBSP_UART_7, UartTxDoneBSP_UART_8,
      UartTxDoneBSP_UART_9,
  };
  return (unsigned)uart < BSP_UART_NUM ? callbacks[uart] : NULL;
}

static void (*UartErrorTrampoline(BSP_UART_t uart))(void) {
  static void (*const callbacks[BSP_UART_NUM])(void) = {
      UartErrorBSP_UART_0, UartErrorBSP_UART_1, UartErrorBSP_UART_2,
      UartErrorBSP_UART_3, UartErrorBSP_UART_4, UartErrorBSP_UART_5,
      UartErrorBSP_UART_6, UartErrorBSP_UART_7, UartErrorBSP_UART_8,
      UartErrorBSP_UART_9,
  };
  return (unsigned)uart < BSP_UART_NUM ? callbacks[uart] : NULL;
}

int8_t MrLink_ChannelBackend_InitUart(MrLink_Channel_t* channel,
                                      const MrLink_ChannelUartConfig_t* config) {
  if (channel == NULL || config == NULL) {
    return MRLINK_CHANNEL_ERR_NULL;
  }
  if (config->rx_slots == NULL || config->rx_len_storage == NULL) {
    return MRLINK_CHANNEL_ERR_NULL;
  }
  if ((uint32_t)config->uart >= (uint32_t)BSP_UART_NUM || config->rx_slot_count < 2u ||
      config->rx_slot_size == 0u) {
    return MRLINK_CHANNEL_ERR;
  }

  MrLink_UartBackend_t* backend = ChannelUart(channel);
  if (UartBackendHasConflict(config->uart, backend)) {
    return MRLINK_CHANNEL_ERR;
  }

  if (UartRxTrampoline(config->uart) == NULL || UartTxDoneTrampoline(config->uart) == NULL ||
      UartErrorTrampoline(config->uart) == NULL || UartAbortTxTrampoline(config->uart) == NULL) {
    return MRLINK_CHANNEL_ERR_UNSUPPORTED;
  }

  if (BSP_UART_Claim(config->uart, backend) != BSP_OK) return MRLINK_CHANNEL_ERR_BUSY;
  memset(channel, 0, sizeof(*channel));

  backend = ChannelUart(channel);
  backend->uart = config->uart;
  backend->rx_slots = config->rx_slots;
  backend->rx_slot_size = config->rx_slot_size;
  backend->rx_slot_count = config->rx_slot_count;
  backend->rx_len = config->rx_len_storage;
  backend->rx_ready = config->rx_ready;
  backend->rx_ready_ctx = config->rx_ready_ctx;

  for (uint8_t i = 0u; i < config->rx_slot_count; i++) {
    config->rx_len_storage[i] = 0u;
  }

  s_uart_backends[config->uart] = backend;

  int8_t status =
      BSP_UART_RegisterOwnedRxEvent(config->uart, backend, UartRxTrampoline(config->uart));
  if (status == BSP_OK)
    status = BSP_UART_RegisterOwnedCallback(config->uart, backend, BSP_UART_ERROR_CB,
                                            UartErrorTrampoline(config->uart));
  if (status == BSP_OK)
    status = BSP_UART_RegisterOwnedCallback(config->uart, backend, BSP_UART_TX_CPLT_CB,
                                            UartTxDoneTrampoline(config->uart));
  if (status == BSP_OK)
    status = BSP_UART_RegisterOwnedCallback(config->uart, backend, BSP_UART_ABORT_TX_CPLT_CB,
                                            UartAbortTxTrampoline(config->uart));
  if (status == BSP_OK)
    status = BSP_UART_RegisterOwnedCallback(config->uart, backend, BSP_UART_ABORT_CPLT_CB,
                                            UartAbortTxTrampoline(config->uart));
  if (status != BSP_OK) {
    (void)BSP_UART_Release(config->uart, backend);
    s_uart_backends[config->uart] = NULL;
    memset(channel, 0, sizeof(*channel));
    return status;
  }
  MrLink_Channel_SetBackend(channel, MRLINK_CHANNEL_HW_UART, &kUartOps);
  return BSP_OK;
}

static int8_t UartStartRx(void* ctx) {
  MrLink_UartBackend_t* backend = (MrLink_UartBackend_t*)ctx;
  if (backend != NULL && backend->rx_active) return BSP_OK;
  return UartStartRxInternal(backend, true);
}

static int8_t UartStartRxInternal(MrLink_UartBackend_t* backend, bool allow_abort) {
  if (backend == NULL) {
    return MRLINK_CHANNEL_ERR_NULL;
  }

  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  const uint8_t idx = backend->rx_write_idx;
  if (backend->rx_len[idx] != 0u) UartDropPendingSlot(backend, idx);
  int8_t status =
      BSP_UART_ReceiveToIdle(backend->uart, UartSlotPtr(backend, idx), backend->rx_slot_size, true);
  UART_HandleTypeDef* h = BSP_UART_GetHandle(backend->uart);
  if (status != BSP_OK && allow_abort && h && HAL_UART_AbortReceive(h) == HAL_OK)
    status = BSP_UART_ReceiveToIdle(backend->uart, UartSlotPtr(backend, idx), backend->rx_slot_size,
                                    true);
  backend->rx_active = status == BSP_OK;
  if (!token) __enable_irq();
  return status;
}

static void UartServiceTx(MrLink_UartBackend_t* backend) {
  if (backend->tx_abort_pending) {
    UART_HandleTypeDef* h = BSP_UART_GetHandle(backend->uart);
    if (h && HAL_UART_AbortTransmit(h) == HAL_OK) UartTxError(backend);
  }
}

static uint16_t UartPopRx(void* ctx, uint8_t* dst, uint16_t dst_size) {
  MrLink_UartBackend_t* backend = (MrLink_UartBackend_t*)ctx;
  if (backend == NULL || dst == NULL || dst_size == 0u) {
    return 0u;
  }

  UartServiceTx(backend);
  if (!backend->rx_active && UartStartRx(backend) != BSP_OK) backend->rx_start_fail_count++;
  uint16_t len = 0u;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const uint8_t idx = backend->rx_read_idx;
  len = backend->rx_len[idx];
  if (len > 0u) {
    const uint16_t remaining =
        (len > backend->rx_read_offset) ? (uint16_t)(len - backend->rx_read_offset) : 0u;
    len = (remaining > dst_size) ? dst_size : remaining;
    if (len > 0u) {
      memcpy(dst, UartSlotPtr(backend, idx) + backend->rx_read_offset, len);
      backend->rx_read_offset = (uint16_t)(backend->rx_read_offset + len);
    }
    if (backend->rx_read_offset >= backend->rx_len[idx]) {
      backend->rx_len[idx] = 0u;
      backend->rx_read_offset = 0u;
      backend->rx_read_idx = (uint8_t)((idx + 1u) % backend->rx_slot_count);
    }
  }
  if (primask == 0u) {
    __enable_irq();
  }
  return len;
}

static int8_t UartSend(void* ctx, const uint8_t* data, uint16_t len) {
  MrLink_UartBackend_t* backend = (MrLink_UartBackend_t*)ctx;
  if (backend == NULL) {
    return MRLINK_CHANNEL_ERR_NULL;
  }
  if (!data || !len || len > MRLINK_CHANNEL_TX_BUFFER_SIZE) return MRLINK_CHANNEL_ERR;
  UartServiceTx(backend);
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  if (backend->tx_busy) {
    if (token == 0u) __enable_irq();
    return MRLINK_CHANNEL_ERR_BUSY;
  }
  memcpy(backend->tx_buf, data, len);
  backend->tx_busy = true;
  const int8_t ret = BSP_UART_Transmit(backend->uart, backend->tx_buf, len, true);
  if (ret != BSP_OK) backend->tx_busy = false;
  if (token == 0u) __enable_irq();
  return ret;
}

static int8_t UartRegisterTxCallbacks(void* ctx, MrLink_ChannelNotify_t tx_done,
                                      MrLink_ChannelNotify_t tx_error, void* callback_ctx) {
  MrLink_UartBackend_t* backend = (MrLink_UartBackend_t*)ctx;
  if (backend == NULL) {
    return MRLINK_CHANNEL_ERR_NULL;
  }

  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  backend->tx_done = tx_done;
  backend->tx_error = tx_error;
  backend->tx_callback_ctx = callback_ctx;
  if (token == 0u) __enable_irq();
  return MRLINK_CHANNEL_OK;
}

static int8_t UartDeinit(void* ctx) {
  MrLink_UartBackend_t* backend = ctx;
  if (!backend) return MRLINK_CHANNEL_ERR_NULL;
  UART_HandleTypeDef* huart = BSP_UART_GetHandle(backend->uart);
  if (!huart) return MRLINK_CHANNEL_ERR;
  /* Blocking HAL aborts complete before callback pointers or DMA storage expire. */
  if (HAL_UART_AbortReceive(huart) != HAL_OK) return MRLINK_CHANNEL_ERR_BUSY;
  backend->rx_active = false;
  if (HAL_UART_AbortTransmit(huart) != HAL_OK) return MRLINK_CHANNEL_ERR_BUSY;
  backend->tx_busy = false;
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  const int8_t status = BSP_UART_Release(backend->uart, backend);
  if (status == BSP_OK) s_uart_backends[backend->uart] = NULL;
  if (token == 0u) __enable_irq();
  return status;
}

static bool UartIsRxActive(void* ctx) {
  const MrLink_UartBackend_t* backend = (const MrLink_UartBackend_t*)ctx;
  return (backend != NULL) ? backend->rx_active : false;
}

static uint32_t UartGetRxOverflowCount(const void* ctx) {
  const MrLink_UartBackend_t* backend = (const MrLink_UartBackend_t*)ctx;
  return (backend != NULL) ? backend->rx_overflow_count : 0u;
}

static uint32_t UartGetRxIrqCount(const void* ctx) {
  const MrLink_UartBackend_t* backend = (const MrLink_UartBackend_t*)ctx;
  return (backend != NULL) ? backend->rx_irq_count : 0u;
}

static uint32_t UartGetRxIrqByteCount(const void* ctx) {
  const MrLink_UartBackend_t* backend = (const MrLink_UartBackend_t*)ctx;
  return (backend != NULL) ? backend->rx_irq_byte_count : 0u;
}

static uint32_t UartGetRxStartFailCount(const void* ctx) {
  const MrLink_UartBackend_t* backend = (const MrLink_UartBackend_t*)ctx;
  return (backend != NULL) ? backend->rx_start_fail_count : 0u;
}
