#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#include "bsp/fdcan.h"
#include "bsp/uart.h"

#define MRLINK_CHANNEL_OK (0)
#define MRLINK_CHANNEL_ERR (-1)
#define MRLINK_CHANNEL_ERR_NULL (-2)
#define MRLINK_CHANNEL_ERR_UNSUPPORTED (-3)

#define MRLINK_CHANNEL_TX_BUFFER_SIZE (96u)
#define MRLINK_CHANNEL_FDCAN_MAX_FRAME_SIZE (64u)
#define MRLINK_CHANNEL_ERR_BUSY (-4)
typedef void (*MrLink_ChannelNotify_t)(void* ctx);
typedef enum {
  MRLINK_CHANNEL_HW_NONE = 0,
  MRLINK_CHANNEL_HW_UART = 1,
  MRLINK_CHANNEL_HW_USB = 2,
  MRLINK_CHANNEL_HW_FDCAN = 3
} MrLink_ChannelHardware_t;
typedef struct {
  BSP_UART_t uart;
  uint8_t* rx_slots;
  uint16_t rx_slot_size;
  uint8_t rx_slot_count;
  volatile uint16_t* rx_len_storage;
  MrLink_ChannelNotify_t rx_ready;
  void* rx_ready_ctx;
} MrLink_ChannelUartConfig_t;

typedef struct {
  BSP_FDCAN_t fdcan;
  BSP_FDCAN_Format_t tx_format;
  uint32_t tx_id;
  uint32_t rx_id;
  uint8_t rx_queue_size;
  uint8_t max_frame_size;
} MrLink_ChannelFdcanConfig_t;

typedef int8_t (*MrLink_ChannelUsbStartRx_t)(void* ctx);
typedef uint16_t (*MrLink_ChannelUsbPopRx_t)(void* ctx, uint8_t* dst, uint16_t dst_size);
/* Nonblocking submit; failure retains no pointer. Callback registration is
 * transactional; a reported TX done/error means DMA no longer owns the buffer. */
typedef int8_t (*MrLink_ChannelUsbSend_t)(void* ctx, uint8_t* data, uint16_t len);
typedef int8_t (*MrLink_ChannelUsbRegisterTxCallbacks_t)(void* ctx, MrLink_ChannelNotify_t tx_done,
                                                         MrLink_ChannelNotify_t tx_error,
                                                         void* callback_ctx);
typedef bool (*MrLink_ChannelUsbIsRxActive_t)(void* ctx);
/* Stop must synchronously cancel DMA and detach callbacks before returning OK. */
typedef int8_t (*MrLink_ChannelUsbStop_t)(void* ctx);

typedef struct {
  void* ctx;
  MrLink_ChannelUsbStartRx_t start_rx;
  MrLink_ChannelUsbPopRx_t pop_rx;
  MrLink_ChannelUsbSend_t send;
  MrLink_ChannelUsbRegisterTxCallbacks_t register_tx_callbacks;
  MrLink_ChannelUsbIsRxActive_t is_rx_active;
  MrLink_ChannelUsbStop_t stop;
  bool synchronous_send; /* true: send copies/consumes before return */
} MrLink_ChannelUsbConfig_t;

#ifdef __cplusplus
#define MRLINK_CHANNEL_ALIGN alignas(32)
#else
#define MRLINK_CHANNEL_ALIGN _Alignas(32)
#endif
/* Actual layouts, with no casts from an unrelated byte-array object. */
typedef struct {
  BSP_UART_t uart;
  uint8_t* rx_slots;
  uint16_t rx_slot_size;
  uint8_t rx_slot_count;
  volatile bool rx_active;
  volatile uint8_t rx_write_idx, rx_read_idx;
  volatile uint16_t rx_read_offset;
  volatile uint16_t* rx_len;
  MrLink_ChannelNotify_t rx_ready;
  void* rx_ready_ctx;
  MrLink_ChannelNotify_t tx_done, tx_error;
  void* tx_callback_ctx;
  volatile bool tx_busy, tx_abort_pending;
  uint32_t rx_overflow_count, rx_irq_count, rx_irq_byte_count, rx_start_fail_count;
  MRLINK_CHANNEL_ALIGN uint8_t tx_buf[MRLINK_CHANNEL_TX_BUFFER_SIZE];
} MrLink_UartBackend_t;
typedef struct {
  BSP_FDCAN_t fdcan;
  BSP_FDCAN_Format_t tx_format;
  uint32_t tx_id, rx_id;
  uint8_t rx_queue_size, max_frame_size;
  bool rx_active;
  MrLink_ChannelNotify_t tx_done, tx_error;
  void* tx_callback_ctx;
  uint32_t rx_irq_count, rx_irq_byte_count, rx_overflow_count, rx_start_fail_count;
  uint8_t pending[MRLINK_CHANNEL_FDCAN_MAX_FRAME_SIZE];
  uint16_t pending_size, pending_offset;
} MrLink_FdcanBackend_t;
typedef struct {
  MrLink_ChannelUsbConfig_t config;
  volatile bool tx_busy;
  MrLink_ChannelNotify_t tx_done, tx_error;
  void* tx_callback_ctx;
  MRLINK_CHANNEL_ALIGN uint8_t tx_buf[MRLINK_CHANNEL_TX_BUFFER_SIZE];
} MrLink_UsbBackend_t;
typedef union {
  MrLink_UartBackend_t uart;
  MrLink_FdcanBackend_t fdcan;
  MrLink_UsbBackend_t usb;
} MrLink_ChannelBackendStorage_t;
typedef struct {
  MrLink_ChannelHardware_t hardware;
  const void* backend_ops;
  MrLink_ChannelBackendStorage_t backend_storage;
} MrLink_Channel_t;
#undef MRLINK_CHANNEL_ALIGN

/* Zero-initialize before Init. No copying/moving. Deinit in task context before
 * storage expires, while the application consumer is stopped. A failed Deinit
 * leaves the object and DMA buffers alive: retry after stopping hardware.
 * Init rejects an already initialized channel; successful Send always copies
 * input before return, including async UART/USB. TX notifications are short signals in ISR
 * (UART/async USB) or task context (FDCAN/TX recovery), never blocking business work. UART resource
 * callbacks are exclusively claimed. */
int8_t MrLink_Channel_Deinit(MrLink_Channel_t* channel);

int8_t MrLink_Channel_InitUart(MrLink_Channel_t* channel, const MrLink_ChannelUartConfig_t* config);
int8_t MrLink_Channel_InitFdcan(MrLink_Channel_t* channel,
                                const MrLink_ChannelFdcanConfig_t* config);
int8_t MrLink_Channel_InitUsb(MrLink_Channel_t* channel, const MrLink_ChannelUsbConfig_t* config);

MrLink_ChannelHardware_t MrLink_Channel_GetHardware(const MrLink_Channel_t* channel);

int8_t MrLink_Channel_StartRx(const MrLink_Channel_t* channel);
uint16_t MrLink_Channel_PopRx(const MrLink_Channel_t* channel, uint8_t* dst, uint16_t dst_size);
int8_t MrLink_Channel_Send(const MrLink_Channel_t* channel, const uint8_t* data, uint16_t len);
int8_t MrLink_Channel_RegisterTxCallbacks(const MrLink_Channel_t* channel,
                                          MrLink_ChannelNotify_t tx_done,
                                          MrLink_ChannelNotify_t tx_error, void* callback_ctx);
bool MrLink_Channel_IsRxActive(const MrLink_Channel_t* channel);

uint32_t MrLink_Channel_GetRxOverflowCount(const MrLink_Channel_t* channel);
uint32_t MrLink_Channel_GetRxIrqCount(const MrLink_Channel_t* channel);
uint32_t MrLink_Channel_GetRxIrqByteCount(const MrLink_Channel_t* channel);
uint32_t MrLink_Channel_GetRxStartFailCount(const MrLink_Channel_t* channel);

#ifdef __cplusplus
}
#endif
