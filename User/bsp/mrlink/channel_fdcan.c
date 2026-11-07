#include <stddef.h>
#include <string.h>

#include "bsp/mrlink/internal.h"

static int8_t FdcanDeinit(void* ctx);
static int8_t FdcanStartRx(void* ctx);
static uint16_t FdcanPopRx(void* ctx, uint8_t* dst, uint16_t dst_size);
static int8_t FdcanSend(void* ctx, const uint8_t* data, uint16_t len);
static int8_t FdcanRegisterTxCallbacks(void* ctx, MrLink_ChannelNotify_t tx_done,
                                       MrLink_ChannelNotify_t tx_error, void* callback_ctx);
static bool FdcanIsRxActive(void* ctx);
static uint32_t FdcanGetRxOverflowCount(const void* ctx);
static uint32_t FdcanGetRxIrqCount(const void* ctx);
static uint32_t FdcanGetRxIrqByteCount(const void* ctx);
static uint32_t FdcanGetRxStartFailCount(const void* ctx);

static const MrLink_ChannelOps_t kFdcanOps = {
    .deinit = FdcanDeinit,
    .start_rx = FdcanStartRx,
    .pop_rx = FdcanPopRx,
    .send = FdcanSend,
    .register_tx_callbacks = FdcanRegisterTxCallbacks,
    .is_rx_active = FdcanIsRxActive,
    .get_rx_overflow_count = FdcanGetRxOverflowCount,
    .get_rx_irq_count = FdcanGetRxIrqCount,
    .get_rx_irq_byte_count = FdcanGetRxIrqByteCount,
    .get_rx_start_fail_count = FdcanGetRxStartFailCount,
};

static MrLink_FdcanBackend_t* ChannelFdcan(MrLink_Channel_t* channel) {
  return &channel->backend_storage.fdcan;
}

static bool FdcanFormatIsData(BSP_FDCAN_Format_t format) {
  return format == BSP_FDCAN_FORMAT_STD_DATA || format == BSP_FDCAN_FORMAT_EXT_DATA;
}

int8_t MrLink_ChannelBackend_InitFdcan(MrLink_Channel_t* channel,
                                       const MrLink_ChannelFdcanConfig_t* config) {
  if (channel == NULL || config == NULL) {
    return MRLINK_CHANNEL_ERR_NULL;
  }
  if ((unsigned)config->fdcan >= BSP_FDCAN_NUM || !FdcanFormatIsData(config->tx_format)) {
    return MRLINK_CHANNEL_ERR;
  }

  const uint32_t limit = config->tx_format == BSP_FDCAN_FORMAT_STD_DATA ? 0x7ffu : 0x1fffffffu;
  if (config->tx_id > limit || config->rx_id > limit) return MRLINK_CHANNEL_ERR;
  const uint8_t max_frame_size =
      (config->max_frame_size > 0u) ? config->max_frame_size : MRLINK_CHANNEL_FDCAN_MAX_FRAME_SIZE;
  if (max_frame_size > MRLINK_CHANNEL_FDCAN_MAX_FRAME_SIZE) {
    return MRLINK_CHANNEL_ERR;
  }

  memset(channel, 0, sizeof(*channel));
  MrLink_FdcanBackend_t* backend = ChannelFdcan(channel);
  backend->fdcan = config->fdcan;
  backend->tx_format = config->tx_format;
  backend->tx_id = config->tx_id;
  backend->rx_id = config->rx_id;
  backend->rx_queue_size = config->rx_queue_size;
  backend->max_frame_size = max_frame_size;

  MrLink_Channel_SetBackend(channel, MRLINK_CHANNEL_HW_FDCAN, &kFdcanOps);
  const int8_t status = FdcanStartRx(backend);
  if (status != BSP_OK) memset(channel, 0, sizeof(*channel));
  return status;
}

static int8_t FdcanStartRx(void* ctx) {
  MrLink_FdcanBackend_t* backend = (MrLink_FdcanBackend_t*)ctx;
  if (backend == NULL) {
    return MRLINK_CHANNEL_ERR_NULL;
  }
  if (backend->rx_active) {
    return MRLINK_CHANNEL_OK;
  }
  const uint8_t queue_size =
      backend->rx_queue_size > 0u ? backend->rx_queue_size : BSP_FDCAN_DEFAULT_QUEUE_SIZE;
  const int8_t ret = BSP_FDCAN_RegisterId(backend->fdcan, backend->rx_id, queue_size);
  backend->rx_active = (ret == BSP_OK);
  if (ret != BSP_OK) {
    backend->rx_start_fail_count++;
  }
  return ret;
}

static uint16_t FdcanPopRx(void* ctx, uint8_t* dst, uint16_t dst_size) {
  MrLink_FdcanBackend_t* backend = (MrLink_FdcanBackend_t*)ctx;
  if (backend == NULL || dst == NULL || dst_size == 0u) {
    return 0u;
  }

  if (!backend->pending_size) {
    BSP_FDCAN_Message_t msg = {0};
    if (BSP_FDCAN_GetMessage(backend->fdcan, backend->rx_id, &msg, BSP_FDCAN_TIMEOUT_IMMEDIATE) !=
        BSP_OK)
      return 0u;
    backend->rx_irq_count++;
    backend->rx_irq_byte_count += msg.dlc;
    const BSP_FDCAN_FrameType_t expected = backend->tx_format == BSP_FDCAN_FORMAT_STD_DATA
                                               ? BSP_FDCAN_FRAME_STD_DATA
                                               : BSP_FDCAN_FRAME_EXT_DATA;
    if (msg.dlc > backend->max_frame_size || msg.frame_type != expected ||
        msg.original_id != backend->rx_id) {
      backend->rx_overflow_count++;
      return 0u;
    }
    memcpy(backend->pending, msg.data, msg.dlc);
    backend->pending_size = msg.dlc;
    backend->pending_offset = 0;
  }
  const uint16_t remaining = (uint16_t)(backend->pending_size - backend->pending_offset);
  const uint16_t n = remaining < dst_size ? remaining : dst_size;
  memcpy(dst, backend->pending + backend->pending_offset, n);
  backend->pending_offset = (uint16_t)(backend->pending_offset + n);
  if (backend->pending_offset == backend->pending_size) {
    backend->pending_size = backend->pending_offset = 0;
  }
  return n;
}

static int8_t FdcanSend(void* ctx, const uint8_t* data, uint16_t len) {
  MrLink_FdcanBackend_t* backend = (MrLink_FdcanBackend_t*)ctx;
  if (backend == NULL) {
    return MRLINK_CHANNEL_ERR_NULL;
  }
  if (data == NULL && len > 0u) {
    return MRLINK_CHANNEL_ERR_NULL;
  }
  if (len > backend->max_frame_size || len > MRLINK_CHANNEL_FDCAN_MAX_FRAME_SIZE) {
    return MRLINK_CHANNEL_ERR;
  }
  /* BSP copies into its TX queue before returning; completion means accepted
   * into that software queue, not physical bus delivery. */
  const int8_t status =
      BSP_FDCAN_Transmit(backend->fdcan, backend->tx_format, backend->tx_id, data, (uint8_t)len);
  if (status == BSP_OK && backend->tx_done) backend->tx_done(backend->tx_callback_ctx);
  if (status != BSP_OK && backend->tx_error) backend->tx_error(backend->tx_callback_ctx);
  return status;
}

static int8_t FdcanRegisterTxCallbacks(void* ctx, MrLink_ChannelNotify_t tx_done,
                                       MrLink_ChannelNotify_t tx_error, void* callback_ctx) {
  MrLink_FdcanBackend_t* backend = ctx;
  if (!backend) return MRLINK_CHANNEL_ERR_NULL;
  backend->tx_done = tx_done;
  backend->tx_error = tx_error;
  backend->tx_callback_ctx = callback_ctx;
  return MRLINK_CHANNEL_OK;
}

static int8_t FdcanDeinit(void* ctx) {
  MrLink_FdcanBackend_t* backend = ctx;
  if (!backend) return MRLINK_CHANNEL_ERR_NULL;
  if (!backend->rx_active) return MRLINK_CHANNEL_OK;
  const int8_t status = BSP_FDCAN_UnregisterId(backend->fdcan, backend->rx_id);
  if (status == BSP_OK) backend->rx_active = false;
  return status;
}

static bool FdcanIsRxActive(void* ctx) {
  const MrLink_FdcanBackend_t* backend = (const MrLink_FdcanBackend_t*)ctx;
  return (backend != NULL) ? backend->rx_active : false;
}

static uint32_t FdcanGetRxOverflowCount(const void* ctx) {
  const MrLink_FdcanBackend_t* backend = (const MrLink_FdcanBackend_t*)ctx;
  return (backend != NULL) ? backend->rx_overflow_count : 0u;
}

static uint32_t FdcanGetRxIrqCount(const void* ctx) {
  const MrLink_FdcanBackend_t* backend = (const MrLink_FdcanBackend_t*)ctx;
  return (backend != NULL) ? backend->rx_irq_count : 0u;
}

static uint32_t FdcanGetRxIrqByteCount(const void* ctx) {
  const MrLink_FdcanBackend_t* backend = (const MrLink_FdcanBackend_t*)ctx;
  return (backend != NULL) ? backend->rx_irq_byte_count : 0u;
}

static uint32_t FdcanGetRxStartFailCount(const void* ctx) {
  const MrLink_FdcanBackend_t* backend = (const MrLink_FdcanBackend_t*)ctx;
  return (backend != NULL) ? backend->rx_start_fail_count : 0u;
}
