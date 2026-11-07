#include <stddef.h>
#include <string.h>

#include "bsp/mrlink/internal.h"
#include "cmsis_compiler.h"

static void UsbDone(void* ctx) {
  MrLink_UsbBackend_t* b = ctx;
  b->tx_busy = false;
  if (b->tx_done) b->tx_done(b->tx_callback_ctx);
}
static void UsbError(void* ctx) {
  MrLink_UsbBackend_t* b = ctx;
  b->tx_busy = false;
  if (b->tx_error) b->tx_error(b->tx_callback_ctx);
}
static int8_t UsbDeinit(void* ctx) {
  MrLink_UsbBackend_t* b = ctx;
  if (b->config.stop) {
    const int8_t status = b->config.stop(b->config.ctx);
    if (status != MRLINK_CHANNEL_OK) return status;
  }
  b->tx_busy = false;
  return MRLINK_CHANNEL_OK;
}
static int8_t UsbStartRx(void* ctx) {
  MrLink_UsbBackend_t* b = ctx;
  return b->config.start_rx ? b->config.start_rx(b->config.ctx) : MRLINK_CHANNEL_OK;
}
static uint16_t UsbPopRx(void* ctx, uint8_t* dst, uint16_t n) {
  MrLink_UsbBackend_t* b = ctx;
  if (!dst || !n) return 0;
  return b->config.pop_rx(b->config.ctx, dst, n);
}
static int8_t UsbSend(void* ctx, const uint8_t* data, uint16_t n) {
  MrLink_UsbBackend_t* b = ctx;
  if (!data || !n || n > sizeof(b->tx_buf)) return MRLINK_CHANNEL_ERR;
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  if (b->tx_busy) {
    if (!token) __enable_irq();
    return MRLINK_CHANNEL_ERR_BUSY;
  }
  memcpy(b->tx_buf, data, n);
  b->tx_busy = true;
  const int8_t status = b->config.send(b->config.ctx, b->tx_buf, n);
  if (status != MRLINK_CHANNEL_OK || b->config.synchronous_send) b->tx_busy = false;
  if (!token) __enable_irq();
  if (status == MRLINK_CHANNEL_OK && b->config.synchronous_send && b->tx_done)
    b->tx_done(b->tx_callback_ctx);
  return status;
}
static int8_t UsbRegisterTxCallbacks(void* ctx, MrLink_ChannelNotify_t done,
                                     MrLink_ChannelNotify_t error, void* user) {
  MrLink_UsbBackend_t* b = ctx;
  const uint32_t token = __get_PRIMASK();
  __disable_irq();
  b->tx_done = done;
  b->tx_error = error;
  b->tx_callback_ctx = user;
  if (!token) __enable_irq();
  return MRLINK_CHANNEL_OK;
}
static bool UsbIsRxActive(void* ctx) {
  MrLink_UsbBackend_t* b = ctx;
  return b->config.is_rx_active ? b->config.is_rx_active(b->config.ctx) : false;
}
static const MrLink_ChannelOps_t kUsbOps = {.deinit = UsbDeinit,
                                            .start_rx = UsbStartRx,
                                            .pop_rx = UsbPopRx,
                                            .send = UsbSend,
                                            .register_tx_callbacks = UsbRegisterTxCallbacks,
                                            .is_rx_active = UsbIsRxActive};
int8_t MrLink_ChannelBackend_InitUsb(MrLink_Channel_t* channel,
                                     const MrLink_ChannelUsbConfig_t* config) {
  if (!channel || !config) return MRLINK_CHANNEL_ERR_NULL;
  if (!config->send || !config->pop_rx ||
      (!config->synchronous_send && (!config->register_tx_callbacks || !config->stop)))
    return MRLINK_CHANNEL_ERR_UNSUPPORTED;
  memset(channel, 0, sizeof(*channel));
  MrLink_UsbBackend_t* b = &channel->backend_storage.usb;
  b->config = *config;
  if (!config->synchronous_send) {
    /* Board callback registration must be transactional on failure. */
    const int8_t status = config->register_tx_callbacks(config->ctx, UsbDone, UsbError, b);
    if (status != MRLINK_CHANNEL_OK) {
      memset(channel, 0, sizeof(*channel));
      return status;
    }
  }
  MrLink_Channel_SetBackend(channel, MRLINK_CHANNEL_HW_USB, &kUsbOps);
  return MRLINK_CHANNEL_OK;
}
