#pragma once
#include "bsp/mrlink/channel.h"
#include "cmsis_compiler.h"
#include "component/communication/mrlink/mrlink.hpp"

namespace mr::bsp {
inline MrLink_Synchronization_t MrLinkIrqSynchronization() {
  return {[](void*) -> uintptr_t {
            const uint32_t token = __get_PRIMASK();
            __disable_irq();
            return token;
          },
          [](void*, uintptr_t token) {
            if (token == 0u) __enable_irq();
          },
          nullptr};
}
/* External channel must stay alive until transport is cleared, and must be
 * explicitly Deinit'ed before its storage is released. No hidden static owner. */
inline mr::link::Transport MrLinkTransport(MrLink_Channel_t& channel) {
  return {&channel,
          [](const uint8_t* p, uint16_t n, void* ctx) {
            return MrLink_Channel_Send(static_cast<MrLink_Channel_t*>(ctx), p, n) ==
                   MRLINK_CHANNEL_OK;
          },
          [](void* ctx, uint8_t* p, uint16_t n) {
            return MrLink_Channel_PopRx(static_cast<MrLink_Channel_t*>(ctx), p, n);
          },
          [](void* ctx) { return MrLink_Channel_StartRx(static_cast<MrLink_Channel_t*>(ctx)); }};
}
template <uint8_t H>
int8_t BindMrLinkChannel(mr::link::BasicInstance<H>& link, MrLink_Channel_t& channel) {
  const uint16_t max_frame = link.MaxFrameSize();
  if (!max_frame || channel.hardware == MRLINK_CHANNEL_HW_NONE) return MRLINK_CHANNEL_ERR;
  if (channel.hardware == MRLINK_CHANNEL_HW_FDCAN &&
      max_frame > channel.backend_storage.fdcan.max_frame_size)
    return MRLINK_CHANNEL_ERR;
  link.BindTransport(MrLinkTransport(channel));
  return MRLINK_CHANNEL_OK;
}
}  // namespace mr::bsp
