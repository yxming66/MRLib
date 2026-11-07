#pragma once
#include "../app/telemetry.hpp"
#include "bsp/adapters/mrlink.hpp"
namespace board {
/* Call BSP_DMA_Configure with your verified memory/cache policy first.
 * Place the complete ChannelAndLink in DMA-accessible board memory. */
struct ChannelAndLink {
  mr::link::Bus<> link;
  MrLink_Channel_t channel{};
  alignas(32) uint8_t slots[4][96]{};
  volatile uint16_t lengths[4]{};
  ChannelAndLink() = default;
  ChannelAndLink(const ChannelAndLink&) = delete;
  ChannelAndLink& operator=(const ChannelAndLink&) = delete;
  int8_t Begin(BSP_UART_t uart) {
    if (channel.hardware != MRLINK_CHANNEL_HW_NONE) return MRLINK_CHANNEL_ERR_BUSY;
    int8_t status = link.Begin();
    if (status != MRLINK_OK) return status;
    const auto sync = mr::bsp::MrLinkIrqSynchronization();
    if ((status = link.SetSynchronization(&sync)) != MRLINK_OK) return status;
    MrLink_ChannelUartConfig_t cfg{};
    cfg.uart = uart;
    cfg.rx_slots = &slots[0][0];
    cfg.rx_slot_size = 96;
    cfg.rx_slot_count = 4;
    cfg.rx_len_storage = lengths;
    if ((status = MrLink_Channel_InitUart(&channel, &cfg)) != MRLINK_CHANNEL_OK) return status;
    if ((status = mr::bsp::BindMrLinkChannel(link, channel)) != MRLINK_CHANNEL_OK) return status;
    return link.StartRx();
  }
  int8_t Stop() {
    const int8_t status = MrLink_Channel_Deinit(&channel);
    if (status == MRLINK_CHANNEL_OK) link.ClearWriter();
    return status;  // On failure retain this whole object and retry; no automatic destruction.
  }
};
}  // namespace board
