#include "../../examples/mrlink/board/h7_dma_policy.hpp"
#include "../../examples/mrlink/board/uart_link.hpp"
#include "bsp/adapters/mrlink.hpp"
static_assert(alignof(MrLink_Channel_t) >= 32);
static_assert(sizeof(MrLink_Channel_t) % 32 == 0);
static_assert(offsetof(MrLink_UartBackend_t, tx_buf) % 32 == 0);
static_assert(offsetof(MrLink_UsbBackend_t, tx_buf) % 32 == 0);
template class mr::link::BasicInstance<1>;
template class mr::link::BasicInstance<16>;
bool CompilePublicApi(board::ChannelAndLink& assembly) {
  return assembly.link.Publish(app::Telemetry{1, 0.5f});
}
