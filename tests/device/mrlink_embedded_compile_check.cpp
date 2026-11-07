#include "../../examples/mrlink/board/h7_dma_policy.hpp"
#include "../../examples/mrlink/board/uart_link.hpp"
#include "bsp/adapters/mrlink.hpp"
#include "device/bmi088.h"
#include "../../examples/h7_integration/board/bindings.hpp"
static_assert(alignof(MrLink_Channel_t) >= 32);
static_assert(sizeof(MrLink_Channel_t) % 32 == 0);
static_assert(offsetof(MrLink_UartBackend_t, tx_buf) % 32 == 0);
static_assert(offsetof(MrLink_UsbBackend_t, tx_buf) % 32 == 0);
template class mr::link::BasicInstance<1>;
template class mr::link::BasicInstance<16>;
bool CompilePublicApi(board::ChannelAndLink& assembly) {
  return assembly.link.Publish(app::Telemetry{1, 0.5f});
}

// Ordinary project assembly: no default bus/pins or generated configuration.
int8_t CompileBmi088Api(BMI088_t& imu, const BMI088_Cali_t& calibration) {
  const BMI088_Config_t config{
      BSP_SPI_FindEndpoint(board::bindings::kImuSpi),
      BSP_GPIO_FindEndpoint(board::bindings::kImuAcclCs),
      BSP_GPIO_FindEndpoint(board::bindings::kImuGyroCs),
      BSP_GPIO_FindEndpoint(board::bindings::kImuAcclInt),
      BSP_GPIO_FindEndpoint(board::bindings::kImuGyroInt)};
  return BMI088_Init(&imu, &calibration, &config);
}
