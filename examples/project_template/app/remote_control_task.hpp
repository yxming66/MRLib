#pragma once

#include <cstdint>

#include "app/app_messages.hpp"
#include "bsp/uart.h"
#include "device/dr16.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"

namespace app {

// DR16 receive blocks on its thread flags, so it has a dedicated task instead
// of sharing the fixed-period chassis/gimbal control loop.
class RemoteControlTask final {
 public:
  static constexpr uint32_t kStackBytes = 2048;
  RemoteControlTask() = default;
  RemoteControlTask(const RemoteControlTask&) = delete;
  RemoteControlTask& operator=(const RemoteControlTask&) = delete;

  bool Start(BSP_UART_t uart);

 private:
  static void Entry(void* context);
  [[noreturn]] void Run();
  void PublishOffline();
  RemoteSwitch ConvertSwitch(DR16_SwitchPos_t position) const;

  BSP_UART_t uart_ = BSP_UART_ERR;
  DR16_t receiver_{};
  StaticTask_t control_{};
  alignas(8) StackType_t stack_[kStackBytes / sizeof(StackType_t)]{};
  uint32_t sequence_ = 0;
  bool started_ = false;
};

}  // namespace app
