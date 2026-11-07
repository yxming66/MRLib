#pragma once
#include "bsp/can.h"

namespace board::bindings {
inline constexpr BSP_CAN_t kMotorCan = BSP_CAN_1;
inline constexpr const char* kTelemetryUart = "usart1";
}  // namespace board::bindings
