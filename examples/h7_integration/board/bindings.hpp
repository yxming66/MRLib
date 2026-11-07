#pragma once
#include "bsp/can.h"

namespace board::bindings {
inline constexpr BSP_CAN_t kMotorCan = BSP_CAN_1;
inline constexpr const char* kTelemetryUart = "usart1";
inline constexpr const char* kBuzzerPwm = "tim12_ch2";
inline constexpr const char* kImuHeatPwm = "tim3_ch4";
inline constexpr const char* kArmServoPwm = "tim1_ch1";
inline constexpr const char* kRodServoPwm = "tim1_ch3";
}  // namespace board::bindings
