#pragma once
#include "bsp/can.h"

namespace board::bindings {
inline constexpr BSP_CAN_t kMotorCan = BSP_CAN_1;
inline constexpr const char* kTelemetryUart = "usart1";
inline constexpr const char* kBuzzerPwm = "tim12_ch2";
inline constexpr const char* kImuHeatPwm = "tim3_ch4";
inline constexpr const char* kArmServoPwm = "tim1_ch1";
inline constexpr const char* kRodServoPwm = "tim1_ch3";
inline constexpr const char* kImuSpi = "spi2";
inline constexpr const char* kImuAcclCs = "pc0";
inline constexpr const char* kImuGyroCs = "pc3";
inline constexpr const char* kImuAcclInt = "pe10";
inline constexpr const char* kImuGyroInt = "pe12";
inline constexpr const char* kArmValve = "pc10";
inline constexpr const char* kRodValve = "pc11";
}  // namespace board::bindings
