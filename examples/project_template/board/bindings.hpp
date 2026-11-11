#pragma once
// 项目用途与物理端点名称的绑定；每块板独立维护，名称须与登记表一致。
namespace board::bindings {
inline constexpr const char* kChassisCan = "fdcan1";
inline constexpr const char* kTelemetryUart = "usart1";
inline constexpr const char* kRemoteUart = "uart5";
inline constexpr const char* kImuSpi = "spi2";
inline constexpr const char* kAccelCs = "accel_cs";
inline constexpr const char* kGyroCs = "gyro_cs";
inline constexpr const char* kBuzzerPwm = "buzzer";
}
