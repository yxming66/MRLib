#pragma once
#include <array>
#include "app/robot_config.hpp"
#include "device/motor/motor.hpp"
#include "device/wheel/wheel.hpp"
#include "module/chassis/chassis.hpp"

namespace app {
inline mr::motor::MotorInstanceConfig<mr::motor::MotorKind::RM> WheelConfig(
    mr::motor::CanPort& port, mr::motor::RmCommandBus& commands, std::size_t index) {
  mr::motor::MotorInstanceConfig<mr::motor::MotorKind::RM> config;
  config.port = &port;
  config.commands = &commands;
  config.id = params::kWheelFeedbackIds[index];
  config.reverse = params::kWheelReverse[index];
  config.feedback_timeout_ms = params::kMotorFeedbackTimeoutMs;
  return config;
}

// 所有对象由工程持有，固定存储；顺序为右前、左前、左后、右后。
class ChassisAssembly final {
 public:
  explicit ChassisAssembly(mr::motor::CanPort& port)
      : commands(port),
        front_right(WheelConfig(port, commands, 0), {params::kExternalRatio}),
        front_left(WheelConfig(port, commands, 1), {params::kExternalRatio}),
        rear_left(WheelConfig(port, commands, 2), {params::kExternalRatio}),
        rear_right(WheelConfig(port, commands, 3), {params::kExternalRatio}),
        wheel_fr(front_right, params::kWheelRadiusM),
        wheel_fl(front_left, params::kWheelRadiusM),
        wheel_rl(rear_left, params::kWheelRadiusM),
        wheel_rr(rear_right, params::kWheelRadiusM) {}
  ChassisAssembly(const ChassisAssembly&) = delete;
  ChassisAssembly& operator=(const ChassisAssembly&) = delete;

  // 当前只订阅反馈。不使能、不提交、不 Flush。
  int8_t RegisterFeedback() {
    for (auto* wheel : Wheels()) {
      const auto result = wheel->Register();
      if (result != DEVICE_OK) return result;
    }
    return DEVICE_OK;
  }
  uint8_t PollFeedback() {
    const auto wheels = Wheels();
    uint8_t online_mask = 0;
    for (std::size_t i = 0; i < wheels.size(); ++i) {
      feedback_results[i] = wheels[i]->UpdateFeedback(feedback[i]);
      if (feedback_results[i] == DEVICE_OK && feedback[i].online)
        online_mask |= static_cast<uint8_t>(1u << i);
    }
    return online_mask;
  }
  std::array<mr::wheel::WheelDevice*, 4> Wheels() {
    return {&wheel_fr, &wheel_fl, &wheel_rl, &wheel_rr};
  }
  std::array<mr::motor::RmM3508Motor*, 4> Motors() {
    return {&front_right, &front_left, &rear_left, &rear_right};
  }

  mr::motor::RmCommandBus commands;
  mr::motor::RmM3508Motor front_right, front_left, rear_left, rear_right;
  mr::wheel::MotorWheel<mr::motor::RmM3508Motor> wheel_fr, wheel_fl, wheel_rl, wheel_rr;
  // 待 PID、限幅和方向确定后，使用 kKinematics + 调参结果 Register。
  mr::module::chassis::Chassis chassis;
  std::array<mr::wheel::WheelFeedback, 4> feedback{};
  std::array<int8_t, 4> feedback_results{};
};
}  // namespace app
