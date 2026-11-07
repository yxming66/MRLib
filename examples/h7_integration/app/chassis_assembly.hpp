#pragma once

#include "motor_config.hpp"
#include "robot_params.hpp"
#include "device/motor/motor.hpp"
#include "device/wheel/wheel.hpp"

namespace app {
// Ordinary project-owned C++ assembly. Hardware transport is borrowed.
// Order is front-right, front-left, rear-left, rear-right.
class ChassisAssembly final {
 public:
  explicit ChassisAssembly(mr::motor::CanPort& port)
      : commands(port),
        motor_0(WheelConfig(port, commands, params::kWheelFeedbackIds[0]), {params::kExternalRatio}),
        motor_1(WheelConfig(port, commands, params::kWheelFeedbackIds[1]), {params::kExternalRatio}),
        motor_2(WheelConfig(port, commands, params::kWheelFeedbackIds[2]), {params::kExternalRatio}),
        motor_3(WheelConfig(port, commands, params::kWheelFeedbackIds[3]), {params::kExternalRatio}),
        wheel_0(motor_0, params::kWheelRadiusM), wheel_1(motor_1, params::kWheelRadiusM),
        wheel_2(motor_2, params::kWheelRadiusM), wheel_3(motor_3, params::kWheelRadiusM) {}
  ChassisAssembly(const ChassisAssembly&) = delete;
  ChassisAssembly& operator=(const ChassisAssembly&) = delete;

  mr::module::chassis::ChassisStatus Register(uint32_t period_us) {
    return chassis.Register(params::ChassisConfig(period_us), Wheels());
  }
  std::array<mr::wheel::WheelDevice*, 4> Wheels() {
    return {&wheel_0, &wheel_1, &wheel_2, &wheel_3};
  }

  mr::motor::RmCommandBus commands;
  mr::motor::RmM3508Motor motor_0, motor_1, motor_2, motor_3;
  mr::wheel::MotorWheel<mr::motor::RmM3508Motor> wheel_0, wheel_1, wheel_2, wheel_3;
  mr::module::chassis::Chassis chassis;
};
}  // namespace app
