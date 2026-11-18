#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "component/math/scalar.hpp"
#include "device/joint/joint_interface.hpp"

namespace mr::module::gimbal {

enum class Lifecycle : uint8_t { kUninitialized, kReady, kEnabled, kFault };
enum class Status : uint8_t {
  kOk = 0, kInvalidArgument, kInvalidState, kInvalidConfiguration,
  kDeviceError, kInvalidFeedback,
};

struct AxisGains { float kp = 0.0f; float kd = 0.0f; };
struct AxisLimits {
  float min_rad = -component::math::kPi;
  float max_rad = component::math::kPi;
};

template <std::size_t AxisCount>
struct JointConfig {
  std::array<IActuator*, AxisCount> axes{};
  std::array<AxisGains, AxisCount> gains{};
  std::array<AxisLimits, AxisCount> limits{};
};

template <std::size_t AxisCount>
struct JointFeedback {
  std::array<float, AxisCount> angles{};
  std::array<ActuatorState, AxisCount> axes{};
  bool valid = false;
};

}  // namespace mr::module::gimbal
