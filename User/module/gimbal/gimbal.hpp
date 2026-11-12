#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "component/math/scalar.hpp"
#include "device/device.h"
#include "device/joint/joint_interface.hpp"
#include "robotics/gimbal/gimbal_kinematics.hpp"

namespace mr::module::gimbal {

enum class Lifecycle : uint8_t { kUninitialized, kReady, kEnabled, kFault };

enum class Status : uint8_t {
  kOk = 0,
  kInvalidArgument,
  kInvalidState,
  kInvalidConfiguration,
  kDeviceError,
  kInvalidFeedback,
  kKinematicsError,
};

struct AxisGains {
  float kp = 0.0f;
  float kd = 0.0f;
};

struct GimbalConfig {
  robotics::gimbal::KinematicsConfig kinematics{};
  std::array<IActuator*, 2> axes{};  // [yaw, pitch], caller-owned.
  std::array<AxisGains, 2> gains{};
};

struct Feedback {
  robotics::gimbal::Angles angles{};
  std::array<ActuatorState, 2> axes{};
  bool valid = false;
};

// Generic two-axis module. Axis order is always yaw, pitch; model geometry is
// selected at registration. Device objects and all module storage are static.
class Gimbal final {
 public:
  static constexpr std::size_t kAxisCount = 2;
  using Config = GimbalConfig;
  using Command = robotics::gimbal::Angles;

  Gimbal() = default;
  Gimbal(const Gimbal&) = delete;
  Gimbal& operator=(const Gimbal&) = delete;
  Gimbal(Gimbal&&) = delete;
  Gimbal& operator=(Gimbal&&) = delete;

  Status Register(const GimbalConfig& config) {
    if (lifecycle_ != Lifecycle::kUninitialized) return Status::kInvalidState;
    if (!IsConfigValid(config)) return Status::kInvalidConfiguration;
    if (!kinematics_.Configure(config.kinematics)) return Status::kKinematicsError;
    config_ = config;
    for (std::size_t i = 0; i < kAxisCount; ++i) {
      const int8_t result = config_.axes[i]->Register();
      ++registered_count_;
      if (result != DEVICE_OK) return Fault(Status::kDeviceError, result);
    }
    lifecycle_ = Lifecycle::kReady;
    return Status::kOk;
  }

  Status UpdateFeedback() {
    if (lifecycle_ == Lifecycle::kUninitialized || lifecycle_ == Lifecycle::kFault)
      return Status::kInvalidState;
    if (pending_) return Status::kInvalidState;
    feedback_ = {};
    ClearPending();
    for (std::size_t i = 0; i < kAxisCount; ++i) {
      const int8_t result = config_.axes[i]->Update();
      feedback_.axes[i] = config_.axes[i]->GetState();
      if (result != DEVICE_OK || !feedback_.axes[i].online ||
          !component::math::is_finite_scalar(feedback_.axes[i].position) ||
          !component::math::is_finite_scalar(feedback_.axes[i].velocity) ||
          !component::math::is_finite_scalar(feedback_.axes[i].torque))
        return Fault(Status::kInvalidFeedback, result == DEVICE_OK ? DEVICE_ERR : result);
    }
    feedback_.angles = {feedback_.axes[0].position, feedback_.axes[1].position};
    feedback_.valid = true;
    return Status::kOk;
  }

  Status Enable() {
    if (lifecycle_ != Lifecycle::kReady) return Status::kInvalidState;
    if (!feedback_.valid || !IsWithinLimits(feedback_.angles)) return Status::kInvalidFeedback;
    for (auto* axis : config_.axes) {
      const int8_t result = axis->Enable();
      if (result != DEVICE_OK) return Fault(Status::kDeviceError, result);
    }
    command_ = feedback_.angles;
    lifecycle_ = Lifecycle::kEnabled;
    return Status::kOk;
  }

  Status Disable() {
    if (lifecycle_ == Lifecycle::kUninitialized) return Status::kInvalidState;
    pending_ = false;
    feedback_.valid = false;
    command_ = {};
    ClearPending();
    const int8_t result = RelaxRegistered();
    if (result != DEVICE_OK) return Fault(Status::kDeviceError, result);
    lifecycle_ = Lifecycle::kReady;
    return Status::kOk;
  }

  Status SetTargetAngles(const Command& target) {
    if (lifecycle_ != Lifecycle::kEnabled) return Status::kInvalidState;
    if (!robotics::gimbal::IsFinite(target)) return Status::kInvalidArgument;
    if (!IsWithinLimits(target)) return Status::kInvalidArgument;
    command_ = target;
    return Status::kOk;
  }

  Status SetTargetDirection(const robotics::gimbal::Direction& direction) {
    if (lifecycle_ != Lifecycle::kEnabled) return Status::kInvalidState;
    Command target{};
    const auto result = kinematics_.DirectionToAngles(direction, target);
    if (result != robotics::gimbal::Status::kOk) return Status::kKinematicsError;
    return SetTargetAngles(target);
  }

  Status Step(float dt_s) {
    if (lifecycle_ != Lifecycle::kEnabled) return Status::kInvalidState;
    if (!feedback_.valid || pending_) return Status::kInvalidState;
    if (!component::math::is_finite_scalar(dt_s) || dt_s < 1.0e-6f || dt_s > 0.1f)
      return Fault(Status::kInvalidArgument, DEVICE_ERR);
    for (std::size_t i = 0; i < kAxisCount; ++i) {
      const AxisGains& axis = config_.gains[i];
      const int8_t result = config_.axes[i]->SetMIT(
          i == 0 ? command_.yaw_rad : command_.pitch_rad, 0.0f,
          axis.kp, axis.kd, 0.0f);
      if (result != DEVICE_OK) return Fault(Status::kDeviceError, result);
    }
    pending_ = true;
    return Status::kOk;
  }

  Status CommitOutput() {
    if (lifecycle_ != Lifecycle::kEnabled || !pending_) return Status::kInvalidState;
    pending_ = false;
    for (auto* axis : config_.axes) {
      const int8_t result = axis->CommitControl();
      if (result != DEVICE_OK) return Fault(Status::kDeviceError, result);
    }
    feedback_.valid = false;
    return Status::kOk;
  }

  Lifecycle lifecycle() const { return lifecycle_; }
  const Feedback& feedback() const { return feedback_; }
  const Command& command() const { return command_; }
  int8_t last_device_error() const { return last_device_error_; }
  int8_t last_relax_error() const { return last_relax_error_; }
  const robotics::gimbal::GimbalKinematics& kinematics() const { return kinematics_; }

 private:
  static bool IsConfigValid(const GimbalConfig& config) {
    for (std::size_t i = 0; i < kAxisCount; ++i) {
      if (!config.axes[i] || !component::math::is_finite_scalar(config.gains[i].kp) ||
          !component::math::is_finite_scalar(config.gains[i].kd) ||
          config.gains[i].kp < 0.0f || config.gains[i].kd < 0.0f)
        return false;
      if (i && config.axes[i] == config.axes[0]) return false;
    }
    const auto& model = config.kinematics.yaw_pitch;
    return config.kinematics.model == robotics::gimbal::Model::kYawPitch &&
           model.min_yaw_rad < model.max_yaw_rad &&
           model.min_pitch_rad < model.max_pitch_rad;
  }

  bool IsWithinLimits(const Command& target) const {
    const auto& limits = config_.kinematics.yaw_pitch;
    return target.yaw_rad >= limits.min_yaw_rad && target.yaw_rad <= limits.max_yaw_rad &&
           target.pitch_rad >= limits.min_pitch_rad && target.pitch_rad <= limits.max_pitch_rad;
  }

  void ClearPending() {
    for (std::size_t i = 0; i < registered_count_; ++i)
      config_.axes[i]->ClearPendingControl();
  }

  int8_t RelaxRegistered() {
    int8_t result = DEVICE_OK;
    for (std::size_t i = 0; i < registered_count_; ++i) {
      const int8_t next = config_.axes[i]->Relax();
      if (result == DEVICE_OK) result = next;
    }
    return result;
  }

  Status Fault(Status status, int8_t device_error) {
    lifecycle_ = Lifecycle::kFault;
    last_device_error_ = device_error;
    pending_ = false;
    feedback_.valid = false;
    ClearPending();
    command_ = {};
    last_relax_error_ = RelaxRegistered();
    return status;
  }

  GimbalConfig config_{};
  robotics::gimbal::GimbalKinematics kinematics_{};
  Feedback feedback_{};
  Command command_{};
  Lifecycle lifecycle_ = Lifecycle::kUninitialized;
  std::size_t registered_count_ = 0;
  int8_t last_device_error_ = DEVICE_OK;
  int8_t last_relax_error_ = DEVICE_OK;
  bool pending_ = false;
};

}  // namespace mr::module::gimbal
