#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "component/math/scalar.hpp"
#include "device/device.h"
#include "device/joint/joint_interface.hpp"
#include "robotics/gimbal/gimbal_kinematics.hpp"

namespace mr::module::gimbal {

constexpr std::size_t kMaxAxisCount = 3;

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
  std::array<IActuator*, kMaxAxisCount> axes{};
  std::array<AxisGains, kMaxAxisCount> gains{};
};

struct Feedback {
  robotics::gimbal::JointAngles angles{};
  std::array<ActuatorState, kMaxAxisCount> axes{};
  std::size_t axis_count = 0;
  bool valid = false;
};

// One board-neutral module supports the registered robotics model. Storage is
// fixed for at most three axes; model registration selects the active count.
// Axis handles and all state remain caller-owned/static; no heap is used.
class Gimbal final {
 public:
  using Config = GimbalConfig;
  using Command = robotics::gimbal::JointAngles;

  Gimbal() = default;
  Gimbal(const Gimbal&) = delete;
  Gimbal& operator=(const Gimbal&) = delete;
  Gimbal(Gimbal&&) = delete;
  Gimbal& operator=(Gimbal&&) = delete;

  Status Register(const Config& config) {
    if (lifecycle_ != Lifecycle::kUninitialized) return Status::kInvalidState;
    if (!kinematics_.Configure(config.kinematics)) return Status::kKinematicsError;
    axis_count_ = kinematics_.AxisCount();
    if (!IsConfigValid(config)) return Status::kInvalidConfiguration;
    config_ = config;
    for (std::size_t i = 0; i < axis_count_; ++i) {
      const int8_t result = config_.axes[i]->Register();
      ++registered_count_;
      if (result != DEVICE_OK) return Fault(Status::kDeviceError, result);
    }
    lifecycle_ = Lifecycle::kReady;
    return Status::kOk;
  }

  // Samples encoder/device state for inner actuator control and safe holding.
  // App-level IMU feedback remains separate and is consumed by its controller.
  Status UpdateFeedback() {
    if (lifecycle_ == Lifecycle::kUninitialized || lifecycle_ == Lifecycle::kFault || pending_)
      return Status::kInvalidState;
    feedback_ = {};
    feedback_.axis_count = axis_count_;
    for (std::size_t i = 0; i < axis_count_; ++i) {
      const int8_t result = config_.axes[i]->Update();
      feedback_.axes[i] = config_.axes[i]->GetState();
      if (result != DEVICE_OK || !feedback_.axes[i].online ||
          !component::math::is_finite_scalar(feedback_.axes[i].position) ||
          !component::math::is_finite_scalar(feedback_.axes[i].velocity) ||
          !component::math::is_finite_scalar(feedback_.axes[i].torque))
        return Fault(Status::kInvalidFeedback, result == DEVICE_OK ? DEVICE_ERR : result);
      feedback_.angles[i] = feedback_.axes[i].position;
    }
    feedback_.valid = true;
    return Status::kOk;
  }

  Status Enable() {
    if (lifecycle_ != Lifecycle::kReady) return Status::kInvalidState;
    if (!feedback_.valid || !kinematics_.IsWithinLimits(feedback_.angles))
      return Status::kInvalidFeedback;
    for (std::size_t i = 0; i < axis_count_; ++i) {
      const int8_t result = config_.axes[i]->Enable();
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
    for (std::size_t i = 0; i < axis_count_; ++i)
      if (!component::math::is_finite_scalar(target[i])) return Status::kInvalidArgument;
    if (!kinematics_.IsWithinLimits(target)) return Status::kInvalidArgument;
    command_ = target;
    return Status::kOk;
  }

  // pitch1_rad resolves the redundant pitch axis for the yaw-pit-pit model;
  // it is ignored by the ordinary yaw-pitch model.
  Status SetTargetDirection(const robotics::gimbal::Direction& direction,
                            float pitch1_rad = 0.0f) {
    if (lifecycle_ != Lifecycle::kEnabled) return Status::kInvalidState;
    Command target{};
    const auto result = kinematics_.DirectionToAngles(direction, pitch1_rad, target);
    if (result != robotics::gimbal::Status::kOk) return Status::kKinematicsError;
    return SetTargetAngles(target);
  }

  Status Step(float dt_s) {
    if (lifecycle_ != Lifecycle::kEnabled) return Status::kInvalidState;
    if (!feedback_.valid || pending_) return Status::kInvalidState;
    if (!component::math::is_finite_scalar(dt_s) || dt_s < 1.0e-6f || dt_s > 0.1f)
      return Fault(Status::kInvalidArgument, DEVICE_ERR);
    for (std::size_t i = 0; i < axis_count_; ++i) {
      const AxisGains& gains = config_.gains[i];
      const int8_t result = config_.axes[i]->SetMIT(command_[i], 0.0f,
                                                    gains.kp, gains.kd, 0.0f);
      if (result != DEVICE_OK) return Fault(Status::kDeviceError, result);
    }
    pending_ = true;
    return Status::kOk;
  }

  Status CommitOutput() {
    if (lifecycle_ != Lifecycle::kEnabled || !pending_) return Status::kInvalidState;
    pending_ = false;
    for (std::size_t i = 0; i < axis_count_; ++i) {
      const int8_t result = config_.axes[i]->CommitControl();
      if (result != DEVICE_OK) return Fault(Status::kDeviceError, result);
    }
    feedback_.valid = false;
    return Status::kOk;
  }

  std::size_t axis_count() const { return axis_count_; }
  Lifecycle lifecycle() const { return lifecycle_; }
  const Feedback& feedback() const { return feedback_; }
  const Command& command() const { return command_; }
  int8_t last_device_error() const { return last_device_error_; }
  int8_t last_relax_error() const { return last_relax_error_; }
  const robotics::gimbal::GimbalKinematics& kinematics() const { return kinematics_; }

 private:
  bool IsConfigValid(const Config& config) const {
    for (std::size_t i = 0; i < axis_count_; ++i) {
      if (!config.axes[i] || !component::math::is_finite_scalar(config.gains[i].kp) ||
          !component::math::is_finite_scalar(config.gains[i].kd) ||
          config.gains[i].kp < 0.0f || config.gains[i].kd < 0.0f)
        return false;
      for (std::size_t j = 0; j < i; ++j)
        if (config.axes[i] == config.axes[j]) return false;
    }
    for (std::size_t i = axis_count_; i < kMaxAxisCount; ++i)
      if (config.axes[i] != nullptr) return false;
    return axis_count_ > 0 && axis_count_ <= kMaxAxisCount;
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

  Config config_{};
  robotics::gimbal::GimbalKinematics kinematics_{};
  Feedback feedback_{};
  Command command_{};
  Lifecycle lifecycle_ = Lifecycle::kUninitialized;
  std::size_t axis_count_ = 0;
  std::size_t registered_count_ = 0;
  int8_t last_device_error_ = DEVICE_OK;
  int8_t last_relax_error_ = DEVICE_OK;
  bool pending_ = false;
};

}  // namespace mr::module::gimbal
