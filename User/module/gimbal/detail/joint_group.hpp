#pragma once

#include "device/device.h"
#include "module/gimbal/types.hpp"

namespace mr::module::gimbal::detail {

// Internal actuator handling only. Modes and target semantics stay in each gimbal.
template <std::size_t AxisCount>
class JointGroup final {
 public:
  using Angles = std::array<float, AxisCount>;
  JointGroup() = default;
  JointGroup(const JointGroup&) = delete;
  JointGroup& operator=(const JointGroup&) = delete;

  Status Register(const JointConfig<AxisCount>& config) {
    if (lifecycle_ != Lifecycle::kUninitialized) return Status::kInvalidState;
    for (std::size_t i = 0; i < AxisCount; ++i) {
      if (!config.axes[i] || !Finite(config.gains[i].kp) || !Finite(config.gains[i].kd) ||
          config.gains[i].kp < 0.0f || config.gains[i].kd < 0.0f ||
          !Finite(config.limits[i].min_rad) || !Finite(config.limits[i].max_rad) ||
          config.limits[i].min_rad >= config.limits[i].max_rad)
        return Status::kInvalidConfiguration;
      for (std::size_t j = 0; j < i; ++j)
        if (config.axes[i] == config.axes[j]) return Status::kInvalidConfiguration;
    }
    config_ = config;
    for (auto* axis : config_.axes) {
      const int8_t result = axis->Register();
      ++registered_count_;
      if (result != DEVICE_OK) return Fault(Status::kDeviceError, result);
    }
    registration_complete_ = true;
    lifecycle_ = Lifecycle::kReady;
    return Status::kOk;
  }

  Status Enable() {
    if (lifecycle_ != Lifecycle::kReady) return Status::kInvalidState;
    const Status result = ReadFeedback();
    if (result != Status::kOk) return result;
    if (!IsWithinLimits(feedback_.angles)) return Status::kInvalidFeedback;
    for (auto* axis : config_.axes) {
      const int8_t next = axis->Enable();
      if (next != DEVICE_OK) return Fault(Status::kDeviceError, next);
    }
    command_ = feedback_.angles;
    lifecycle_ = Lifecycle::kEnabled;
    return Status::kOk;
  }

  Status Disable() {
    if (lifecycle_ == Lifecycle::kUninitialized) return Status::kInvalidState;
    ClearPending();
    feedback_.valid = false;
    command_ = {};
    last_relax_error_ = Relax();
    if (last_relax_error_ != DEVICE_OK)
      return Fault(Status::kDeviceError, last_relax_error_);
    // Failed partial registration cannot be promoted to a usable mechanism.
    if (!registration_complete_) return Status::kInvalidState;
    lifecycle_ = Lifecycle::kReady;
    return Status::kOk;
  }

  bool IsWithinLimits(const Angles& angles) const {
    for (std::size_t i = 0; i < AxisCount; ++i)
      if (!Finite(angles[i]) || angles[i] < config_.limits[i].min_rad ||
          angles[i] > config_.limits[i].max_rad) return false;
    return true;
  }

  Status BeginUpdate(float dt_s) {
    if (lifecycle_ != Lifecycle::kEnabled) return Status::kInvalidState;
    if (!Finite(dt_s) || dt_s < 1.0e-6f || dt_s > 0.1f)
      return Fault(Status::kInvalidArgument, DEVICE_ERR);
    return ReadFeedback();
  }

  Status WriteOutput(const Angles& target) {
    if (lifecycle_ != Lifecycle::kEnabled || !feedback_.valid) return Status::kInvalidState;
    if (!IsWithinLimits(target)) return Fault(Status::kInvalidArgument, DEVICE_ERR);
    for (std::size_t i = 0; i < AxisCount; ++i) {
      const auto& gains = config_.gains[i];
      const int8_t result = config_.axes[i]->SetMIT(target[i], 0.0f, gains.kp, gains.kd, 0.0f);
      if (result != DEVICE_OK) return Fault(Status::kDeviceError, result);
    }
    for (auto* axis : config_.axes) {
      const int8_t result = axis->CommitControl();
      if (result != DEVICE_OK) return Fault(Status::kDeviceError, result);
    }
    command_ = target;
    feedback_.valid = false;
    return Status::kOk;
  }

  Lifecycle lifecycle() const { return lifecycle_; }
  const JointFeedback<AxisCount>& feedback() const { return feedback_; }
  const Angles& command() const { return command_; }
  int8_t last_device_error() const { return last_device_error_; }
  int8_t last_relax_error() const { return last_relax_error_; }

 private:
  static bool Finite(float value) { return component::math::is_finite_scalar(value); }

  Status ReadFeedback() {
    feedback_ = {};
    for (std::size_t i = 0; i < AxisCount; ++i) {
      const int8_t result = config_.axes[i]->Update();
      const auto state = config_.axes[i]->GetState();
      feedback_.axes[i] = state;
      if (result != DEVICE_OK || !state.online || !Finite(state.position) ||
          !Finite(state.velocity) || !Finite(state.torque))
        return Fault(Status::kInvalidFeedback, result == DEVICE_OK ? DEVICE_ERR : result);
      feedback_.angles[i] = state.position;
    }
    feedback_.valid = true;
    return Status::kOk;
  }

  void ClearPending() {
    for (std::size_t i = 0; i < registered_count_; ++i)
      config_.axes[i]->ClearPendingControl();
  }

  int8_t Relax() {
    int8_t result = DEVICE_OK;
    for (std::size_t i = 0; i < registered_count_; ++i) {
      const int8_t next = config_.axes[i]->Relax();
      if (result == DEVICE_OK) result = next;
    }
    return result;
  }

  Status Fault(Status status, int8_t error) {
    lifecycle_ = Lifecycle::kFault;
    last_device_error_ = error;
    feedback_.valid = false;
    ClearPending();
    last_relax_error_ = Relax();
    return status;
  }

  JointConfig<AxisCount> config_{};
  JointFeedback<AxisCount> feedback_{};
  Angles command_{};
  Lifecycle lifecycle_ = Lifecycle::kUninitialized;
  std::size_t registered_count_ = 0;
  bool registration_complete_ = false;
  int8_t last_device_error_ = DEVICE_OK;
  int8_t last_relax_error_ = DEVICE_OK;
};

}  // namespace mr::module::gimbal::detail
