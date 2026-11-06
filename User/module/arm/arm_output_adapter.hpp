#pragma once

#include <array>
#include <cstddef>
#include "component/math/scalar.hpp"
#include "device/device.h"
#include "device/joint/joint_interface.hpp"
#include "robotics/arm/core/types.hpp"

namespace mr::module::arm {

// Actuators expose joint coordinates: rad, rad/s, Nm for revolute joints;
// m, m/s, N for prismatic joints. Gear/coupling conversion belongs to device.
template <int N>
class ArmOutputAdapter final {
  static_assert(N > 0, "Arm requires at least one joint");
 public:
  struct Gains { float kp = 0; float kd = 0; float effort_scale = 1; };
  struct Config {
    std::array<IActuator*, N> actuators{};
    std::array<Gains, N> gains{};
  };
  static bool IsConfigValid(const Config& config) {
    for (int i = 0; i < N; ++i) {
      if (!config.actuators[i]) return false;
      for (int j = 0; j < i; ++j)
        if (config.actuators[i] == config.actuators[j]) return false;
      const auto& g = config.gains[i];
      if (!component::math::is_finite_scalar(g.kp) || g.kp < 0 ||
          !component::math::is_finite_scalar(g.kd) || g.kd < 0 ||
          !component::math::is_finite_scalar(g.effort_scale) ||
          g.effort_scale < 0) return false;
    }
    return true;
  }
  bool Bind(const Config& config) {
    if (bound_ || !IsConfigValid(config)) return false;
    config_ = config;
    bound_ = true;
    return true;
  }
  int8_t Register() {
    if (!bound_) return DEVICE_ERR_NULL;
    for (auto* actuator : config_.actuators) {
      const int8_t result = actuator->Register();
      if (result != DEVICE_OK) return result;
    }
    return DEVICE_OK;
  }
  int8_t Enable() {
    if (!bound_) return DEVICE_ERR_NULL;
    for (auto* actuator : config_.actuators) {
      const int8_t result = actuator->Enable();
      if (result != DEVICE_OK) return result;
    }
    return DEVICE_OK;
  }
  void ClearPending() {
    if (bound_)
      for (auto* actuator : config_.actuators) actuator->ClearPendingControl();
  }
  int8_t Relax() {
    if (!bound_) return DEVICE_ERR_NULL;
    ClearPending();
    int8_t result = DEVICE_OK;
    for (auto* actuator : config_.actuators) {
      const auto next = actuator->Relax();
      if (result == DEVICE_OK) result = next;
    }
    return result;
  }
  int8_t UpdateFeedback(robotics::arm::JointState<N>& state) {
    if (!bound_) return DEVICE_ERR_NULL;
    state = {};
    int8_t result = DEVICE_OK;
    for (int i = 0; i < N; ++i) {
      const auto next = config_.actuators[i]->Update();
      const auto sample = config_.actuators[i]->GetState();
      state.position[i][0] = sample.position;
      state.velocity[i][0] = sample.velocity;
      state.effort[i][0] = sample.torque;
      if (result == DEVICE_OK)
        result = next != DEVICE_OK ? next : (sample.online ? DEVICE_OK : DEVICE_ERR);
    }
    state.valid = result == DEVICE_OK && robotics::arm::IsJointStateFinite(state);
    return state.valid ? DEVICE_OK : (result == DEVICE_OK ? DEVICE_ERR : result);
  }
  int8_t Apply(const robotics::arm::JointCommand<N>& command) {
    using robotics::arm::JointCommandMode;
    if (!bound_) return DEVICE_ERR_NULL;
    if (!robotics::arm::IsJointCommandUsable(command)) return DEVICE_ERR;
    if (command.mode == JointCommandMode::kDisabled) return Relax();
    if (command.mode != JointCommandMode::kPosition &&
        command.mode != JointCommandMode::kPositionVelocity &&
        command.mode != JointCommandMode::kPositionVelocityEffort &&
        command.mode != JointCommandMode::kEffort) return DEVICE_ERR_UNSUPPORTED;
    for (int i = 0; i < N; ++i)
      if (!component::math::is_finite_scalar(
          command.effort_feedforward[i][0] * config_.gains[i].effort_scale))
        return DEVICE_ERR;
    for (int i = 0; i < N; ++i) {
      const auto& g = config_.gains[i];
      const bool effort = command.mode == JointCommandMode::kEffort ||
                          command.mode == JointCommandMode::kPositionVelocityEffort;
      const float torque = effort ? command.effort_feedforward[i][0] * g.effort_scale : 0;
      const int8_t result = command.mode == JointCommandMode::kEffort
          ? config_.actuators[i]->SetTorque(torque)
          : config_.actuators[i]->SetMIT(command.position[i][0],
                command.mode == JointCommandMode::kPosition ? 0 : command.velocity[i][0],
                g.kp, g.kd, torque);
      if (result != DEVICE_OK) return result;
    }
    return DEVICE_OK;
  }
  int8_t Commit() {
    if (!bound_) return DEVICE_ERR_NULL;
    for (auto* actuator : config_.actuators) {
      const int8_t result = actuator->CommitControl();
      if (result != DEVICE_OK) return result;
    }
    return DEVICE_OK;
  }
  int8_t SetZero(size_t index) {
    return bound_ && index < N ? config_.actuators[index]->SetZero() : DEVICE_ERR_NULL;
  }
 private:
  Config config_{};
  bool bound_ = false;
};
}  // namespace mr::module::arm
