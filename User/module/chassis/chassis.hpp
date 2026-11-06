#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "component/controller/pid.hpp"
#include "component/math/scalar.hpp"
#include "device/wheel/wheel_interface.hpp"
#include "robotics/chassis/chassis_kinematics.hpp"

namespace mr::module::chassis {

enum class ChassisStatus : uint8_t {
  kOk = 0,
  kInvalidArgument,
  kInvalidState,
  kInvalidConfiguration,
  kDeviceError,
  kInvalidFeedback,
  kNoFreshFeedback,
  kKinematicsError,
};

enum class ChassisState : uint8_t {
  kUnregistered = 0,
  kRegistered,
  kEnabled,
  kFault,
};

struct ChassisVelocityLimit {
  float vx_mps = 0.0f;
  float vy_mps = 0.0f;
  float wz_rad_s = 0.0f;
};

struct ChassisConfig {
  mr::robotics::chassis::ChassisKinematicsConfig kinematics{};
  ChassisVelocityLimit max_body_velocity{};
  float max_wheel_speed_mps = 0.0f;
  mr::comp::cntlr::pid_config wheel_velocity_pid{};
  uint32_t feedback_timeout_ms = 20U;
};

struct ChassisFeedback {
  mr::robotics::chassis::PlanarVelocity body_velocity{};
  std::array<mr::wheel::WheelFeedback,
             mr::robotics::chassis::ChassisKinematics::kWheelCount> wheels{};
  bool valid = false;
};

struct ChassisOutput {
  mr::robotics::chassis::WheelSpeeds wheel_speed_ref_mps{};
  std::array<float, mr::robotics::chassis::ChassisKinematics::kWheelCount>
      wheel_output_torque_nm{};
  bool valid = false;
};

/**
 * Generic four-wheel velocity-control module.
 *
 * Register selects a robotics kinematics model and binds caller-owned wheel
 * devices. UpdateFeedback receives the caller's monotonic millisecond clock,
 * which must use the same epoch as the wheel bus timestamps. The update sequence
 * is UpdateFeedback -> SetCommand -> Step ->
 * CommitOutput. No model selection is embedded in a task or board adapter.
 */
class Chassis final {
 public:
  static constexpr std::size_t kWheelCount =
      mr::robotics::chassis::ChassisKinematics::kWheelCount;

  ChassisStatus Register(
      const ChassisConfig& config,
      const std::array<mr::wheel::WheelDevice*, kWheelCount>& wheels) {
    if (state_ != ChassisState::kUnregistered) {
      return ChassisStatus::kInvalidState;
    }
    if (!IsValidConfig(config)) {
      return ChassisStatus::kInvalidConfiguration;
    }
    for (const mr::wheel::WheelDevice* wheel : wheels) {
      if (wheel == nullptr) {
        return ChassisStatus::kInvalidArgument;
      }
    }
    if (!kinematics_.Configure(config.kinematics)) {
      return ChassisStatus::kKinematicsError;
    }

    config_ = config;
    wheels_ = wheels;
    for (auto& controller : wheel_velocity_pid_) {
      controller.Configure(config_.wheel_velocity_pid);
    }

    for (std::size_t index = 0; index < kWheelCount; ++index) {
      if (wheels_[index]->Register() != 0) {
        registered_wheels_ = index;
        EnterFault();
        return ChassisStatus::kDeviceError;
      }
      registered_wheels_ = index + 1U;
    }

    state_ = ChassisState::kRegistered;
    return ChassisStatus::kOk;
  }

  ChassisStatus Enable() {
    if (state_ != ChassisState::kRegistered) {
      return ChassisStatus::kInvalidState;
    }
    for (mr::wheel::WheelDevice* wheel : wheels_) {
      if (wheel->Enable() != 0) {
        EnterFault();
        return ChassisStatus::kDeviceError;
      }
    }
    state_ = ChassisState::kEnabled;
    return ChassisStatus::kOk;
  }

  // Compatibility path for callers that have not yet wired a board clock.
  // It validates the sample but cannot enforce the timestamp timeout.
  ChassisStatus UpdateFeedback() { return UpdateFeedbackInternal(0U, false); }

  ChassisStatus UpdateFeedback(uint32_t now_ms) {
    return UpdateFeedbackInternal(now_ms, true);
  }

 private:
  ChassisStatus UpdateFeedbackInternal(uint32_t now_ms, bool enforce_age) {
    if (state_ != ChassisState::kEnabled || output_pending_) {
      return ChassisStatus::kInvalidState;
    }

    mr::robotics::chassis::WheelSpeeds wheel_speeds{};
    for (std::size_t index = 0; index < kWheelCount; ++index) {
      auto& wheel_feedback = feedback_.wheels[index];
      if (wheels_[index]->UpdateFeedback(wheel_feedback) != 0 ||
          !wheel_feedback.online ||
          (enforce_age &&
           now_ms - wheel_feedback.timestamp_ms > config_.feedback_timeout_ms) ||
          !mr::component::math::is_finite_scalar(
              wheel_feedback.linear_velocity_mps) ||
          !mr::component::math::is_finite_scalar(
              wheel_feedback.position_rad) ||
          !mr::component::math::is_finite_scalar(
              wheel_feedback.output_torque_nm)) {
        feedback_ = {};
        feedback_updated_ = false;
        EnterFault();
        return ChassisStatus::kInvalidFeedback;
      }
      wheel_speeds[index] = wheel_feedback.linear_velocity_mps;
    }

    if (!kinematics_.ForwardKinematics(wheel_speeds,
                                      feedback_.body_velocity)) {
      feedback_ = {};
      feedback_updated_ = false;
      EnterFault();
      return ChassisStatus::kKinematicsError;
    }
    feedback_.valid = true;
    feedback_updated_ = true;
    return ChassisStatus::kOk;
  }

 public:

  ChassisStatus SetCommand(
      const mr::robotics::chassis::PlanarVelocity& command) {
    if (state_ != ChassisState::kEnabled) {
      return ChassisStatus::kInvalidState;
    }
    if (!mr::robotics::chassis::IsFinite(command)) {
      command_ = {};
      EnterFault();
      return ChassisStatus::kInvalidArgument;
    }

    command_.vx_mps = mr::component::math::clamp_scalar(
        command.vx_mps, -config_.max_body_velocity.vx_mps,
        config_.max_body_velocity.vx_mps);
    command_.vy_mps = mr::component::math::clamp_scalar(
        command.vy_mps, -config_.max_body_velocity.vy_mps,
        config_.max_body_velocity.vy_mps);
    command_.wz_rad_s = mr::component::math::clamp_scalar(
        command.wz_rad_s, -config_.max_body_velocity.wz_rad_s,
        config_.max_body_velocity.wz_rad_s);
    return ChassisStatus::kOk;
  }

  ChassisStatus Step(float dt_s) {
    if (state_ != ChassisState::kEnabled) {
      return ChassisStatus::kInvalidState;
    }
    if (output_pending_) {
      return ChassisStatus::kInvalidState;
    }
    if (!feedback_updated_) {
      return ChassisStatus::kNoFreshFeedback;
    }
    if (!mr::component::math::is_finite_scalar(dt_s) || dt_s < kMinDtS ||
        dt_s > kMaxDtS) {
      EnterFault();
      return ChassisStatus::kInvalidArgument;
    }

    mr::robotics::chassis::WheelSpeeds wheel_speed_refs{};
    if (!kinematics_.InverseKinematics(command_, wheel_speed_refs)) {
      EnterFault();
      return ChassisStatus::kKinematicsError;
    }
    if (!mr::robotics::chassis::ChassisKinematics::ScaleWheelSpeedsToLimit(
            wheel_speed_refs, config_.max_wheel_speed_mps)) {
      EnterFault();
      return ChassisStatus::kKinematicsError;
    }

    output_ = {};
    output_.wheel_speed_ref_mps = wheel_speed_refs;
    for (std::size_t index = 0; index < kWheelCount; ++index) {
      const auto result = wheel_velocity_pid_[index].Step(
          wheel_speed_refs[index], feedback_.wheels[index].linear_velocity_mps,
          dt_s);
      if (!result.valid || wheels_[index]->SetTorque(result.output) != 0 ||
          wheels_[index]->UpdateCommand() != 0) {
        output_ = {};
        EnterFault();
        return ChassisStatus::kDeviceError;
      }
      output_.wheel_output_torque_nm[index] = result.output;
    }

    output_.valid = true;
    output_pending_ = true;
    feedback_updated_ = false;
    return ChassisStatus::kOk;
  }

  ChassisStatus CommitOutput() {
    if (state_ != ChassisState::kEnabled || !output_pending_ ||
        !output_.valid) {
      return ChassisStatus::kInvalidState;
    }
    for (mr::wheel::WheelDevice* wheel : wheels_) {
      if (wheel->CommitCommand() != 0) {
        EnterFault();
        return ChassisStatus::kDeviceError;
      }
    }
    output_pending_ = false;
    return ChassisStatus::kOk;
  }

  ChassisStatus Relax() {
    if (state_ == ChassisState::kUnregistered) {
      return ChassisStatus::kInvalidState;
    }
    const bool success = RelaxRegisteredWheels();
    ResetControlState();
    state_ = success ? ChassisState::kRegistered : ChassisState::kFault;
    return success ? ChassisStatus::kOk : ChassisStatus::kDeviceError;
  }

  ChassisState state() const { return state_; }
  mr::robotics::chassis::ChassisModel model() const {
    return kinematics_.model();
  }
  const ChassisFeedback& feedback() const { return feedback_; }
  const ChassisOutput& output() const { return output_; }

 private:

  static constexpr float kMinDtS = 1.0e-6f;
  static constexpr float kMaxDtS = 0.1f;

  static bool IsValidConfig(const ChassisConfig& config) {
    const auto& limit = config.max_body_velocity;
    return mr::component::math::is_finite_scalar(limit.vx_mps) &&
           limit.vx_mps > 0.0f &&
           mr::component::math::is_finite_scalar(limit.vy_mps) &&
           limit.vy_mps > 0.0f &&
           mr::component::math::is_finite_scalar(limit.wz_rad_s) &&
           limit.wz_rad_s > 0.0f &&
           mr::component::math::is_finite_scalar(
               config.max_wheel_speed_mps) &&
           config.max_wheel_speed_mps > 0.0f &&
           config.feedback_timeout_ms > 0U &&
           config.feedback_timeout_ms <= INT32_MAX &&
           config.wheel_velocity_pid.output_limit.enabled() &&
           mr::comp::cntlr::pid::IsConfigValid(config.wheel_velocity_pid);
  }

  void EnterFault() {
    (void)RelaxRegisteredWheels();
    ResetControlState();
    state_ = ChassisState::kFault;
  }

  bool RelaxRegisteredWheels() {
    bool success = true;
    for (std::size_t index = 0; index < registered_wheels_; ++index) {
      mr::wheel::WheelDevice* wheel = wheels_[index];
      if (wheel == nullptr) {
        success = false;
        continue;
      }
      const bool relaxed = wheel->Relax() == 0;
      const bool committed = wheel->CommitCommand() == 0;
      success = success && relaxed && committed;
    }
    return success;
  }

  void ResetControlState() {
    command_ = {};
    feedback_ = {};
    output_ = {};
    output_pending_ = false;
    feedback_updated_ = false;
    for (auto& controller : wheel_velocity_pid_) {
      controller.Reset();
    }
  }

  ChassisConfig config_{};
  mr::robotics::chassis::ChassisKinematics kinematics_{};
  std::array<mr::wheel::WheelDevice*, kWheelCount> wheels_{};
  std::array<mr::comp::cntlr::pid, kWheelCount> wheel_velocity_pid_{};
  mr::robotics::chassis::PlanarVelocity command_{};
  ChassisFeedback feedback_{};
  ChassisOutput output_{};
  ChassisState state_ = ChassisState::kUnregistered;
  std::size_t registered_wheels_ = 0U;
  bool feedback_updated_ = false;
  bool output_pending_ = false;
};

}  // namespace mr::module::chassis
