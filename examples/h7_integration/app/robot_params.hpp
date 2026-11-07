#pragma once

#include <array>
#include <cstdint>
#include "device/joint/joint.hpp"
#include "module/chassis/chassis.hpp"

// Project parameters. Edit this file directly; no generator owns it.
// Demonstration values only: these are not zzrobot's calibrated parameters.
namespace app::params {
inline constexpr bool kAllowMotorTx = false;
inline constexpr uint32_t kControlPeriodUs = 1000;
static_assert(kControlPeriodUs > 0, "Control period must be positive");
inline constexpr uint32_t kTelemetryPeriodMs = 100;
inline constexpr int kArmJointCount = 3;
inline constexpr uint16_t kWheelFeedbackIds[] = {0x201, 0x202, 0x203, 0x204};
inline constexpr float kWheelRadiusM = 0.1f;
inline constexpr uint32_t kMotorFeedbackTimeoutMs = 20;
inline constexpr float kExternalRatio = 1.0f;  // Only additional mechanical reduction.

inline mr::module::chassis::ChassisConfig ChassisConfig(uint32_t period_us) {
  mr::module::chassis::ChassisConfig config;
  config.kinematics.model = mr::robotics::chassis::ChassisModel::kMecanum;
  config.kinematics.geometry = {0.4f, 0.4f};  // Half wheelbase / half trackwidth, metres.
  config.max_body_velocity = {2.0f, 2.0f, 2.0f};  // m/s, m/s, rad/s.
  config.max_wheel_speed_mps = 3.0f;
  config.feedback_timeout_ms = kMotorFeedbackTimeoutMs;
  config.wheel_velocity_pid.gains = {1.0f, 1.0f, 0.0f, 0.0f};
  config.wheel_velocity_pid.output_limit = mr::comp::cntlr::SymmetricLimit::Abs(2.0f);
  config.wheel_velocity_pid.control_freq_hz = 1000000.0f / period_us;
  return config;
}

struct JointGains { float kp, kd, effort_scale; };
struct JointParameters {
  uint16_t control_id, feedback_id;
  const char* name;
  float zero_rad, min_rad, max_rad, max_velocity_rad_s, max_acceleration_rad_s2;
  std::array<float, 3> axis, translation_m, rpy_rad;
  JointGains gains;
};
inline constexpr std::array<JointParameters, kArmJointCount> kJoints{{
    {0x11, 0x321, "shoulder", 0, -1, 1, 1, 2, {0, 0, 1}, {0.1f, 0, 0}, {0, 0, 0}, {5, 0.2f, 0}},
    {0x12, 0x322, "elbow",    0, -1, 1, 1, 2, {0, 0, 1}, {0.1f, 0, 0}, {0, 0, 0}, {5, 0.2f, 0}},
    {0x13, 0x323, "wrist",    0, -1, 1, 1, 2, {0, 0, 1}, {0.1f, 0, 0}, {0, 0, 0}, {5, 0.2f, 0}},
}};
inline constexpr float kMitPositionRangeRad = 12.5f;
inline constexpr float kMitVelocityRangeRadS = 30.0f;
inline constexpr float kMitTorqueRangeNm = 10.0f;
inline constexpr std::array<float, 3> kBaseTranslationM{}, kBaseRpyRad{};
inline constexpr std::array<float, 3> kToolTranslationM{}, kToolRpyRad{};
inline constexpr std::array<float, 3> kGravityWorld{};  // Gravity compensation disabled.
}  // namespace app::params
