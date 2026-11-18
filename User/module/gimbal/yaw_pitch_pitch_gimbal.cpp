#include "module/gimbal/yaw_pitch_pitch_gimbal.hpp"
#include <algorithm>
#include <cmath>

namespace mr::module::gimbal {
namespace {
float Approach(float current, float goal, float step) {
  return current + std::clamp(goal - current, -step, step);
}
bool Within(float angle, const AxisLimits& limits) {
  return component::math::is_finite_scalar(angle) && angle >= limits.min_rad && angle <= limits.max_rad;
}
}

Status YawPitchPitchGimbal::Register(const Config& config) {
  if (lifecycle() != Lifecycle::kUninitialized) return Status::kInvalidState;
  // Every posture needs pitch2=pitch1 for a level head. Never silently clamp
  // an unreachable compensation angle or expand a user's mechanical limits.
  for (float angle : {0.0f, config.low_fire_pitch1_rad, config.high_fire_pitch1_rad})
    if (!Within(angle, config.limits[1]) || !Within(angle, config.limits[2]))
      return Status::kInvalidConfiguration;
  if (!Within(0.0f, config.limits[0])) return Status::kInvalidConfiguration;
  for (float value : {config.posture_speed_rad_s, config.leveling_speed_rad_s,
                     config.fold_yaw_speed_rad_s, config.position_tolerance_rad,
                     config.settle_time_s})
    if (!component::math::is_positive_scalar(value)) return Status::kInvalidConfiguration;
  const Status result = joints_.Register(config);
  if (result == Status::kOk) config_ = config;
  return result;
}

Status YawPitchPitchGimbal::Enable() {
  const Status result = joints_.Enable();
  if (result == Status::kOk) {
    target_ = {feedback().angles[0], feedback().angles[1], feedback().angles[2]};
    transition_requested_ = true;
    stage_ = mode_ == Mode::kFolded ? Stage::kFoldYaw : Stage::kLeveling;
  }
  return result;
}

Status YawPitchPitchGimbal::Disable() {
  const Status result = joints_.Disable();
  stage_ = Stage::kInactive;
  transition_requested_ = false;
  return result;
}

Status YawPitchPitchGimbal::SetMode(Mode mode) {
  if (lifecycle() != Lifecycle::kReady && lifecycle() != Lifecycle::kEnabled)
    return Status::kInvalidState;
  switch (mode) {
    case Mode::kFolded:
    case Mode::kLowFire:
    case Mode::kHighFire:
      if (mode != mode_) {
        mode_ = mode;
        transition_requested_ = true;
        if (lifecycle() == Lifecycle::kEnabled)
          stage_ = mode_ == Mode::kFolded ? Stage::kFoldYaw : Stage::kLeveling;
      }
      return Status::kOk;
  }
  return Status::kInvalidArgument;
}

Status YawPitchPitchGimbal::SetTarget(const Target& target) {
  if (lifecycle() != Lifecycle::kEnabled) return Status::kInvalidState;
  const std::array<float, 3> angles{target.yaw_rad, target.pitch1_rad, target.pitch2_rad};
  if (!joints_.AreFinite(angles))
    return Status::kInvalidArgument;
  const auto bounded = joints_.ClampAngles(angles);
  target_ = {bounded[0], bounded[1], bounded[2]};
  return Status::kOk;
}

float YawPitchPitchGimbal::PostureAngle() const {
  switch (mode_) {
    case Mode::kLowFire: return config_.low_fire_pitch1_rad;
    case Mode::kHighFire: return config_.high_fire_pitch1_rad;
    case Mode::kFolded: return 0.0f;
  }
  return 0.0f;
}

void YawPitchPitchGimbal::BeginTransition() {
  transition_command_ = feedback().angles;
  settled_s_ = 0.0f;
  stage_ = mode_ == Mode::kFolded ? Stage::kFoldYaw : Stage::kLeveling;
  transition_requested_ = false;
}

bool YawPitchPitchGimbal::IsLevel() const {
  return std::fabs(head_pitch_rad()) <= config_.position_tolerance_rad;
}

Status YawPitchPitchGimbal::Update(float dt_s) {
  const Status result = joints_.BeginUpdate(dt_s);
  if (result != Status::kOk) return result;
  if (transition_requested_) BeginTransition();
  if (stage_ == Stage::kWorking)
    return joints_.WriteOutput({target_.yaw_rad, PostureAngle(), target_.pitch2_rad});
  if (stage_ == Stage::kFolded)
    return joints_.WriteOutput({0.0f, 0.0f, 0.0f});
  if (mode_ == Mode::kFolded) return UpdateFold(dt_s);
  return UpdateTransition(dt_s);
}

Status YawPitchPitchGimbal::UpdateFold(float dt_s) {
  const auto& actual = feedback().angles;
  const float tolerance = config_.position_tolerance_rad;
  transition_command_[0] = Approach(transition_command_[0], 0.0f,
                                     config_.fold_yaw_speed_rad_s * dt_s);
  const bool yaw_ready = std::fabs(transition_command_[0]) < 1.0e-6f &&
      std::fabs(actual[0]) <= tolerance;
  const bool level = std::fabs(head_pitch_rad()) <= tolerance;
  if (stage_ == Stage::kFoldYaw) {
    // Return yaw and level the head concurrently; hold pitch1 until both settle.
    transition_command_[2] = Approach(transition_command_[2], actual[1],
                                       config_.leveling_speed_rad_s * dt_s);
    settled_s_ = yaw_ready && level ? settled_s_ + dt_s : 0.0f;
    if (settled_s_ >= config_.settle_time_s) {
      stage_ = Stage::kFoldPitch;
      settled_s_ = 0.0f;
    }
  } else if (stage_ == Stage::kFoldPitch) {
    // Both joints return directly to zero at the same target speed. No head
    // leveling gate or feedback-following compensation during this phase.
    for (size_t i = 1; i < 3; ++i)
      transition_command_[i] = Approach(transition_command_[i], 0.0f,
                                         config_.posture_speed_rad_s * dt_s);
    bool ready = yaw_ready;
    for (size_t i = 1; i < 3; ++i)
      ready = ready && std::fabs(transition_command_[i]) < 1.0e-6f &&
          std::fabs(actual[i]) <= tolerance;
    settled_s_ = ready ? settled_s_ + dt_s : 0.0f;
    if (settled_s_ >= config_.settle_time_s) {
      target_ = {};
      transition_command_ = {};
      stage_ = Stage::kFolded;
    }
  }
  return joints_.WriteOutput(transition_command_);
}

Status YawPitchPitchGimbal::UpdateTransition(float dt_s) {
  const auto& actual = feedback().angles;
  const float goal = PostureAngle();
  transition_command_[0] = target_.yaw_rad;
  if (stage_ == Stage::kLeveling) {
    // Hold pitch1 until the head has returned to horizontal before work motion.
    transition_command_[2] = Approach(transition_command_[2], actual[1],
                                        config_.leveling_speed_rad_s * dt_s);
    settled_s_ = IsLevel() ? settled_s_ + dt_s : 0.0f;
    if (settled_s_ >= config_.settle_time_s) {
      stage_ = Stage::kMoving;
      settled_s_ = 0.0f;
    }
  } else if (stage_ == Stage::kMoving) {
    // After initial leveling, both joints move directly to the posture angle
    // at the same target speed, independent of head angle and motor lag.
    for (size_t i = 1; i < 3; ++i)
      transition_command_[i] = Approach(transition_command_[i], goal,
                                         config_.posture_speed_rad_s * dt_s);
    if (std::fabs(transition_command_[1] - goal) < 1.0e-6f &&
        std::fabs(transition_command_[2] - goal) < 1.0e-6f) {
      stage_ = Stage::kSettling;
      settled_s_ = 0.0f;
    }
  } else if (stage_ == Stage::kSettling) {
    const bool at_posture = std::fabs(actual[1] - goal) <= config_.position_tolerance_rad &&
        std::fabs(actual[2] - goal) <= config_.position_tolerance_rad;
    settled_s_ = at_posture ? settled_s_ + dt_s : 0.0f;
    if (settled_s_ >= config_.settle_time_s) {
      // Discard pitch2 targets received while transitioning. Manual control
      // resumes from the level angle, without replaying accumulated stick input.
      target_.pitch1_rad = goal;
      target_.pitch2_rad = goal;
      transition_command_[1] = transition_command_[2] = goal;
      stage_ = Stage::kWorking;
    }
  }
  return joints_.WriteOutput(transition_command_);
}
}  // namespace mr::module::gimbal
