#include "module/gimbal/yaw_pitch_pitch_gimbal.hpp"

namespace mr::module::gimbal {

Status YawPitchPitchGimbal::Register(const Config& config) {
  if (lifecycle() != Lifecycle::kUninitialized) return Status::kInvalidState;
  for (float angle : {config.low_fire_pitch1_rad, config.high_fire_pitch1_rad})
    if (!component::math::is_finite_scalar(angle) ||
        angle < config.limits[1].min_rad || angle > config.limits[1].max_rad)
      return Status::kInvalidConfiguration;
  const Status result = joints_.Register(config);
  if (result == Status::kOk) {
    low_fire_pitch1_rad_ = config.low_fire_pitch1_rad;
    high_fire_pitch1_rad_ = config.high_fire_pitch1_rad;
  }
  return result;
}

Status YawPitchPitchGimbal::Enable() {
  const Status result = joints_.Enable();
  if (result == Status::kOk)
    target_ = {feedback().angles[0], feedback().angles[1], feedback().angles[2]};
  return result;
}

Status YawPitchPitchGimbal::Disable() { return joints_.Disable(); }

Status YawPitchPitchGimbal::SetMode(Mode mode) {
  if (lifecycle() != Lifecycle::kReady && lifecycle() != Lifecycle::kEnabled)
    return Status::kInvalidState;
  switch (mode) {
    case Mode::kNormal:
    case Mode::kLowFire:
    case Mode::kHighFire: mode_ = mode; return Status::kOk;
  }
  return Status::kInvalidArgument;
}

Status YawPitchPitchGimbal::SetTarget(const Target& target) {
  if (lifecycle() != Lifecycle::kEnabled) return Status::kInvalidState;
  if (!joints_.IsWithinLimits({target.yaw_rad, target.pitch1_rad, target.pitch2_rad}))
    return Status::kInvalidArgument;
  target_ = target;
  return Status::kOk;
}

Status YawPitchPitchGimbal::Update(float dt_s) {
  const Status result = joints_.BeginUpdate(dt_s);
  if (result != Status::kOk) return result;
  switch (mode_) {
    case Mode::kNormal: return UpdateNormal(dt_s);
    case Mode::kLowFire: return UpdateLowFire(dt_s);
    case Mode::kHighFire: return UpdateHighFire(dt_s);
  }
  return Status::kInvalidState;
}

Status YawPitchPitchGimbal::UpdateNormal(float /*dt_s*/) {
  return joints_.WriteOutput({target_.yaw_rad, target_.pitch1_rad, target_.pitch2_rad});
}

Status YawPitchPitchGimbal::UpdateLowFire(float /*dt_s*/) {
  return joints_.WriteOutput({target_.yaw_rad, low_fire_pitch1_rad_, target_.pitch2_rad});
}

Status YawPitchPitchGimbal::UpdateHighFire(float /*dt_s*/) {
  return joints_.WriteOutput({target_.yaw_rad, high_fire_pitch1_rad_, target_.pitch2_rad});
}

}  // namespace mr::module::gimbal
