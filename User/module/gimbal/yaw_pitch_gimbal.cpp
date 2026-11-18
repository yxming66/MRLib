#include "module/gimbal/yaw_pitch_gimbal.hpp"

namespace mr::module::gimbal {

Status YawPitchGimbal::Register(const Config& config) { return joints_.Register(config); }

Status YawPitchGimbal::Enable() {
  const Status result = joints_.Enable();
  if (result == Status::kOk)
    target_ = {feedback().angles[0], feedback().angles[1]};
  return result;
}

Status YawPitchGimbal::Disable() { return joints_.Disable(); }

Status YawPitchGimbal::SetMode(Mode mode) {
  if (lifecycle() != Lifecycle::kReady && lifecycle() != Lifecycle::kEnabled)
    return Status::kInvalidState;
  if (mode != Mode::kNormal) return Status::kInvalidArgument;
  mode_ = mode;
  return Status::kOk;
}

Status YawPitchGimbal::SetTarget(const Target& target) {
  if (lifecycle() != Lifecycle::kEnabled) return Status::kInvalidState;
  const std::array<float, 2> angles{target.yaw_rad, target.pitch_rad};
  if (!joints_.AreFinite(angles))
    return Status::kInvalidArgument;
  const auto bounded = joints_.ClampAngles(angles);
  target_ = {bounded[0], bounded[1]};
  return Status::kOk;
}

Status YawPitchGimbal::Update(float dt_s) {
  const Status result = joints_.BeginUpdate(dt_s);
  if (result != Status::kOk) return result;
  switch (mode_) {
    case Mode::kNormal: return UpdateNormal(dt_s);
  }
  return Status::kInvalidState;
}

Status YawPitchGimbal::UpdateNormal(float /*dt_s*/) {
  return joints_.WriteOutput({target_.yaw_rad, target_.pitch_rad});
}

}  // namespace mr::module::gimbal
