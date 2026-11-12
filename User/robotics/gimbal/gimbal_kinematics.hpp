#pragma once

#include <cstdint>

#include "robotics/gimbal/yaw_pitch_gimbal.hpp"

namespace mr::robotics::gimbal {

enum class Model : uint8_t { kYawPitch = 0 };

struct KinematicsConfig {
  Model model = Model::kYawPitch;
  YawPitchGimbal::Config yaw_pitch{};
};

// Fixed-storage model selection point; adding a gimbal geometry does not alter
// the module interface or introduce runtime allocation.
class GimbalKinematics final {
 public:
  bool Configure(const KinematicsConfig& config) {
    if (config.model != Model::kYawPitch || !yaw_pitch_.Configure(config.yaw_pitch))
      return false;
    model_ = config.model;
    configured_ = true;
    return true;
  }

  Model model() const { return model_; }
  bool IsValid() const { return configured_; }
  Status DirectionToAngles(const Direction& direction, Angles& angles) const {
    if (!configured_) { angles = {}; return Status::kInvalidModel; }
    return yaw_pitch_.DirectionToAngles(direction, angles);
  }
  Status AnglesToDirection(const Angles& angles, Direction& direction) const {
    if (!configured_) { direction = {}; return Status::kInvalidModel; }
    return yaw_pitch_.AnglesToDirection(angles, direction);
  }

 private:
  Model model_ = Model::kYawPitch;
  YawPitchGimbal yaw_pitch_{};
  bool configured_ = false;
};

}  // namespace mr::robotics::gimbal
