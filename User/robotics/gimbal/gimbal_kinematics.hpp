#pragma once

#include <cstddef>
#include <cstdint>

#include "robotics/gimbal/yaw_pitch_gimbal.hpp"
#include "robotics/gimbal/yaw_pitch_pitch_gimbal.hpp"

namespace mr::robotics::gimbal {

enum class Model : uint8_t { kYawPitch = 0, kYawPitchPitch = 1 };

struct KinematicsConfig {
  Model model = Model::kYawPitch;
  YawPitchGimbal::Config yaw_pitch{};
  YawPitchPitchGimbal::Config yaw_pitch_pitch{};
};

// Fixed-storage model selection point; adding a gimbal geometry does not alter
// the module interface or introduce runtime allocation.
class GimbalKinematics final {
 public:
  bool Configure(const KinematicsConfig& config) {
    bool valid = false;
    switch (config.model) {
      case Model::kYawPitch: valid = yaw_pitch_.Configure(config.yaw_pitch); break;
      case Model::kYawPitchPitch: valid = yaw_pitch_pitch_.Configure(config.yaw_pitch_pitch); break;
      default: return false;
    }
    if (!valid) return false;
    model_ = config.model;
    configured_ = true;
    return true;
  }

  Model model() const { return model_; }
  bool IsValid() const { return configured_; }
  std::size_t AxisCount() const {
    return model_ == Model::kYawPitchPitch ? 3U : 2U;
  }
  Status DirectionToAngles(const Direction& direction, float pitch1_rad,
                           JointAngles& angles) const {
    angles = {};
    if (!configured_) return Status::kInvalidModel;
    if (model_ == Model::kYawPitch) {
      Angles two_axis{};
      const Status result = yaw_pitch_.DirectionToAngles(direction, two_axis);
      angles[0] = two_axis.yaw_rad;
      angles[1] = two_axis.pitch_rad;
      return result;
    }
    YawPitchPitchAngles three_axis{};
    const Status result = yaw_pitch_pitch_.DirectionToAngles(direction, pitch1_rad, three_axis);
    angles = {three_axis.yaw_rad, three_axis.pitch1_rad, three_axis.pitch2_rad};
    return result;
  }
  Status AnglesToDirection(const JointAngles& angles, Direction& direction) const {
    direction = {};
    if (!configured_) return Status::kInvalidModel;
    if (model_ == Model::kYawPitch) {
      return yaw_pitch_.AnglesToDirection({angles[0], angles[1]}, direction);
    }
    return yaw_pitch_pitch_.AnglesToDirection(
        {angles[0], angles[1], angles[2]}, direction);
  }
  bool IsWithinLimits(const JointAngles& angles) const {
    if (!configured_) return false;
    if (model_ == Model::kYawPitch)
      return angles[0] >= yaw_pitch_.config().min_yaw_rad &&
             angles[0] <= yaw_pitch_.config().max_yaw_rad &&
             angles[1] >= yaw_pitch_.config().min_pitch_rad &&
             angles[1] <= yaw_pitch_.config().max_pitch_rad;
    return yaw_pitch_pitch_.IsWithinLimits(
        {angles[0], angles[1], angles[2]});
  }

 private:
  Model model_ = Model::kYawPitch;
  YawPitchGimbal yaw_pitch_{};
  YawPitchPitchGimbal yaw_pitch_pitch_{};
  bool configured_ = false;
};

}  // namespace mr::robotics::gimbal
