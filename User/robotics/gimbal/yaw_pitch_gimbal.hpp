#pragma once

#include <cmath>

#include "component/math/scalar.hpp"
#include "robotics/gimbal/types.hpp"

namespace mr::robotics::gimbal {

// Base frame: +X forward, +Y left, +Z up. Positive yaw turns toward +Y;
// positive pitch raises the line of sight. This model assumes intersecting axes.
class YawPitchGimbal final {
 public:
  struct Config {
    float min_yaw_rad = -mr::component::math::kPi;
    float max_yaw_rad = mr::component::math::kPi;
    float min_pitch_rad = -mr::component::math::kHalfPi;
    float max_pitch_rad = mr::component::math::kHalfPi;
    float vertical_singularity_epsilon = 1.0e-6f;
  };

  bool Configure(const Config& config) {
    if (!mr::component::math::is_finite_scalar(config.min_yaw_rad) ||
        !mr::component::math::is_finite_scalar(config.max_yaw_rad) ||
        !mr::component::math::is_finite_scalar(config.min_pitch_rad) ||
        !mr::component::math::is_finite_scalar(config.max_pitch_rad) ||
        !mr::component::math::is_finite_scalar(config.vertical_singularity_epsilon) ||
        config.min_yaw_rad >= config.max_yaw_rad ||
        config.min_pitch_rad >= config.max_pitch_rad ||
        config.vertical_singularity_epsilon <= 0.0f ||
        config.vertical_singularity_epsilon >= 1.0f)
      return false;
    config_ = config;
    configured_ = true;
    return true;
  }

  bool IsValid() const { return configured_; }
  const Config& config() const { return config_; }

  Status DirectionToAngles(const Direction& direction, Angles& angles) const {
    angles = {};
    if (!configured_) return Status::kInvalidModel;
    if (!IsFinite(direction)) return Status::kInvalidInput;
    const float horizontal = std::hypot(direction.x, direction.y);
    const float magnitude = std::hypot(horizontal, direction.z);
    if (!std::isfinite(magnitude) || magnitude <= config_.vertical_singularity_epsilon)
      return Status::kInvalidInput;
    if (horizontal / magnitude <= config_.vertical_singularity_epsilon)
      return Status::kSingularity;  // Yaw is undefined at the vertical poles.

    angles.yaw_rad = std::atan2(direction.y, direction.x);
    angles.pitch_rad = std::atan2(direction.z, horizontal);
    return IsWithinLimits(angles) ? Status::kOk : Status::kLimitViolation;
  }

  Status AnglesToDirection(const Angles& angles, Direction& direction) const {
    direction = {};
    if (!configured_) return Status::kInvalidModel;
    if (!IsFinite(angles)) return Status::kInvalidInput;
    if (!IsWithinLimits(angles)) return Status::kLimitViolation;
    const float cp = std::cos(angles.pitch_rad);
    direction = {cp * std::cos(angles.yaw_rad),
                 cp * std::sin(angles.yaw_rad),
                 std::sin(angles.pitch_rad)};
    return IsFinite(direction) ? Status::kOk : Status::kInvalidInput;
  }

 private:
  bool IsWithinLimits(const Angles& angles) const {
    return angles.yaw_rad >= config_.min_yaw_rad &&
           angles.yaw_rad <= config_.max_yaw_rad &&
           angles.pitch_rad >= config_.min_pitch_rad &&
           angles.pitch_rad <= config_.max_pitch_rad;
  }

  Config config_{};
  bool configured_ = false;
};

}  // namespace mr::robotics::gimbal
