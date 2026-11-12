#pragma once

#include <cmath>

#include "component/math/scalar.hpp"
#include "robotics/gimbal/types.hpp"

namespace mr::robotics::gimbal {

// Base frame: +X forward, +Y left, +Z up. The pitch axes are parallel and
// intersecting; positive pitch angles add in the world elevation.
class YawPitchPitchGimbal final {
 public:
  struct Config {
    float min_yaw_rad = -mr::component::math::kPi;
    float max_yaw_rad = mr::component::math::kPi;
    float min_pitch1_rad = 0.0f;
    float max_pitch1_rad = mr::component::math::kHalfPi;
    float min_pitch2_rad = -mr::component::math::kHalfPi;
    float max_pitch2_rad = mr::component::math::kHalfPi;
    float vertical_singularity_epsilon = 1.0e-6f;
  };

  bool Configure(const Config& config) {
    if (!mr::component::math::is_finite_scalar(config.min_yaw_rad) ||
        !mr::component::math::is_finite_scalar(config.max_yaw_rad) ||
        !mr::component::math::is_finite_scalar(config.min_pitch1_rad) ||
        !mr::component::math::is_finite_scalar(config.max_pitch1_rad) ||
        !mr::component::math::is_finite_scalar(config.min_pitch2_rad) ||
        !mr::component::math::is_finite_scalar(config.max_pitch2_rad) ||
        !mr::component::math::is_finite_scalar(config.vertical_singularity_epsilon) ||
        config.min_yaw_rad >= config.max_yaw_rad ||
        config.min_pitch1_rad >= config.max_pitch1_rad ||
        config.min_pitch2_rad >= config.max_pitch2_rad ||
        config.vertical_singularity_epsilon <= 0.0f ||
        config.vertical_singularity_epsilon >= 1.0f)
      return false;
    config_ = config;
    configured_ = true;
    return true;
  }

  bool IsValid() const { return configured_; }
  const Config& config() const { return config_; }

  Status DirectionToAngles(const Direction& direction, float pitch1_rad,
                           YawPitchPitchAngles& angles) const {
    angles = {};
    if (!configured_) return Status::kInvalidModel;
    if (!IsFinite(direction) || !mr::component::math::is_finite_scalar(pitch1_rad))
      return Status::kInvalidInput;
    if (pitch1_rad < config_.min_pitch1_rad || pitch1_rad > config_.max_pitch1_rad)
      return Status::kLimitViolation;

    const float horizontal = std::hypot(direction.x, direction.y);
    const float magnitude = std::hypot(horizontal, direction.z);
    if (!std::isfinite(magnitude) || magnitude <= config_.vertical_singularity_epsilon)
      return Status::kInvalidInput;
    if (horizontal / magnitude <= config_.vertical_singularity_epsilon)
      return Status::kSingularity;

    angles.yaw_rad = std::atan2(direction.y, direction.x);
    angles.pitch1_rad = pitch1_rad;
    angles.pitch2_rad = std::atan2(direction.z, horizontal) - pitch1_rad;
    return IsWithinLimits(angles) ? Status::kOk : Status::kLimitViolation;
  }

  Status AnglesToDirection(const YawPitchPitchAngles& angles,
                           Direction& direction) const {
    direction = {};
    if (!configured_) return Status::kInvalidModel;
    if (!IsFinite(angles)) return Status::kInvalidInput;
    if (!IsWithinLimits(angles)) return Status::kLimitViolation;
    const float elevation = angles.pitch1_rad + angles.pitch2_rad;
    const float cp = std::cos(elevation);
    direction = {cp * std::cos(angles.yaw_rad),
                 cp * std::sin(angles.yaw_rad),
                 std::sin(elevation)};
    return IsFinite(direction) ? Status::kOk : Status::kInvalidInput;
  }

  bool IsWithinLimits(const YawPitchPitchAngles& angles) const {
    return configured_ && IsFinite(angles) &&
           angles.yaw_rad >= config_.min_yaw_rad && angles.yaw_rad <= config_.max_yaw_rad &&
           angles.pitch1_rad >= config_.min_pitch1_rad &&
           angles.pitch1_rad <= config_.max_pitch1_rad &&
           angles.pitch2_rad >= config_.min_pitch2_rad &&
           angles.pitch2_rad <= config_.max_pitch2_rad;
  }

 private:
  Config config_{};
  bool configured_ = false;
};

}  // namespace mr::robotics::gimbal
