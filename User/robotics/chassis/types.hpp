#pragma once

#include <array>
#include <cstddef>

#include "component/math/scalar.hpp"

namespace mr::robotics::chassis {

/** Body velocity in the chassis frame.
 *
 * +x points forward, +y points left, and +z/+wz use the right-hand rule.
 * Linear values use m/s and angular values use rad/s.
 */
struct PlanarVelocity {
  float vx_mps = 0.0f;
  float vy_mps = 0.0f;
  float wz_rad_s = 0.0f;
};

constexpr std::size_t kWheelCount = 4U;
using WheelSpeeds = std::array<float, kWheelCount>;

inline bool IsFinite(const PlanarVelocity& velocity) {
  return mr::component::math::is_finite_scalar(velocity.vx_mps) &&
         mr::component::math::is_finite_scalar(velocity.vy_mps) &&
         mr::component::math::is_finite_scalar(velocity.wz_rad_s);
}

inline bool IsFinite(const WheelSpeeds& wheel_speeds) {
  return mr::component::math::values_are_finite(wheel_speeds.data(),
                                                wheel_speeds.size());
}

/** Maximum absolute wheel linear speed in the configured wheel order. */
inline float MaxAbsWheelSpeed(const WheelSpeeds& wheel_speeds) {
  return mr::component::math::max_abs_array(wheel_speeds.data(),
                                             wheel_speeds.size());
}

/** Scale all wheel linear speeds proportionally to an absolute limit. */
inline bool ScaleWheelSpeedsToLimit(WheelSpeeds& wheel_speeds,
                                    float max_abs_speed_mps) {
  if (!IsFinite(wheel_speeds) ||
      !mr::component::math::is_positive_scalar(max_abs_speed_mps)) {
    return false;
  }
  return mr::component::math::scale_to_abs_limit(
      wheel_speeds.data(), wheel_speeds.size(), max_abs_speed_mps);
}

}  // namespace mr::robotics::chassis
