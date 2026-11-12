#pragma once

#include <cmath>
#include <cstdint>

namespace mr::robotics::gimbal {

enum class Status : uint8_t {
  kOk = 0,
  kInvalidInput,
  kInvalidModel,
  kLimitViolation,
  kSingularity,
};

struct Angles {
  float yaw_rad = 0.0f;
  float pitch_rad = 0.0f;
};

struct Direction {
  float x = 1.0f;
  float y = 0.0f;
  float z = 0.0f;
};

inline bool IsFinite(const Angles& angles) {
  return std::isfinite(angles.yaw_rad) && std::isfinite(angles.pitch_rad);
}

inline bool IsFinite(const Direction& direction) {
  return std::isfinite(direction.x) && std::isfinite(direction.y) &&
         std::isfinite(direction.z);
}

}  // namespace mr::robotics::gimbal
