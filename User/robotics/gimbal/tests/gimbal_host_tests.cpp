#include <cassert>
#include <cmath>
#include <limits>

#include "robotics/gimbal/gimbal_kinematics.hpp"

using namespace mr::robotics::gimbal;

static bool Near(float a, float b, float tolerance = 1.0e-5f) {
  return std::fabs(a - b) <= tolerance;
}

int main() {
  GimbalKinematics model;
  Angles angles{};
  Direction direction{};
  assert(model.DirectionToAngles({1.0f, 0.0f, 0.0f}, angles) == Status::kInvalidModel);
  assert(model.Configure({}));
  assert(model.DirectionToAngles({1.0f, 1.0f, 1.0f}, angles) == Status::kOk);
  assert(Near(angles.yaw_rad, 0.78539816f));
  assert(Near(angles.pitch_rad, 0.61547971f));
  assert(model.AnglesToDirection(angles, direction) == Status::kOk);
  const float magnitude = std::sqrt(3.0f);
  assert(Near(direction.x, 1.0f / magnitude));
  assert(Near(direction.y, 1.0f / magnitude));
  assert(Near(direction.z, 1.0f / magnitude));
  assert(model.DirectionToAngles({0.0f, 0.0f, 0.0f}, angles) == Status::kInvalidInput);
  assert(model.DirectionToAngles({0.0f, 0.0f, 1.0f}, angles) == Status::kSingularity);
  assert(model.DirectionToAngles({std::numeric_limits<float>::quiet_NaN(), 1, 0}, angles) ==
         Status::kInvalidInput);
  assert(model.DirectionToAngles({-1.0f, 0.0f, 0.0f}, angles) == Status::kOk);
  assert(Near(std::fabs(angles.yaw_rad), 3.14159265f));
  assert(model.AnglesToDirection({4.0f, 0.0f}, direction) == Status::kLimitViolation);

  YawPitchGimbal limited;
  YawPitchGimbal::Config config;
  config.min_yaw_rad = -1.0f;
  config.max_yaw_rad = 1.0f;
  assert(limited.Configure(config));
  assert(limited.DirectionToAngles({0.0f, 1.0f, 0.0f}, angles) == Status::kLimitViolation);
  config.min_yaw_rad = 1.0f;
  config.max_yaw_rad = -1.0f;
  assert(!limited.Configure(config));
}
