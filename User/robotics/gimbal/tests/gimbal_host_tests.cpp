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
  JointAngles angles{};
  Direction direction{};
  assert(model.DirectionToAngles({1.0f, 0.0f, 0.0f}, 0.0f, angles) == Status::kInvalidModel);
  assert(model.Configure({}));
  assert(model.DirectionToAngles({1.0f, 1.0f, 1.0f}, 0.0f, angles) == Status::kOk);
  assert(Near(angles[0], 0.78539816f));
  assert(Near(angles[1], 0.61547971f));
  assert(model.AnglesToDirection(angles, direction) == Status::kOk);
  const float magnitude = std::sqrt(3.0f);
  assert(Near(direction.x, 1.0f / magnitude));
  assert(Near(direction.y, 1.0f / magnitude));
  assert(Near(direction.z, 1.0f / magnitude));
  assert(model.DirectionToAngles({0.0f, 0.0f, 0.0f}, 0.0f, angles) == Status::kInvalidInput);
  assert(model.DirectionToAngles({0.0f, 0.0f, 1.0f}, 0.0f, angles) == Status::kSingularity);
  assert(model.DirectionToAngles({std::numeric_limits<float>::quiet_NaN(), 1, 0}, 0.0f, angles) ==
         Status::kInvalidInput);
  assert(model.DirectionToAngles({-1.0f, 0.0f, 0.0f}, 0.0f, angles) == Status::kOk);
  assert(Near(std::fabs(angles[0]), 3.14159265f));
  assert(model.AnglesToDirection({4.0f, 0.0f, 0.0f}, direction) == Status::kLimitViolation);

  YawPitchGimbal limited;
  YawPitchGimbal::Config config;
  config.min_yaw_rad = -1.0f;
  config.max_yaw_rad = 1.0f;
  assert(limited.Configure(config));
  Angles yaw_pitch_angles{};
  assert(limited.DirectionToAngles({0.0f, 1.0f, 0.0f}, yaw_pitch_angles) ==
         Status::kLimitViolation);
  config.min_yaw_rad = 1.0f;
  config.max_yaw_rad = -1.0f;
  assert(!limited.Configure(config));

  KinematicsConfig triple_config;
  triple_config.model = Model::kYawPitchPitch;
  triple_config.yaw_pitch_pitch.min_pitch1_rad = 0.0f;
  triple_config.yaw_pitch_pitch.max_pitch1_rad = mr::component::math::kHalfPi;
  triple_config.yaw_pitch_pitch.min_pitch2_rad = -mr::component::math::kHalfPi;
  triple_config.yaw_pitch_pitch.max_pitch2_rad = mr::component::math::kHalfPi;
  assert(model.Configure(triple_config) && model.AxisCount() == 3);
  const YawPitchPitchAngles nested{0.4f, 0.7f, -0.2f};
  JointAngles triple{nested.yaw_rad, nested.pitch1_rad, nested.pitch2_rad};
  assert(model.AnglesToDirection(triple, direction) == Status::kOk);
  assert(model.DirectionToAngles(direction, nested.pitch1_rad, angles) == Status::kOk);
  assert(Near(angles[0], nested.yaw_rad));
  assert(Near(angles[1], nested.pitch1_rad));
  assert(Near(angles[2], nested.pitch2_rad));

  // The fold coupling theta2=theta1 is preserved by the model's forward map.
  const float fold_angle = mr::component::math::kHalfPi * 0.5f;
  assert(model.AnglesToDirection({0.0f, fold_angle, fold_angle}, direction) == Status::kOk);
  assert(Near(direction.x, 0.0f, 1.0e-5f) && Near(direction.z, 1.0f));
  assert(model.DirectionToAngles(direction, 0.0f, angles) == Status::kSingularity);
}
