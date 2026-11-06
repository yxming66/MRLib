#pragma once

#include "../core/types.hpp"
#include "component/math/matrix.hpp"
#include "component/math/scalar.hpp"
#include "robotics/core/rotation.hpp"
#include "robotics/core/transform.hpp"

namespace mr::robotics::arm::kinematics {

struct PoseError {
  // World-frame [tool-origin position error; rotation-vector error], not SE(3) log.
  Twist6 twist;
  Scalar position_norm;
  Scalar orientation_norm;
  bool valid;

  PoseError()
      : twist(mr::component::math::matrix::zeros<6, 1>()),
        position_norm(0.0f),
        orientation_norm(0.0f),
        valid(false) {}
};

inline PoseError ComputePoseError(const Transform& target, const Transform& current) {
  PoseError result;
  if (!IsTransformValid(target) || !IsTransformValid(current)) {
    return result;
  }

  const Vec3 position_error =
      mr::robotics::core::TranslationOf(target) - mr::robotics::core::TranslationOf(current);
  const Rotation relative_rotation =
      mr::robotics::core::RotationOf(target) * mr::robotics::core::RotationOf(current).transpose();
  const mr::robotics::core::Vec4 axis_angle =
      mr::robotics::core::AxisAngleFromRotation(relative_rotation);
  const Vec3 orientation_error = axis_angle.block<3, 1>(0, 0) * axis_angle[3][0];

  for (int index = 0; index < 3; ++index) {
    result.twist[index][0] = position_error[index][0];
    result.twist[index + 3][0] = orientation_error[index][0];
  }
  result.position_norm = position_error.norm();
  result.orientation_norm = orientation_error.norm();
  result.valid = mr::component::math::is_finite_scalar(result.position_norm) &&
                 mr::component::math::is_finite_scalar(result.orientation_norm);
  if (!result.valid) {
    result.twist = mr::component::math::matrix::zeros<6, 1>();
    result.position_norm = 0.0f;
    result.orientation_norm = 0.0f;
  }
  return result;
}

}  // namespace mr::robotics::arm::kinematics
