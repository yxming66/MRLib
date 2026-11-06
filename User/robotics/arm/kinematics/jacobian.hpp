#pragma once

#include "../model/arm_model.hpp"
#include "component/math/matrix.hpp"
#include "forward_kinematics.hpp"

namespace mr::robotics::arm::kinematics {

template <int DoF>
using Jacobian = mr::component::math::Matrix<6, DoF>;

// Geometric Jacobian in the model world frame, at the tool origin. Rows are [vx, vy, vz, wx, wy,
// wz].
template <int DoF>
inline bool TryComputeJacobian(const ArmModel<DoF>& model, const JointVector<DoF>& position,
                               Jacobian<DoF>* output) {
  if (!output) return false;
  *output = {};
  Jacobian<DoF> result = mr::component::math::matrix::zeros<6, DoF>();
  KinematicFrames<DoF> frames;
  if (!ComputeKinematicFrames(model, position, &frames)) {
    return false;
  }

  const Vec3 end_position = mr::robotics::core::TranslationOf(frames.end_effector);
  for (int index = 0; index < DoF; ++index) {
    const JointModel& joint = model.Joint(index);
    Vec3 linear = mr::component::math::matrix::zeros<3, 1>();
    Vec3 angular = mr::component::math::matrix::zeros<3, 1>();
    if (joint.type == JointType::kRevolute) {
      linear = mr::component::math::vector3f::cross(
          frames.joint_axis_world[index], end_position - frames.joint_origin_world[index]);
      angular = frames.joint_axis_world[index];
    } else {
      linear = frames.joint_axis_world[index];
    }

    for (int row = 0; row < 3; ++row) {
      result(row, index) = linear[row][0];
      result(row + 3, index) = angular[row][0];
    }
  }
  if (!mr::component::math::matrix::is_finite(result)) return false;
  *output = result;
  return true;
}

template <int DoF>
inline Jacobian<DoF> ComputeJacobian(const ArmModel<DoF>& model, const JointVector<DoF>& position) {
  Jacobian<DoF> result;
  if (!TryComputeJacobian(model, position, &result))
    result.fill(std::numeric_limits<float>::quiet_NaN());
  return result;
}

template <int DoF>
inline bool IsJacobianFinite(const Jacobian<DoF>& jacobian) {
  return mr::component::math::matrix::is_finite(jacobian);
}

}  // namespace mr::robotics::arm::kinematics
