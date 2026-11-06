#pragma once

#include <stdint.h>

#include "component/math/matrix.hpp"
#include "robotics/core/rotation.hpp"
#include "robotics/core/transform.hpp"

namespace mr::robotics::arm {

using Scalar = mr::component::math::Scalar;
using Transform = mr::robotics::core::Transform;
using Rotation = mr::robotics::core::Rotation;
using Vec3 = mr::robotics::core::Vec3;
using Twist6 = mr::robotics::core::Twist6;

template <int DoF>
using JointVector = mr::component::math::Matrix<DoF, 1>;

// The homogeneous row is part of the arm pose contract.  Reject malformed
// poses instead of silently repairing command or model data.
inline bool IsTransformValid(const Transform& transform) {
  return mr::robotics::core::IsTransformValid(transform);
}

enum class JointCommandMode : uint8_t {
  kDisabled = 0,
  kPosition,
  kPositionVelocity,
  kPositionVelocityEffort,
  kEffort,
};

template <int DoF>
struct JointState {
  JointVector<DoF> position;
  JointVector<DoF> velocity;
  JointVector<DoF> effort;
  bool valid;

  JointState()
      : position(mr::component::math::matrix::zeros<DoF, 1>()),
        velocity(mr::component::math::matrix::zeros<DoF, 1>()),
        effort(mr::component::math::matrix::zeros<DoF, 1>()),
        valid(false) {}
};

template <int DoF>
struct JointCommand {
  JointVector<DoF> position;
  JointVector<DoF> velocity;
  JointVector<DoF> acceleration;
  JointVector<DoF> effort_feedforward;
  JointCommandMode mode;
  bool valid;

  JointCommand()
      : position(mr::component::math::matrix::zeros<DoF, 1>()),
        velocity(mr::component::math::matrix::zeros<DoF, 1>()),
        acceleration(mr::component::math::matrix::zeros<DoF, 1>()),
        effort_feedforward(mr::component::math::matrix::zeros<DoF, 1>()),
        mode(JointCommandMode::kDisabled),
        valid(false) {}
};

template <int DoF>
inline bool IsJointVectorFinite(const JointVector<DoF>& values) {
  return mr::component::math::matrix::is_finite(values);
}

template <int DoF>
inline bool IsJointStateFinite(const JointState<DoF>& state) {
  return IsJointVectorFinite(state.position) && IsJointVectorFinite(state.velocity) &&
         IsJointVectorFinite(state.effort);
}

template <int DoF>
inline bool IsJointStateUsable(const JointState<DoF>& state) {
  return state.valid && IsJointStateFinite(state);
}

template <int DoF>
inline bool IsJointCommandFinite(const JointCommand<DoF>& command) {
  return IsJointVectorFinite(command.position) && IsJointVectorFinite(command.velocity) &&
         IsJointVectorFinite(command.acceleration) &&
         IsJointVectorFinite(command.effort_feedforward);
}

template <int DoF>
inline bool IsJointCommandUsable(const JointCommand<DoF>& command) {
  return command.valid && IsJointCommandFinite(command);
}

}  // namespace mr::robotics::arm
