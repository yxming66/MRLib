#pragma once

#include <limits>

#include "../model/arm_model.hpp"
#include "component/math/matrix.hpp"
#include "component/math/scalar.hpp"
#include "robotics/core/rotation.hpp"
#include "robotics/core/transform.hpp"

namespace mr::robotics::arm::kinematics {

template <int DoF>
struct KinematicFrames {
  Transform joint_frame[DoF];
  Transform link_frame[DoF];
  Vec3 joint_origin_world[DoF];
  Vec3 joint_axis_world[DoF];
  Transform end_effector;
  bool valid;

  KinematicFrames()
      : joint_frame(),
        link_frame(),
        joint_origin_world(),
        joint_axis_world(),
        end_effector(mr::component::math::matrix::identity<4>()),
        valid(false) {}
};

namespace internal {

inline Vec3 NormalizeAxis(const Vec3& axis) {
  const Scalar norm = axis.norm();
  if (!mr::component::math::is_finite_scalar(norm) ||
      norm <= mr::component::math::kDefaultEpsilon) {
    return mr::component::math::matrix::zeros<3, 1>();
  }
  return axis / norm;
}

inline Transform MakeJointMotion(const JointModel& joint, Scalar position) {
  const Vec3 axis = NormalizeAxis(joint.axis_in_joint_frame);
  const Scalar coordinate = position + joint.position_offset;
  if (joint.type == JointType::kRevolute) {
    return mr::robotics::core::TransformFromAxisAngle(axis, coordinate);
  }
  return mr::robotics::core::TransformFromTranslation(axis * coordinate);
}

}  // namespace internal

template <int DoF>
inline bool ComputeKinematicFrames(const ArmModel<DoF>& model, const JointVector<DoF>& position,
                                   KinematicFrames<DoF>* frames) {
  if (frames == nullptr) {
    return false;
  }
  frames->valid = false;
  if (!IsModelValid(model) || !IsJointVectorFinite(position)) {
    return false;
  }

  Transform current = model.BaseTransform();
  for (int index = 0; index < DoF; ++index) {
    const JointModel& joint = model.Joint(index);
    const Transform joint_frame = current * joint.parent_to_joint;
    const Vec3 axis_local = internal::NormalizeAxis(joint.axis_in_joint_frame);
    if (axis_local.norm() <= mr::component::math::kDefaultEpsilon) {
      return false;
    }

    if (!IsTransformValid(joint_frame)) return false;
    frames->joint_frame[index] = joint_frame;
    frames->joint_origin_world[index] = mr::robotics::core::TranslationOf(joint_frame);
    frames->joint_axis_world[index] = mr::robotics::core::RotationOf(joint_frame) * axis_local;
    frames->link_frame[index] = joint_frame * internal::MakeJointMotion(joint, position[index][0]);
    current = frames->link_frame[index];
    if (!IsTransformValid(current)) return false;
  }

  frames->end_effector = current * model.ToolTransform();
  if (!IsTransformValid(frames->end_effector)) return false;
  frames->valid = true;
  return true;
}

template <int DoF>
inline Transform ForwardKinematics(const ArmModel<DoF>& model, const JointVector<DoF>& position) {
  KinematicFrames<DoF> frames;
  if (!ComputeKinematicFrames(model, position, &frames)) {
    Transform invalid;
    invalid.fill(std::numeric_limits<float>::quiet_NaN());
    return invalid;
  }
  return frames.end_effector;
}

template <int DoF>
inline bool TryForwardKinematics(const ArmModel<DoF>& model, const JointVector<DoF>& position,
                                 Transform* output) {
  if (output == nullptr) {
    return false;
  }
  KinematicFrames<DoF> frames;
  if (!ComputeKinematicFrames(model, position, &frames)) {
    *output = {};
    return false;
  }
  *output = frames.end_effector;
  return true;
}

}  // namespace mr::robotics::arm::kinematics
