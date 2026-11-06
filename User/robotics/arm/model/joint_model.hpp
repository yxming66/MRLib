#pragma once

#include <stdint.h>

#include "../core/types.hpp"
#include "component/math/scalar.hpp"
#include "robotics/core/rotation.hpp"
#include "robotics/core/transform.hpp"

namespace mr::robotics::arm {

enum class JointType : uint8_t {
  kRevolute = 0,
  kPrismatic,
};

struct JointLimits {
  bool position_enabled;
  Scalar position_min;
  Scalar position_max;
  Scalar max_velocity;
  Scalar max_acceleration;

  JointLimits()
      : position_enabled(false),
        position_min(0.0f),
        position_max(0.0f),
        max_velocity(0.0f),
        max_acceleration(0.0f) {}
};

struct InertialParameters {
  Scalar mass_kg;
  Vec3 center_of_mass;
  Rotation inertia_at_center_of_mass;

  InertialParameters()
      : mass_kg(0.0f),
        center_of_mass(mr::component::math::matrix::zeros<3, 1>()),
        inertia_at_center_of_mass(mr::component::math::matrix::zeros<3, 3>()) {}
};

struct JointModel {
  JointType type;
  Transform parent_to_joint;
  Vec3 axis_in_joint_frame;
  Scalar position_offset;
  JointLimits limits;
  InertialParameters inertial;

  JointModel()
      : type(JointType::kRevolute),
        parent_to_joint(mr::component::math::matrix::identity<4>()),
        axis_in_joint_frame(mr::component::math::matrix::zeros<3, 1>()),
        position_offset(0.0f),
        limits(),
        inertial() {
    axis_in_joint_frame[2][0] = 1.0f;
  }
};

inline bool IsJointTypeValid(JointType type) {
  switch (type) {
    case JointType::kRevolute:
    case JointType::kPrismatic:
      return true;
    default:
      return false;
  }
}

inline bool IsJointLimitsValid(const JointLimits& limits) {
  if (!mr::component::math::is_finite_scalar(limits.max_velocity) ||
      !mr::component::math::is_finite_scalar(limits.max_acceleration) ||
      limits.max_velocity <= 0.0f || limits.max_acceleration <= 0.0f) {
    return false;
  }
  if (!limits.position_enabled) {
    return true;
  }
  return mr::component::math::is_finite_scalar(limits.position_min) &&
         mr::component::math::is_finite_scalar(limits.position_max) &&
         limits.position_min < limits.position_max;
}

inline bool ValidateJointModel(const JointModel& joint) {
  const Scalar axis_norm = joint.axis_in_joint_frame.norm();
  if (!IsJointTypeValid(joint.type) || !IsTransformValid(joint.parent_to_joint) ||
      !mr::component::math::is_finite_scalar(joint.position_offset) ||
      !mr::component::math::is_finite_scalar(axis_norm) ||
      axis_norm <= mr::component::math::kDefaultEpsilon || !IsJointLimitsValid(joint.limits)) {
    return false;
  }

  if (!mr::component::math::is_finite_scalar(joint.inertial.mass_kg) ||
      joint.inertial.mass_kg < 0.0f ||
      !mr::component::math::matrix::is_finite(joint.inertial.center_of_mass) ||
      !mr::component::math::matrix::is_finite(joint.inertial.inertia_at_center_of_mass)) {
    return false;
  }

  if (!mr::component::math::matrix::is_covariance(joint.inertial.inertia_at_center_of_mass))
    return false;

  return true;
}

}  // namespace mr::robotics::arm
