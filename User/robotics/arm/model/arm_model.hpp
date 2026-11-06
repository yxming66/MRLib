#pragma once

#include <stdint.h>

#include "../core/types.hpp"
#include "component/math/matrix.hpp"
#include "component/math/scalar.hpp"
#include "joint_model.hpp"

namespace mr::robotics::arm {

template <int DoF>
class ArmModel {
  static_assert(DoF > 0, "ArmModel requires at least one joint");

 public:
  ArmModel()
      : joints_(),
        base_to_world_(mr::component::math::matrix::identity<4>()),
        tool_to_last_joint_(mr::component::math::matrix::identity<4>()),
        gravity_world_(mr::component::math::matrix::zeros<3, 1>()) {
    gravity_world_[2][0] = -9.81f;
  }

  ArmModel(const JointModel (&joints)[DoF], const Transform& base_to_world,
           const Transform& tool_to_last_joint, const Vec3& gravity_world)
      : joints_(),
        base_to_world_(base_to_world),
        tool_to_last_joint_(tool_to_last_joint),
        gravity_world_(gravity_world) {
    for (int index = 0; index < DoF; ++index) {
      joints_[index] = joints[index];
    }
  }

  bool Validate() const {
    if (!IsTransformValid(base_to_world_) || !IsTransformValid(tool_to_last_joint_) ||
        !mr::component::math::matrix::is_finite(gravity_world_)) {
      return false;
    }
    for (int index = 0; index < DoF; ++index) {
      if (!ValidateJointModel(joints_[index])) {
        return false;
      }
    }
    return true;
  }

  bool IsValidIndex(int index) const { return index >= 0 && index < DoF; }

  const JointModel& Joint(int index) const { return joints_[index]; }

  // T_world_base; transforms base coordinates into world coordinates.
  const Transform& BaseTransform() const { return base_to_world_; }

  // T_last_tool; transforms tool coordinates into the last link frame.
  const Transform& ToolTransform() const { return tool_to_last_joint_; }

  const Vec3& Gravity() const { return gravity_world_; }

 private:
  JointModel joints_[DoF];
  Transform base_to_world_;
  Transform tool_to_last_joint_;
  Vec3 gravity_world_;
};

template <int DoF>
inline bool IsModelValid(const ArmModel<DoF>& model) {
  return model.Validate();
}

}  // namespace mr::robotics::arm
