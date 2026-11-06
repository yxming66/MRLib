#pragma once

#include "status.hpp"
#include "types.hpp"

namespace mr::robotics::arm {

enum class ControlMode : uint8_t {
  kDisabled = 0,
  kHold,
  kMoveJoint,
  kMoveCartesian,
};

template <int DoF>
struct ArmCommand {
  ControlMode mode;
  JointVector<DoF> joint_target;
  Transform pose_target;  // T_world_tool: tool pose expressed in the model world frame.
  JointVector<DoF> effort_feedforward;
  bool position_only;
  // A streaming Cartesian target is updated every control cycle. It uses a
  // bounded IK target with per-joint velocity/acceleration-limited tracking.
  bool streaming_cartesian;

  ArmCommand()
      : mode(ControlMode::kDisabled),
        joint_target(mr::component::math::matrix::zeros<DoF, 1>()),
        pose_target(mr::component::math::matrix::identity<4>()),
        effort_feedforward(mr::component::math::matrix::zeros<DoF, 1>()),
        position_only(true),
        streaming_cartesian(false) {}
};

template <int DoF>
struct ArmOutput {
  Status status;
  JointCommand<DoF> joint_command;
  Transform current_pose;
  bool reached;
  bool position_only_fallback;
  Status primary_ik_status;  // Initial solve status, before an explicitly enabled fallback.
  bool near_singularity;

  ArmOutput()
      : status(Status::kNotConfigured),
        joint_command(),
        current_pose(mr::component::math::matrix::identity<4>()),
        reached(false),
        position_only_fallback(false),
        primary_ik_status(Status::kNotConfigured),
        near_singularity(false) {}
};

template <int DoF>
inline bool IsArmCommandFinite(const ArmCommand<DoF>& command) {
  return IsJointVectorFinite(command.joint_target) &&
         mr::component::math::matrix::is_finite(command.pose_target) &&
         IsJointVectorFinite(command.effort_feedforward);
}

}  // namespace mr::robotics::arm
