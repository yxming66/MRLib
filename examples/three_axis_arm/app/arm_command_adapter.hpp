#pragma once

// Converts the C command boundary into the hardware-independent arm command.
// Remote input integration and workspace policy belong here, not in the
// generic robotics library.

#include "arm_control_types.h"
#include "robotics/arm/core/command.hpp"

namespace mr::arm {

using ThreeAxisArmCommand =
    mr::robotics::arm::ArmCommand<ARM_JOINT_COUNT>;
using ThreeAxisJointState =
    mr::robotics::arm::JointState<ARM_JOINT_COUNT>;
using ThreeAxisJointVector =
    mr::robotics::arm::JointVector<ARM_JOINT_COUNT>;
using ArmTransform = mr::robotics::arm::Transform;

struct ArmCommandAdapterResult {
  ThreeAxisArmCommand command;
  ArmTransform target_pose;
  ThreeAxisJointVector target_joints;
  bool accepted;
  bool target_updated;
  bool reset_to_feedback;
  bool workspace_rejected;

  ArmCommandAdapterResult();
};

ArmTransform PoseToTransform(const ArmPose_t& pose);
ArmPose_t TransformToPose(const ArmTransform& transform);

class ArmCommandAdapter final {
 public:
  ArmCommandAdapter();

  ArmCommandAdapter(const ArmCommandAdapter&) = delete;
  ArmCommandAdapter& operator=(const ArmCommandAdapter&) = delete;

  void Configure(const ArmCartesianRemoteParam_t& parameters);
  void Reset(const ThreeAxisJointState& feedback,
             const ArmTransform& current_pose);

  ArmCommandAdapterResult Build(const Arm_CMD_t& input,
                                const ThreeAxisJointState& feedback,
                                const ArmTransform& current_pose,
                                float dt_s);

  const ArmTransform& TargetPose() const { return target_pose_; }
  const ThreeAxisJointVector& TargetJoints() const { return target_joints_; }
  bool IsInitialized() const { return initialized_; }

 private:
  bool IsWorkspaceSafe(const ArmTransform& pose) const;
  ArmTransform ApplyDelta(const ArmTransform& base,
                          const ArmPoseDelta_t& delta,
                          ArmControlFrame_t frame) const;
  ArmPoseDelta_t MakeRemoteDelta(const ArmPoseDelta_t& velocity,
                                 float dt_s) const;
  float NormalizeRemoteAxis(float value, float max_value) const;
  void SetHoldCommand(ArmCommandAdapterResult* result) const;
  void SetMoveCartesianCommand(ArmCommandAdapterResult* result) const;
  void SetMoveJointCommand(ArmCommandAdapterResult* result) const;

  ArmCartesianRemoteParam_t parameters_{};
  ArmTransform target_pose_;
  ThreeAxisJointVector target_joints_;
  bool initialized_;
};

}  // namespace mr::arm
