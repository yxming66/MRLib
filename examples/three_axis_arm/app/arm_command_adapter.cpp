#include "arm_command_adapter.hpp"

#include <math.h>

#include "component/math/matrix.hpp"
#include "component/math/scalar.hpp"
#include "robotics/core/rotation.hpp"
#include "robotics/core/transform.hpp"

namespace mr::arm {
namespace {

using mr::robotics::arm::ControlMode;
using mr::robotics::arm::IsJointStateFinite;
using mr::robotics::arm::IsJointStateUsable;
using mr::robotics::arm::Transform;
using mr::robotics::arm::Vec3;
using mr::robotics::core::MakeTransform;
using mr::robotics::core::Rotation;
using mr::robotics::core::RotationFromRpy;
using mr::robotics::core::RpyFromRotation;
using mr::robotics::core::RotationOf;
using mr::robotics::core::TranslationOf;

Vec3 MakeVec3(float x, float y, float z) {
  Vec3 value = mr::component::math::matrix::zeros<3, 1>();
  value[0][0] = x;
  value[1][0] = y;
  value[2][0] = z;
  return value;
}

bool IsPoseFinite(const ArmPose_t& pose) {
  return mr::component::math::is_finite_scalar(pose.x) &&
         mr::component::math::is_finite_scalar(pose.y) &&
         mr::component::math::is_finite_scalar(pose.z) &&
         mr::component::math::is_finite_scalar(pose.roll) &&
         mr::component::math::is_finite_scalar(pose.pitch) &&
         mr::component::math::is_finite_scalar(pose.yaw);
}

bool IsDeltaFinite(const ArmPoseDelta_t& delta) {
  return mr::component::math::is_finite_scalar(delta.x) &&
         mr::component::math::is_finite_scalar(delta.y) &&
         mr::component::math::is_finite_scalar(delta.z) &&
         mr::component::math::is_finite_scalar(delta.roll) &&
         mr::component::math::is_finite_scalar(delta.pitch) &&
         mr::component::math::is_finite_scalar(delta.yaw);
}

bool IsJointTargetFinite(const ArmJointAngles_t& target) {
  for (int index = 0; index < ARM_JOINT_COUNT; ++index) {
    if (!mr::component::math::is_finite_scalar(target.q[index])) {
      return false;
    }
  }
  return true;
}

Vec3 PoseRpy(const ArmPose_t& pose) {
  // robotics/core uses the conventional vector layout [roll, pitch, yaw].
  return MakeVec3(pose.roll, pose.pitch, pose.yaw);
}

Transform MakePoseTransform(const ArmPose_t& pose) {
  return MakeTransform(RotationFromRpy(PoseRpy(pose)),
                       MakeVec3(pose.x, pose.y, pose.z));
}

ArmPose_t MakePose(const Transform& transform) {
  const Vec3 translation = TranslationOf(transform);
  const Vec3 rpy = RpyFromRotation(RotationOf(transform));
  ArmPose_t pose{};
  pose.x = translation[0][0];
  pose.y = translation[1][0];
  pose.z = translation[2][0];
  pose.roll = rpy[0][0];
  pose.pitch = rpy[1][0];
  pose.yaw = rpy[2][0];
  return pose;
}

bool IsRangeValid(float lower, float upper) {
  return mr::component::math::is_finite_scalar(lower) &&
         mr::component::math::is_finite_scalar(upper) && lower < upper;
}

}  // namespace

ArmCommandAdapterResult::ArmCommandAdapterResult()
    : command(),
      target_pose(mr::component::math::matrix::identity<4>()),
      target_joints(mr::component::math::matrix::zeros<ARM_JOINT_COUNT, 1>()),
      accepted(false),
      target_updated(false),
      reset_to_feedback(false),
      workspace_rejected(false) {}

ArmTransform PoseToTransform(const ArmPose_t& pose) {
  return MakePoseTransform(pose);
}

ArmPose_t TransformToPose(const ArmTransform& transform) {
  return MakePose(transform);
}

ArmCommandAdapter::ArmCommandAdapter()
    : parameters_{},
      target_pose_(mr::component::math::matrix::identity<4>()),
      target_joints_(mr::component::math::matrix::zeros<ARM_JOINT_COUNT, 1>()),
      initialized_(false) {}

void ArmCommandAdapter::Configure(
    const ArmCartesianRemoteParam_t& parameters) {
  parameters_ = parameters;
  if (!mr::component::math::is_finite_scalar(parameters_.input_deadzone) ||
      parameters_.input_deadzone < 0.0f) {
    parameters_.input_deadzone = 0.0f;
  }
  parameters_.input_deadzone = mr::component::math::clamp_scalar(
      parameters_.input_deadzone, 0.0f, 0.99f);
}

void ArmCommandAdapter::Reset(const ThreeAxisJointState& feedback,
                              const ArmTransform& current_pose) {
  if (IsJointStateFinite(feedback) &&
      mr::component::math::matrix::is_finite(current_pose)) {
    target_joints_ = feedback.position;
    target_pose_ = current_pose;
    initialized_ = true;
    return;
  }
  target_joints_ = mr::component::math::matrix::zeros<ARM_JOINT_COUNT, 1>();
  target_pose_ = mr::component::math::matrix::identity<4>();
  initialized_ = false;
}

float ArmCommandAdapter::NormalizeRemoteAxis(float value,
                                             float max_value) const {
  if (!mr::component::math::is_finite_scalar(value) ||
      !mr::component::math::is_positive_scalar(max_value)) {
    return 0.0f;
  }
  const float normalized = mr::component::math::clamp_scalar(
      value / max_value, -1.0f, 1.0f);
  const float absolute_value =
      mr::component::math::abs_scalar(normalized);
  if (absolute_value <= parameters_.input_deadzone) {
    return 0.0f;
  }
  const float scaled =
      (absolute_value - parameters_.input_deadzone) /
      (1.0f - parameters_.input_deadzone);
  return (normalized >= 0.0f) ? scaled : -scaled;
}

ArmPoseDelta_t ArmCommandAdapter::MakeRemoteDelta(
    const ArmPoseDelta_t& velocity,
    float dt_s) const {
  ArmPoseDelta_t delta{};
  const float linear_limit =
      mr::component::math::is_positive_scalar(
          parameters_.max_linear_velocity)
          ? parameters_.max_linear_velocity
          : 0.20f;
  const float angular_limit =
      mr::component::math::is_positive_scalar(
          parameters_.max_angular_velocity)
          ? parameters_.max_angular_velocity
          : 1.0f;
  const float y_limit =
      mr::component::math::is_positive_scalar(parameters_.max_y_velocity)
          ? parameters_.max_y_velocity
          : linear_limit;
  const float z_limit =
      mr::component::math::is_positive_scalar(parameters_.max_z_velocity)
          ? parameters_.max_z_velocity
          : linear_limit;
  const float pitch_limit =
      mr::component::math::is_positive_scalar(
          parameters_.max_pitch_velocity)
          ? parameters_.max_pitch_velocity
          : angular_limit;

  delta.x = NormalizeRemoteAxis(velocity.x, linear_limit) * linear_limit * dt_s;
  delta.y = NormalizeRemoteAxis(velocity.y, y_limit) * y_limit * dt_s;
  delta.z = NormalizeRemoteAxis(velocity.z, z_limit) * z_limit * dt_s;
  delta.roll = NormalizeRemoteAxis(velocity.roll, angular_limit) *
               angular_limit * dt_s;
  delta.pitch = NormalizeRemoteAxis(velocity.pitch, pitch_limit) *
                pitch_limit * dt_s;
  delta.yaw = NormalizeRemoteAxis(velocity.yaw, angular_limit) *
              angular_limit * dt_s;
  return delta;
}

ArmTransform ArmCommandAdapter::ApplyDelta(const ArmTransform& base,
                                           const ArmPoseDelta_t& delta,
                                           ArmControlFrame_t frame) const {
  const ArmPose_t base_pose = MakePose(base);
  const Vec3 delta_translation = MakeVec3(delta.x, delta.y, delta.z);
  const Rotation delta_rotation = RotationFromRpy(PoseRpy(
      ArmPose_t{0.0f, 0.0f, 0.0f, delta.roll, delta.pitch, delta.yaw}));
  const Rotation base_rotation = RotationOf(base);

  if (frame == ARM_CTRL_FRAME_TOOL) {
    return base * MakeTransform(delta_rotation, delta_translation);
  }

  if (frame == ARM_CTRL_FRAME_HEADING) {
    const float cosine = cosf(base_pose.yaw);
    const float sine = sinf(base_pose.yaw);
    const Vec3 world_translation = MakeVec3(
        cosine * delta.x - sine * delta.y,
        sine * delta.x + cosine * delta.y,
        delta.z);
    ArmPose_t next_pose = base_pose;
    next_pose.x += world_translation[0][0];
    next_pose.y += world_translation[1][0];
    next_pose.z += world_translation[2][0];
    next_pose.roll += delta.roll;
    next_pose.pitch += delta.pitch;
    next_pose.yaw += delta.yaw;
    return MakePoseTransform(next_pose);
  }

  return MakeTransform(delta_rotation * base_rotation,
                       TranslationOf(base) + delta_translation);
}

bool ArmCommandAdapter::IsWorkspaceSafe(const ArmTransform& pose) const {
  if (!mr::component::math::matrix::is_finite(pose)) {
    return false;
  }
  const ArmPose_t value = MakePose(pose);
  if (!IsPoseFinite(value)) {
    return false;
  }
  const ArmWorkspaceLimit_t& workspace = parameters_.workspace;
  if (IsRangeValid(workspace.y_min, workspace.y_max) &&
      (value.y < workspace.y_min || value.y > workspace.y_max)) {
    return false;
  }
  if (IsRangeValid(workspace.z_min, workspace.z_max) &&
      (value.z < workspace.z_min || value.z > workspace.z_max)) {
    return false;
  }
  if (IsRangeValid(workspace.pitch_min, workspace.pitch_max) &&
      (value.pitch < workspace.pitch_min || value.pitch > workspace.pitch_max)) {
    return false;
  }
  return true;
}

void ArmCommandAdapter::SetHoldCommand(
    ArmCommandAdapterResult* result) const {
  if (result == nullptr) {
    return;
  }
  result->command.mode = ControlMode::kHold;
  result->command.joint_target = target_joints_;
  result->command.pose_target = target_pose_;
  result->command.position_only = true;
}

void ArmCommandAdapter::SetMoveCartesianCommand(
    ArmCommandAdapterResult* result) const {
  if (result == nullptr) {
    return;
  }
  result->command.mode = ControlMode::kMoveCartesian;
  result->command.pose_target = target_pose_;
  result->command.joint_target = target_joints_;
  // A three-axis arm cannot independently control all six pose coordinates.
  // Position-only IK keeps the command well-conditioned; pitch remains a
  // workspace constraint until a task-specific orientation model is added.
  result->command.position_only = true;
}

void ArmCommandAdapter::SetMoveJointCommand(
    ArmCommandAdapterResult* result) const {
  if (result == nullptr) {
    return;
  }
  result->command.mode = ControlMode::kMoveJoint;
  result->command.joint_target = target_joints_;
  result->command.pose_target = target_pose_;
  result->command.position_only = true;
}

ArmCommandAdapterResult ArmCommandAdapter::Build(
    const Arm_CMD_t& input,
    const ThreeAxisJointState& feedback,
    const ArmTransform& current_pose,
    float dt_s) {
  ArmCommandAdapterResult result;
  result.target_pose = target_pose_;
  result.target_joints = target_joints_;

  if (!input.enable) {
    result.command.mode = ControlMode::kDisabled;
    initialized_ = false;
    return result;
  }

  if (!IsJointStateUsable(feedback) ||
      !mr::component::math::matrix::is_finite(current_pose) ||
      !mr::component::math::is_finite_scalar(dt_s) || dt_s <= 0.0f) {
    if (initialized_) {
      SetHoldCommand(&result);
    }
    result.target_pose = target_pose_;
    result.target_joints = target_joints_;
    return result;
  }

  if (!initialized_) {
    Reset(feedback, current_pose);
  }

  if (input.set_target_as_current) {
    Reset(feedback, current_pose);
    result.reset_to_feedback = initialized_;
    result.accepted = initialized_;
    SetHoldCommand(&result);
    result.target_pose = target_pose_;
    result.target_joints = target_joints_;
    return result;
  }

  bool candidate_valid = true;
  Transform candidate_pose = target_pose_;
  switch (input.ctrl_type) {
    case ARM_CTRL_REMOTE_CARTESIAN: {
      const ArmPoseDelta_t delta = MakeRemoteDelta(input.joy_vel, dt_s);
      candidate_pose = ApplyDelta(target_pose_, delta, input.frame);
      break;
    }
    case ARM_CTRL_POSE_ABSOLUTE:
      candidate_valid = IsPoseFinite(input.target_pose);
      if (candidate_valid) {
        candidate_pose = PoseToTransform(input.target_pose);
      }
      break;
    case ARM_CTRL_POSE_DELTA:
      candidate_valid = IsDeltaFinite(input.target_delta);
      if (candidate_valid) {
        candidate_pose =
            ApplyDelta(target_pose_, input.target_delta, input.frame);
      }
      break;
    case ARM_CTRL_CUSTOM_JOINT:
      candidate_valid = IsJointTargetFinite(input.target_joints);
      if (candidate_valid) {
        for (int index = 0; index < ARM_JOINT_COUNT; ++index) {
          target_joints_[index][0] = input.target_joints.q[index];
        }
      }
      result.accepted = candidate_valid;
      if (candidate_valid) {
        SetMoveJointCommand(&result);
      } else {
        SetHoldCommand(&result);
      }
      result.target_pose = target_pose_;
      result.target_joints = target_joints_;
      return result;
    default:
      candidate_valid = false;
      break;
  }

  if (!candidate_valid || !IsWorkspaceSafe(candidate_pose)) {
    result.workspace_rejected = candidate_valid;
    result.accepted = false;
    SetHoldCommand(&result);
    result.target_pose = target_pose_;
    result.target_joints = target_joints_;
    return result;
  }

    result.target_updated = true;
    result.accepted = true;
    target_pose_ = candidate_pose;
    SetMoveCartesianCommand(&result);
    result.command.streaming_cartesian =
        input.ctrl_type == ARM_CTRL_REMOTE_CARTESIAN;
    result.target_pose = target_pose_;
  result.target_joints = target_joints_;
  return result;
}

}  // namespace mr::arm
