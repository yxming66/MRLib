#pragma once

// The only high-level runtime controller in the standalone arm library.
// It owns fixed-size planner state, but never knows about motors or buses.

#include "../core/command.hpp"
#include "../core/types.hpp"
#include "../dynamics/gravity_compensation.hpp"
#include "../kinematics/forward_kinematics.hpp"
#include "../kinematics/inverse_kinematics.hpp"
#include "../model/arm_model.hpp"
#include "../planning/cartesian_planner.hpp"
#include "../planning/joint_planner.hpp"
#include "../safety/joint_safety.hpp"
#include "component/math/matrix.hpp"
#include "component/math/scalar.hpp"
#include "component/trajectory/online_trapezoid.hpp"

namespace mr::robotics::arm::control {

template <int DoF>
struct ArmControllerConfig {
  kinematics::InverseKinematicsOptions inverse_kinematics;
  Scalar default_dt_s;
  Scalar max_linear_velocity_m_s;
  Scalar max_angular_velocity_rad_s;
  Scalar max_linear_acceleration_m_s2;
  Scalar max_angular_acceleration_rad_s2;
  bool enable_gravity_compensation;
  bool hold_on_failure;
  bool enable_position_only_fallback;

  ArmControllerConfig()
      : inverse_kinematics(),
        default_dt_s(0.001f),
        max_linear_velocity_m_s(0.2f),
        max_angular_velocity_rad_s(1.0f),
        max_linear_acceleration_m_s2(0.5f),
        max_angular_acceleration_rad_s2(2.0f),
        enable_gravity_compensation(false),
        hold_on_failure(true),
        enable_position_only_fallback(false) {}
};

template <int DoF>
class ArmController {
  static_assert(DoF > 0, "ArmController requires at least one joint");

 public:
  ArmController(const ArmModel<DoF>& model, const ArmControllerConfig<DoF>& config)
      : model_(model),
        config_(config),
        joint_planner_(),
        cartesian_planner_(),
        last_safe_position_(mr::component::math::matrix::zeros<DoF, 1>()),
        hold_position_(mr::component::math::matrix::zeros<DoF, 1>()),
        active_joint_target_(mr::component::math::matrix::zeros<DoF, 1>()),
        active_cartesian_target_(mr::component::math::matrix::identity<4>()),
        active_mode_(ControlMode::kDisabled),
        plan_elapsed_s_(0.0f),
        initialized_(false),
        configured_(IsModelValid(model) && IsConfigurationValid(config)) {}

  void Reset(const JointState<DoF>& feedback) {
    InvalidatePlans();
    initialized_ = false;
    last_safe_position_ = mr::component::math::matrix::zeros<DoF, 1>();
    hold_position_ = last_safe_position_;
    last_safe_velocity_ = {};
    if (!configured_ || !IsJointStateUsable(feedback) ||
        !safety::IsJointPositionSafe(model_, feedback.position)) {
      return;
    }
    last_safe_position_ = feedback.position;
    last_safe_velocity_ = feedback.velocity;
    hold_position_ = feedback.position;
    initialized_ = true;
  }

  ArmOutput<DoF> Update(const JointState<DoF>& feedback, const ArmCommand<DoF>& command,
                        Scalar dt_s) {
    ArmOutput<DoF> output;
    output.current_pose = CurrentPose(feedback);

    if (!configured_) {
      return MakeFailureOutput(Status::kInvalidModel, output.current_pose);
    }

    const Scalar effective_dt_s = ResolveDt(dt_s);
    if (!mr::component::math::is_finite_scalar(effective_dt_s) || effective_dt_s <= 0.0f ||
        !IsJointStateUsable(feedback) || !IsArmCommandFinite(command) ||
        !IsControlModeValid(command.mode)) {
      return MakeFailureOutput(Status::kInvalidInput, output.current_pose);
    }

    if (!safety::IsJointPositionSafe(model_, feedback.position)) {
      return MakeFailureOutput(Status::kLimitViolation, output.current_pose);
    }

    if (!initialized_) {
      Reset(feedback);
      if (!initialized_) {
        return MakeFailureOutput(Status::kInvalidInput, output.current_pose);
      }
    }

    switch (command.mode) {
      case ControlMode::kDisabled:
        return UpdateDisabled(feedback, command, output.current_pose);
      case ControlMode::kHold:
        return UpdateHold(feedback, command, output.current_pose);
      case ControlMode::kMoveJoint:
        return UpdateJoint(feedback, command, effective_dt_s, output.current_pose);
      case ControlMode::kMoveCartesian:
        if (command.streaming_cartesian) {
          return UpdateStreamingCartesian(command, effective_dt_s, output.current_pose);
        }
        return UpdateCartesian(feedback, command, effective_dt_s, output.current_pose);
      default:
        return MakeFailureOutput(Status::kInvalidInput, output.current_pose);
    }
  }

  const ArmModel<DoF>& Model() const { return model_; }
  bool IsConfigured() const { return configured_; }

 private:
  static bool IsPositionFallbackStatus(Status status) {
    return status == Status::kNoConvergence || status == Status::kSingularity;
  }

  kinematics::InverseKinematicsResult<DoF> SolveCartesianIk(
      const Transform& target, const JointVector<DoF>& initial_position, bool position_only,
      kinematics::InverseKinematicsOptions* used_options, bool* used_fallback,
      Status* primary_status) const {
    kinematics::InverseKinematicsRequest<DoF> request;
    request.target_pose = target;
    request.initial_position = initial_position;
    kinematics::InverseKinematicsOptions options = config_.inverse_kinematics;
    if (position_only) {
      options = kinematics::MakePositionOnlyInverseKinematicsOptions(options);
    }

    auto result = kinematics::SolveInverseKinematics(model_, request, options);
    if (primary_status) *primary_status = result.status;
    bool fallback = false;
    if (!position_only && config_.enable_position_only_fallback && !IsOk(result.status) &&
        IsPositionFallbackStatus(result.status)) {
      const auto fallback_options = kinematics::MakePositionOnlyInverseKinematicsOptions(options);
      const auto fallback_result =
          kinematics::SolveInverseKinematics(model_, request, fallback_options);
      if (IsOk(fallback_result.status)) {
        result = fallback_result;
        options = fallback_options;
        fallback = true;
      }
    }
    if (used_options != nullptr) {
      *used_options = options;
    }
    if (used_fallback != nullptr) {
      *used_fallback = fallback;
    }
    return result;
  }

  static bool IsConfigurationValid(const ArmControllerConfig<DoF>& config) {
    const kinematics::InverseKinematicsOptions& ik = config.inverse_kinematics;
    return mr::component::math::is_positive_scalar(config.default_dt_s) &&
           mr::component::math::is_positive_scalar(config.max_linear_velocity_m_s) &&
           mr::component::math::is_positive_scalar(config.max_angular_velocity_rad_s) &&
           mr::component::math::is_positive_scalar(config.max_linear_acceleration_m_s2) &&
           mr::component::math::is_positive_scalar(config.max_angular_acceleration_rad_s2) &&
           kinematics::IsInverseKinematicsOptionsValid(ik);
  }

  static bool IsControlModeValid(ControlMode mode) {
    switch (mode) {
      case ControlMode::kDisabled:
      case ControlMode::kHold:
      case ControlMode::kMoveJoint:
      case ControlMode::kMoveCartesian:
        return true;
      default:
        return false;
    }
  }

  static bool JointVectorsEqual(const JointVector<DoF>& lhs, const JointVector<DoF>& rhs) {
    for (int index = 0; index < DoF; ++index) {
      if (lhs[index][0] != rhs[index][0]) {
        return false;
      }
    }
    return true;
  }

  static bool TransformsEqual(const Transform& lhs, const Transform& rhs) {
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        if (lhs(row, col) != rhs(row, col)) {
          return false;
        }
      }
    }
    return true;
  }

  Transform CurrentPose(const JointState<DoF>& feedback) const {
    if (!IsJointStateFinite(feedback) || !IsModelValid(model_)) {
      return mr::component::math::matrix::identity<4>();
    }
    return kinematics::ForwardKinematics(model_, feedback.position);
  }

  Scalar ResolveDt(Scalar dt_s) const {
    if (dt_s == 0.0f) {
      return config_.default_dt_s;
    }
    return dt_s;
  }

  void InvalidatePlans() {
    joint_plan_active_ = false;
    cartesian_plan_active_ = false;
    active_mode_ = ControlMode::kDisabled;
    plan_elapsed_s_ = 0.0f;
  }

  ArmOutput<DoF> MakeFailureOutput(Status status, const Transform& current_pose) {
    InvalidatePlans();
    last_safe_velocity_ = {};
    ArmOutput<DoF> output;
    output.status = status;
    output.current_pose = current_pose;
    output.reached = false;
    if (!initialized_ || !configured_ || !config_.hold_on_failure ||
        !safety::IsJointPositionSafe(model_, last_safe_position_)) {
      output.joint_command.mode = JointCommandMode::kDisabled;
      output.joint_command.valid = false;
      return output;
    }

    output.joint_command.mode = JointCommandMode::kPositionVelocityEffort;
    output.joint_command.position = last_safe_position_;
    output.joint_command.velocity = mr::component::math::matrix::zeros<DoF, 1>();
    output.joint_command.acceleration = mr::component::math::matrix::zeros<DoF, 1>();
    output.joint_command.effort_feedforward = mr::component::math::matrix::zeros<DoF, 1>();
    output.joint_command.valid = true;
    return output;
  }

  ArmOutput<DoF> UpdateDisabled(const JointState<DoF>& feedback, const ArmCommand<DoF>& command,
                                const Transform& current_pose) {
    (void)command;
    InvalidatePlans();
    hold_position_ = feedback.position;
    last_safe_position_ = feedback.position;
    last_safe_velocity_ = feedback.velocity;
    ArmOutput<DoF> output;
    output.status = Status::kOk;
    output.current_pose = current_pose;
    output.joint_command.position = feedback.position;
    output.joint_command.mode = JointCommandMode::kDisabled;
    output.joint_command.valid = true;
    output.reached = false;
    return output;
  }

  ArmOutput<DoF> UpdateHold(const JointState<DoF>& feedback, const ArmCommand<DoF>& command,
                            const Transform& current_pose) {
    if (active_mode_ != ControlMode::kHold) {
      InvalidatePlans();
      active_mode_ = ControlMode::kHold;
      hold_position_ = feedback.position;
      last_safe_position_ = feedback.position;
    }
    JointCommand<DoF> joint_command = MakePositionCommand(
        hold_position_, mr::component::math::matrix::zeros<DoF, 1>(),
        mr::component::math::matrix::zeros<DoF, 1>(), command.effort_feedforward, true);
    const Status status = AddGravityCompensation(hold_position_, &joint_command.effort_feedforward);
    if (IsFailure(status)) {
      return MakeFailureOutput(status, current_pose);
    }
    ArmOutput<DoF> output = FinishCommand(status, joint_command, current_pose, true);
    if (output.status == Status::kOk || output.status == Status::kLimitViolation) {
      last_safe_position_ = output.joint_command.position;
      last_safe_velocity_ = output.joint_command.velocity;
    }
    return output;
  }

  ArmOutput<DoF> UpdateJoint(const JointState<DoF>& feedback, const ArmCommand<DoF>& command,
                             Scalar dt_s, const Transform& current_pose) {
    if (!safety::IsJointPositionSafe(model_, command.joint_target)) {
      InvalidatePlans();
      return MakeFailureOutput(Status::kLimitViolation, current_pose);
    }

    const bool new_plan = active_mode_ != ControlMode::kMoveJoint || !joint_plan_active_ ||
                          !JointVectorsEqual(active_joint_target_, command.joint_target);
    if (new_plan) {
      planning::JointPlanRequest<DoF> request;
      request.start = feedback.position;
      request.goal = command.joint_target;
      for (int index = 0; index < DoF; ++index) {
        request.max_velocity[index][0] = model_.Joint(index).limits.max_velocity;
        request.max_acceleration[index][0] = model_.Joint(index).limits.max_acceleration;
      }
      if (!joint_planner_.Configure(request)) {
        InvalidatePlans();
        return MakeFailureOutput(Status::kNumericalFailure, current_pose);
      }
      active_joint_target_ = command.joint_target;
      active_mode_ = ControlMode::kMoveJoint;
      joint_plan_active_ = true;
      cartesian_plan_active_ = false;
      plan_elapsed_s_ = 0.0f;
    }

    const planning::JointPlanSample<DoF> sample = joint_planner_.Sample(plan_elapsed_s_);
    if (!sample.valid) {
      InvalidatePlans();
      return MakeFailureOutput(Status::kNumericalFailure, current_pose);
    }

    JointCommand<DoF> joint_command = MakePositionCommand(
        sample.position, sample.velocity, sample.acceleration, command.effort_feedforward, true);
    const Status status =
        AddGravityCompensation(sample.position, &joint_command.effort_feedforward);
    if (IsFailure(status)) {
      InvalidatePlans();
      return MakeFailureOutput(status, current_pose);
    }
    ArmOutput<DoF> output = FinishCommand(status, joint_command, current_pose, sample.finished);
    if (output.status == Status::kOk || output.status == Status::kLimitViolation) {
      last_safe_position_ = output.joint_command.position;
      last_safe_velocity_ = output.joint_command.velocity;
      if (!sample.finished) {
        plan_elapsed_s_ += dt_s;
      }
    }
    return output;
  }

  ArmOutput<DoF> UpdateCartesian(const JointState<DoF>& feedback, const ArmCommand<DoF>& command,
                                 Scalar dt_s, const Transform& current_pose) {
    (void)feedback;
    if (!IsTransformValid(command.pose_target)) {
      InvalidatePlans();
      return MakeFailureOutput(Status::kInvalidInput, current_pose);
    }

    const bool new_plan = active_mode_ != ControlMode::kMoveCartesian || !cartesian_plan_active_ ||
                          !TransformsEqual(active_cartesian_target_, command.pose_target);
    if (new_plan) {
      planning::CartesianPlanRequest request;
      request.start = current_pose;
      request.goal = command.pose_target;
      request.max_linear_velocity_m_s = config_.max_linear_velocity_m_s;
      request.max_angular_velocity_rad_s = config_.max_angular_velocity_rad_s;
      request.max_linear_acceleration_m_s2 = config_.max_linear_acceleration_m_s2;
      request.max_angular_acceleration_rad_s2 = config_.max_angular_acceleration_rad_s2;
      if (!cartesian_planner_.Configure(request)) {
        InvalidatePlans();
        return MakeFailureOutput(Status::kNumericalFailure, current_pose);
      }
      active_cartesian_target_ = command.pose_target;
      active_mode_ = ControlMode::kMoveCartesian;
      joint_plan_active_ = false;
      cartesian_plan_active_ = true;
      plan_elapsed_s_ = 0.0f;
    }

    const planning::CartesianPlanSample sample = cartesian_planner_.Sample(plan_elapsed_s_);
    if (!sample.valid) {
      InvalidatePlans();
      return MakeFailureOutput(Status::kNumericalFailure, current_pose);
    }

    kinematics::InverseKinematicsOptions ik_options = config_.inverse_kinematics;
    bool used_fallback = false;
    Status primary_status = Status::kNotConfigured;
    const kinematics::InverseKinematicsResult<DoF> ik_result =
        SolveCartesianIk(sample.pose, last_safe_position_, command.position_only, &ik_options,
                         &used_fallback, &primary_status);
    if (!IsOk(ik_result.status)) {
      auto failure = MakeFailureOutput(ik_result.status, current_pose);
      failure.primary_ik_status = primary_status;
      failure.near_singularity = ik_result.near_singularity;
      return failure;
    }

    JointCommand<DoF> joint_command = MakePositionCommand(
        ik_result.position, mr::component::math::matrix::zeros<DoF, 1>(),
        mr::component::math::matrix::zeros<DoF, 1>(), command.effort_feedforward, false);
    const Status status =
        AddGravityCompensation(ik_result.position, &joint_command.effort_feedforward);
    if (IsFailure(status)) {
      InvalidatePlans();
      return MakeFailureOutput(status, current_pose);
    }
    ArmOutput<DoF> output = FinishCommand(
        status, joint_command, current_pose,
        sample.finished && ik_result.position_error <= ik_options.position_tolerance &&
            (!ik_options.use_orientation ||
             ik_result.orientation_error <= ik_options.orientation_tolerance));
    output.position_only_fallback = used_fallback;
    output.primary_ik_status = primary_status;
    output.near_singularity = ik_result.near_singularity;
    if (output.status == Status::kOk || output.status == Status::kLimitViolation) {
      last_safe_position_ = output.joint_command.position;
      last_safe_velocity_ = output.joint_command.velocity;
      if (!sample.finished) {
        plan_elapsed_s_ += dt_s;
      }
    }
    return output;
  }

  ArmOutput<DoF> UpdateStreamingCartesian(const ArmCommand<DoF>& command, Scalar dt_s,
                                          const Transform& current_pose) {
    // A remote target is intentionally different from a fixed pose goal: it
    // changes every cycle, so restarting Cartesian time interpolation would
    // keep sampling the beginning of a new plan. Solve the current bounded
    // target directly and limit the resulting joint step instead.
    InvalidatePlans();
    active_mode_ = ControlMode::kMoveCartesian;
    active_cartesian_target_ = command.pose_target;

    if (!IsTransformValid(command.pose_target)) {
      return MakeFailureOutput(Status::kInvalidInput, current_pose);
    }

    kinematics::InverseKinematicsOptions ik_options = config_.inverse_kinematics;
    bool used_fallback = false;
    Status primary_status = Status::kNotConfigured;
    const kinematics::InverseKinematicsResult<DoF> ik_result =
        SolveCartesianIk(command.pose_target, last_safe_position_, command.position_only,
                         &ik_options, &used_fallback, &primary_status);
    if (!IsOk(ik_result.status)) {
      auto failure = MakeFailureOutput(ik_result.status, current_pose);
      failure.primary_ik_status = primary_status;
      failure.near_singularity = ik_result.near_singularity;
      return failure;
    }

    JointVector<DoF> position = last_safe_position_;
    JointVector<DoF> velocity = last_safe_velocity_;
    JointVector<DoF> acceleration;
    bool step_limited = false;
    for (int index = 0; index < DoF; ++index) {
      const auto& limits = model_.Joint(index).limits;
      const Scalar max_velocity = std::fmin(limits.max_velocity, ik_options.max_joint_step / dt_s);
      const auto axis = mr::comp::traj::update_trapezoid_axis(
          last_safe_position_(index, 0), ik_result.position(index, 0), &velocity(index, 0),
          max_velocity, limits.max_acceleration, dt_s, 1.0e-6f);
      if (!axis.valid) return MakeFailureOutput(Status::kLimitViolation, current_pose);
      position(index, 0) = axis.position;
      acceleration(index, 0) = static_cast<Scalar>(
          (static_cast<double>(velocity(index, 0)) - last_safe_velocity_(index, 0)) / dt_s);
      step_limited = step_limited || !axis.reached;
    }

    JointCommand<DoF> joint_command =
        MakePositionCommand(position, velocity, acceleration, command.effort_feedforward, true);
    // Clamping a position here would break the velocity/acceleration contract.
    if (!safety::IsJointCommandWithinLimits(model_, joint_command))
      return MakeFailureOutput(Status::kLimitViolation, current_pose);
    const Status status = AddGravityCompensation(position, &joint_command.effort_feedforward);
    if (IsFailure(status)) {
      return MakeFailureOutput(status, current_pose);
    }

    const bool reached = !step_limited &&
                         ik_result.position_error <= ik_options.position_tolerance &&
                         (!ik_options.use_orientation ||
                          ik_result.orientation_error <= ik_options.orientation_tolerance);
    ArmOutput<DoF> output = FinishCommand(status, joint_command, current_pose, reached);
    output.position_only_fallback = used_fallback;
    output.primary_ik_status = primary_status;
    output.near_singularity = ik_result.near_singularity;
    if (output.status == Status::kOk || output.status == Status::kLimitViolation) {
      last_safe_position_ = output.joint_command.position;
      last_safe_velocity_ = output.joint_command.velocity;
    }
    return output;
  }

  JointCommand<DoF> MakePositionCommand(const JointVector<DoF>& position,
                                        const JointVector<DoF>& velocity,
                                        const JointVector<DoF>& acceleration,
                                        const JointVector<DoF>& effort_feedforward,
                                        bool include_velocity) const {
    JointCommand<DoF> command;
    command.position = position;
    command.velocity = velocity;
    command.acceleration = acceleration;
    command.effort_feedforward = effort_feedforward;
    command.mode =
        include_velocity ? JointCommandMode::kPositionVelocityEffort : JointCommandMode::kPosition;
    command.valid = true;
    return command;
  }

  Status AddGravityCompensation(const JointVector<DoF>& position,
                                JointVector<DoF>* effort_feedforward) const {
    if (effort_feedforward == nullptr || !IsJointVectorFinite(*effort_feedforward)) {
      return Status::kInvalidInput;
    }
    if (!config_.enable_gravity_compensation) {
      return Status::kOk;
    }
    const dynamics::GravityResult<DoF> gravity =
        dynamics::ComputeGravityCompensation(model_, position);
    if (!IsOk(gravity.status)) {
      return gravity.status;
    }
    *effort_feedforward += gravity.effort;
    return IsJointVectorFinite(*effort_feedforward) ? Status::kOk : Status::kNumericalFailure;
  }

  ArmOutput<DoF> FinishCommand(Status status, JointCommand<DoF> command,
                               const Transform& current_pose, bool reached) {
    ArmOutput<DoF> output;
    output.status = status;
    output.current_pose = current_pose;
    output.reached = reached;
    if (!safety::IsJointCommandWithinLimits(model_, command)) {
      const JointVector<DoF> clamped_position =
          safety::ClampJointPosition(model_, command.position);
      command.position = clamped_position;
      if (!safety::IsJointPositionSafe(model_, command.position) ||
          !safety::IsJointVelocitySafe(model_, command.velocity) ||
          !safety::IsJointAccelerationSafe(model_, command.acceleration) ||
          !safety::IsCommandFinite(command)) {
        return MakeFailureOutput(Status::kLimitViolation, current_pose);
      }
      status = Status::kLimitViolation;
      output.status = status;
    }
    output.joint_command = command;
    return output;
  }

  const ArmModel<DoF>& model_;
  ArmControllerConfig<DoF> config_;
  planning::JointPlanner<DoF> joint_planner_;
  planning::CartesianPlanner cartesian_planner_;
  JointVector<DoF> last_safe_position_;
  JointVector<DoF> last_safe_velocity_{};
  JointVector<DoF> hold_position_;
  JointVector<DoF> active_joint_target_;
  Transform active_cartesian_target_;
  ControlMode active_mode_;
  Scalar plan_elapsed_s_;
  bool initialized_;
  bool configured_;
  bool joint_plan_active_ = false;
  bool cartesian_plan_active_ = false;
};

}  // namespace mr::robotics::arm::control
