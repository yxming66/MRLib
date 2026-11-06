#include "module/arm/arm.hpp"

#include <cstring>
#include <new>

#include "component/math/matrix.hpp"
#include "component/math/scalar.hpp"
#include "device/device.h"
#include "robotics/arm/kinematics/forward_kinematics.hpp"

namespace mr::arm {

Runtime::Runtime()
    : model_(),
      controller_config_(),
      controller_storage_{},
      controller_(nullptr),
      command_adapter_(),
      output_adapter_(),
      feedback_(),
      command_{},
      last_output_(),
      last_frame_(ARM_CTRL_FRAME_WORLD),
      last_ctrl_type_(ARM_CTRL_REMOTE_CARTESIAN),
      last_enable_(false),
      initialized_(false),
      control_freq_hz_(500.0f),
      config_(),
      parameters_(nullptr),
      debug_() {}

Runtime::~Runtime() {
  if (controller_ != nullptr) {
    controller_->~Controller();
    controller_ = nullptr;
  }
}

float Runtime::ClampDt(float dt_s, float control_freq_hz) {
  const float frequency = mr::component::math::is_positive_scalar(
                              control_freq_hz)
                              ? control_freq_hz
                              : 500.0f;
  const float nominal_dt_s = 1.0f / frequency;
  const float minimum_dt_s = (nominal_dt_s * 0.5f > 0.0005f)
                                 ? nominal_dt_s * 0.5f
                                 : 0.0005f;
  return mr::component::math::sanitize_dt(
      dt_s, nominal_dt_s, minimum_dt_s, nominal_dt_s * 3.0f);
}

void Runtime::ClearTransientCommand(Arm_CMD_t* command) {
  if (command == nullptr) {
    return;
  }
  command->set_target_as_current = false;
  command->gripper_toggle = false;
  std::memset(&command->target_delta, 0, sizeof(command->target_delta));
  std::memset(&command->joy_vel, 0, sizeof(command->joy_vel));
}

ArmPose_t Runtime::ZeroPose() {
  return ArmPose_t{};
}

ArmJointAngles_t Runtime::JointAnglesFromVector(
    const ThreeAxisJointVector& values) {
  ArmJointAngles_t result{};
  for (int index = 0; index < ARM_JOINT_COUNT; ++index) {
    result.q[index] = values[index][0];
  }
  return result;
}

bool Runtime::InitializeController(float control_freq_hz) {
  if (controller_ != nullptr) {
    return controller_->IsConfigured();
  }
  if (parameters_ == nullptr) {
    return false;
  }

  model_ = BuildThreeAxisArmModel(*parameters_);
  controller_config_ = BuildThreeAxisControllerConfig(
      *parameters_, ClampDt(0.0f, control_freq_hz));
  controller_ = ::new (static_cast<void*>(controller_storage_)) Controller(
      model_, controller_config_);
  return controller_->IsConfigured();
}

bool Runtime::Init(const Config& config, float control_freq_hz) {
  if (initialized_) {
    return true;
  }
  control_freq_hz_ = mr::component::math::is_positive_scalar(control_freq_hz)
                         ? control_freq_hz
                         : 500.0f;

  if (config.parameters == nullptr) {
    return false;
  }
  config_ = config;
  parameters_ = config_.parameters;

  if (!output_adapter_.Initialize(*parameters_) ||
      output_adapter_.Register() != DEVICE_OK ||
      !InitializeController(control_freq_hz_)) {
    return false;
  }

  command_adapter_.Configure(parameters_->remote_cartesian);
  (void)output_adapter_.Enable();
  initialized_ = true;
  command_.ctrl_type = ARM_CTRL_REMOTE_CARTESIAN;
  command_.frame = ARM_CTRL_FRAME_WORLD;
  Update();
  if (feedback_.valid) {
    controller_->Reset(feedback_);
    const ArmTransform current_pose =
        mr::robotics::arm::kinematics::ForwardKinematics(
            model_, feedback_.position);
    command_adapter_.Reset(feedback_, current_pose);
  }
  RefreshDebug(nullptr, nullptr);
  debug_.initialized = true;
  return true;
}

void Runtime::Update() {
  if (!initialized_) {
    return;
  }
  (void)output_adapter_.UpdateFeedback(&feedback_);
  RefreshDebug(nullptr, nullptr);
}

void Runtime::SetCommand(const Arm_CMD_t& command) {
  command_ = command;
}

void Runtime::ResetControllerIfNeeded() {
  if (controller_ == nullptr || !feedback_.valid) {
    return;
  }
  const ArmTransform current_pose =
      mr::robotics::arm::kinematics::ForwardKinematics(
          model_, feedback_.position);
  controller_->Reset(feedback_);
  command_adapter_.Reset(feedback_, current_pose);
}

void Runtime::Control(float dt_s) {
  if (!initialized_ || controller_ == nullptr) {
    return;
  }

  const float effective_dt_s = ClampDt(dt_s, control_freq_hz_);
  const bool mode_changed = command_.ctrl_type != last_ctrl_type_ ||
                            command_.frame != last_frame_;
  if (command_.enable && (!last_enable_ || mode_changed)) {
    ResetControllerIfNeeded();
    (void)output_adapter_.Enable();
  }

  ArmTransform current_pose = mr::component::math::matrix::identity<4>();
  if (feedback_.valid) {
    current_pose = mr::robotics::arm::kinematics::ForwardKinematics(
        model_, feedback_.position);
  }
  const ArmCommandAdapterResult adapter_result = command_adapter_.Build(
      command_, feedback_, current_pose, effective_dt_s);

  ArmOutput output;
  if (feedback_.valid) {
    output = controller_->Update(feedback_, adapter_result.command,
                                 effective_dt_s);
    last_output_ = output;
    if (output_adapter_.Apply(output.joint_command) != DEVICE_OK) {
      (void)output_adapter_.Relax();
      last_output_.status = mr::robotics::arm::Status::kNumericalFailure;
    }
  } else {
    // Never send a hold command using stale feedback. A lost actuator update
    // is handled as a hardware safety event at this boundary.
    (void)output_adapter_.Relax();
    last_output_ = output;
    last_output_.status = mr::robotics::arm::Status::kInvalidInput;
  }

  RefreshDebug(&adapter_result, &last_output_);
  last_enable_ = command_.enable;
  last_frame_ = command_.frame;
  last_ctrl_type_ = command_.ctrl_type;
  ClearTransientCommand(&command_);
}

void Runtime::Commit() {
  if (!initialized_) {
    return;
  }
  debug_.last_commit_ok = output_adapter_.Commit() == DEVICE_OK;
  RefreshDebug(nullptr, nullptr);
}

int8_t Runtime::RelaxJoint(size_t index) {
  if (!initialized_) {
    return DEVICE_ERR_NULL;
  }
  const int8_t result = output_adapter_.Relax(index);
  debug_.last_commit_ok = result == DEVICE_OK;
  return result;
}

int8_t Runtime::RelaxAll() {
  if (!initialized_) {
    return DEVICE_ERR_NULL;
  }
  const int8_t result = output_adapter_.Relax();
  debug_.last_commit_ok = result == DEVICE_OK;
  return result;
}

int8_t Runtime::SetMotorZero(size_t index) {
  if (!initialized_) {
    return DEVICE_ERR_NULL;
  }
  const int8_t result = output_adapter_.SetZero(index);
  if (result == DEVICE_OK) {
    Update();
    ResetControllerIfNeeded();
  }
  return result;
}

void Runtime::RefreshDebug(const ArmCommandAdapterResult* adapter_result,
                           const ArmOutput* output) {
  debug_.initialized = initialized_;
  debug_.enabled = command_.enable;
  debug_.ctrl_type = command_.ctrl_type;
  debug_.frame = command_.frame;
  debug_.feedback_valid = feedback_.valid;
  debug_.status = (output != nullptr) ? output->status : last_output_.status;
  debug_.reached = (output != nullptr) ? output->reached : last_output_.reached;

  if (adapter_result != nullptr) {
    debug_.command_accepted = adapter_result->accepted;
    debug_.workspace_rejected = adapter_result->workspace_rejected;
    debug_.target_pose = TransformToPose(adapter_result->target_pose);
    debug_.target_joints =
        JointAnglesFromVector(adapter_result->target_joints);
  } else {
    debug_.target_pose = TransformToPose(command_adapter_.TargetPose());
    debug_.target_joints =
        JointAnglesFromVector(command_adapter_.TargetJoints());
  }

  if (feedback_.valid && controller_ != nullptr) {
    const ArmTransform current_pose =
        mr::robotics::arm::kinematics::ForwardKinematics(
            model_, feedback_.position);
    debug_.current_pose = TransformToPose(current_pose);
  } else {
    debug_.current_pose = ZeroPose();
  }

  for (int index = 0; index < ARM_JOINT_COUNT; ++index) {
    debug_.current_joints.q[index] = feedback_.position[index][0];
    debug_.joint_velocity[index] = feedback_.velocity[index][0];
    debug_.joint_online[index] = output_adapter_.IsOnline(index);
    const mr::motor::MotorState state = output_adapter_.MotorState(index);
    debug_.joint_temperature_c[index] = state.temperature_c;
    debug_.joint_temperature_warning[index] = state.temperature_warning;
    debug_.joint_temperature_over_limit[index] = state.temperature_over_limit;
    debug_.joint_temperature_limit_latched[index] =
        state.temperature_limit_latched;
    if (output != nullptr) {
      debug_.control_position[index] = output->joint_command.position[index][0];
      debug_.control_velocity[index] = output->joint_command.velocity[index][0];
      debug_.control_acceleration[index] =
          output->joint_command.acceleration[index][0];
      debug_.control_torque_ff[index] =
          output->joint_command.effort_feedforward[index][0];
    }
  }
}

}  // namespace mr::arm
