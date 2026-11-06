#include "module/arm/arm_output_adapter.hpp"

#include "component/math/scalar.hpp"
#include "device/device.h"
#include "device/motor/factory/motor_factory.hpp"

namespace mr::arm {
namespace {

using mr::motor::MotorFactory;
using mr::motor::MotorInstanceConfig;
using mr::motor::MotorKind;
using mr::motor::MotorModel;

float FiniteOr(float value, float fallback) {
  return mr::component::math::is_finite_scalar(value) ? value : fallback;
}

}  // namespace

ArmOutputAdapter::ArmOutputAdapter()
    : joint1_motor_(nullptr),
      joint2_motor_(nullptr),
      joint3_motor_(nullptr),
      kp_{0.0f, 0.0f, 0.0f},
      kd_{0.0f, 0.0f, 0.0f},
      gravity_scale_{1.0f, 1.0f, 1.0f},
      actuators_{},
      motor_states_{},
      joint1_actuator_(),
      joint2_actuator_(),
      joint3_actuator_(),
      initialized_(false) {}

mr::motor::MotorTemperatureProtectionConfig
ArmOutputAdapter::MakeTemperatureProtection(
    const Arm_TemperatureProtectionConfig_t& parameters) {
  mr::motor::MotorTemperatureProtectionConfig result;
  result.warning_c = parameters.warning_c;
  result.limit_c = parameters.limit_c;
  result.auto_relax_on_limit = parameters.auto_relax_on_limit;
  return result;
}

bool ArmOutputAdapter::Initialize(const Arm_Params_t& parameters) {
  if (initialized_) {
    return true;
  }

  const MotorInstanceConfig<MotorKind::LZ> joint1_config{
      static_cast<BSP_CAN_t>(parameters.joint1_motor_param.can),
      parameters.joint1_motor_param.motor_id,
      parameters.joint1_motor_param.host_id,
      static_cast<mr::motor::LzControlMode>(parameters.joint1_motor_param.mode),
      parameters.joint1_motor_param.reverse};
  const MotorInstanceConfig<MotorKind::DM> joint2_config{
      static_cast<BSP_CAN_t>(parameters.joint2_motor_param.can),
      parameters.joint2_motor_param.master_id,
      parameters.joint2_motor_param.can_id,
      parameters.joint2_motor_param.reverse};
  const MotorInstanceConfig<MotorKind::DM> joint3_config{
      static_cast<BSP_CAN_t>(parameters.joint3_motor_param.can),
      parameters.joint3_motor_param.master_id,
      parameters.joint3_motor_param.can_id,
      parameters.joint3_motor_param.reverse};

  joint1_motor_ = MotorFactory::Create<MotorKind::LZ, MotorModel::RSO3>(
      joint1_config, mr::motor::kDirectDriveInstall,
      MakeTemperatureProtection(parameters.joint_temperature_protection[0]));
  joint2_motor_ = MotorFactory::Create<MotorKind::DM, MotorModel::J4340>(
      joint2_config, mr::motor::kDirectDriveInstall,
      MakeTemperatureProtection(parameters.joint_temperature_protection[1]));
  joint3_motor_ = MotorFactory::Create<MotorKind::DM, MotorModel::J4310P>(
      joint3_config, mr::motor::kDirectDriveInstall,
      MakeTemperatureProtection(parameters.joint_temperature_protection[2]));

  if (joint1_motor_ == nullptr || joint2_motor_ == nullptr ||
      joint3_motor_ == nullptr) {
    return false;
  }

  joint1_actuator_.SetMotor(joint1_motor_);
  joint2_actuator_.SetMotor(joint2_motor_);
  joint3_actuator_.SetMotor(joint3_motor_);
  const std::array<mr::IActuator*, ARM_JOINT_COUNT> actuators = {
      &joint1_actuator_, &joint2_actuator_, &joint3_actuator_};
  return Bind(actuators, parameters);
}

bool ArmOutputAdapter::Bind(
    const std::array<mr::IActuator*, ARM_JOINT_COUNT>& actuators,
    const Arm_Params_t& parameters) {
  if (initialized_) {
    return true;
  }
  for (mr::IActuator* actuator : actuators) {
    if (actuator == nullptr) {
      return false;
    }
  }
  actuators_ = actuators;
  LoadControlParameters(parameters);
  initialized_ = true;
  return true;
}

void ArmOutputAdapter::LoadControlParameters(const Arm_Params_t& parameters) {

  for (size_t index = 0; index < ARM_JOINT_COUNT; ++index) {
    kp_[index] = FiniteOr(parameters.joint_kp[index], 0.0f);
    kd_[index] = FiniteOr(parameters.joint_kd[index], 0.0f);
    gravity_scale_[index] = FiniteOr(parameters.gravity_comp_scale[index], 1.0f);
  }
}

int8_t ArmOutputAdapter::FirstFailure(int8_t current, int8_t next) const {
  return (current != DEVICE_OK) ? current : next;
}

int8_t ArmOutputAdapter::Register() {
  if (!initialized_) {
    return DEVICE_ERR_NULL;
  }
  int8_t result = DEVICE_OK;
  for (mr::IActuator* actuator : actuators_) {
    result = FirstFailure(result, actuator->Register());
  }
  return result;
}

int8_t ArmOutputAdapter::Enable() {
  if (!initialized_) {
    return DEVICE_ERR_NULL;
  }
  int8_t result = DEVICE_OK;
  for (mr::IActuator* actuator : actuators_) {
    result = FirstFailure(result, actuator->Enable());
  }
  return result;
}

int8_t ArmOutputAdapter::Relax() {
  if (!initialized_) {
    return DEVICE_ERR_NULL;
  }
  int8_t result = DEVICE_OK;
  for (mr::IActuator* actuator : actuators_) {
    actuator->ClearPendingControl();
    result = FirstFailure(result, actuator->Relax());
  }
  return result;
}

int8_t ArmOutputAdapter::Relax(size_t index) {
  if (!initialized_) {
    return DEVICE_ERR_NULL;
  }
  if (index >= ARM_JOINT_COUNT || actuators_[index] == nullptr) {
    return DEVICE_ERR_NULL;
  }
  actuators_[index]->ClearPendingControl();
  return actuators_[index]->Relax();
}

mr::motor::MotorState ArmOutputAdapter::MotorState(size_t index) const {
  return (index < ARM_JOINT_COUNT) ? motor_states_[index]
                                   : mr::motor::MotorState{};
}

bool ArmOutputAdapter::IsOnline(size_t index) const {
  return MotorState(index).online;
}

int8_t ArmOutputAdapter::UpdateFeedback(ThreeAxisJointState* state) {
  if (!initialized_ || state == nullptr) {
    return DEVICE_ERR_NULL;
  }

  int8_t result = DEVICE_OK;
  for (size_t index = 0; index < ARM_JOINT_COUNT; ++index) {
    result = FirstFailure(result, actuators_[index]->Update());
    RefreshMotorState(index);
  }

  state->valid = true;
  for (size_t index = 0; index < ARM_JOINT_COUNT; ++index) {
    const mr::motor::MotorState motor_state = motor_states_[index];
    state->position[index][0] = motor_state.position_rad;
    state->velocity[index][0] = motor_state.velocity_rad_s;
    state->effort[index][0] = motor_state.torque_nm;
    state->valid = state->valid && motor_state.online &&
                   mr::component::math::is_finite_scalar(
                       motor_state.position_rad) &&
                   mr::component::math::is_finite_scalar(
                       motor_state.velocity_rad_s) &&
                   mr::component::math::is_finite_scalar(motor_state.torque_nm);
  }
  state->valid = state->valid && result == DEVICE_OK;
  return result;
}

int8_t ArmOutputAdapter::ApplyJoint(
    size_t index,
    const ThreeAxisJointCommand& command) {
  const float torque_feedforward =
      command.effort_feedforward[index][0] * gravity_scale_[index];
  if (command.mode == mr::robotics::arm::JointCommandMode::kEffort) {
    return (index < ARM_JOINT_COUNT && actuators_[index] != nullptr)
               ? actuators_[index]->SetTorque(torque_feedforward)
               : DEVICE_ERR_NULL;
  }
  return (index < ARM_JOINT_COUNT && actuators_[index] != nullptr)
             ? actuators_[index]->SetMIT(command.position[index][0],
                                         command.velocity[index][0], kp_[index],
                                         kd_[index], torque_feedforward)
             : DEVICE_ERR_NULL;
}

int8_t ArmOutputAdapter::Apply(const ThreeAxisJointCommand& command) {
  if (!initialized_ || !command.valid) {
    return DEVICE_ERR;
  }
  if (command.mode == mr::robotics::arm::JointCommandMode::kDisabled) {
    return Relax();
  }

  int8_t result = DEVICE_OK;
  for (size_t index = 0; index < ARM_JOINT_COUNT; ++index) {
    result = FirstFailure(result, ApplyJoint(index, command));
  }
  return result;
}

int8_t ArmOutputAdapter::Commit() {
  if (!initialized_) {
    return DEVICE_ERR_NULL;
  }
  int8_t result = DEVICE_OK;
  for (mr::IActuator* actuator : actuators_) {
    result = FirstFailure(result, actuator->CommitControl());
  }
  return result;
}

int8_t ArmOutputAdapter::SetZero(size_t index) {
  if (!initialized_) {
    return DEVICE_ERR_NULL;
  }
  if (index >= ARM_JOINT_COUNT || actuators_[index] == nullptr) {
    return DEVICE_ERR_NULL;
  }
  return actuators_[index]->SetZero();
}

void ArmOutputAdapter::RefreshMotorState(size_t index) {
  if (index >= ARM_JOINT_COUNT || actuators_[index] == nullptr) {
    return;
  }
  if (index == 0U && joint1_motor_ != nullptr) {
    motor_states_[index] = joint1_motor_->GetState();
    return;
  }
  if (index == 1U && joint2_motor_ != nullptr) {
    motor_states_[index] = joint2_motor_->GetState();
    return;
  }
  if (index == 2U && joint3_motor_ != nullptr) {
    motor_states_[index] = joint3_motor_->GetState();
    return;
  }
  const mr::ActuatorState state = actuators_[index]->GetState();
  motor_states_[index].position_rad = state.position;
  motor_states_[index].velocity_rad_s = state.velocity;
  motor_states_[index].torque_nm = state.torque;
  motor_states_[index].online = state.online;
  motor_states_[index].position_valid = state.online;
  motor_states_[index].feedback_valid = state.online;
}

}  // namespace mr::arm
