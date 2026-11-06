#include "device/wheel/wheel.hpp"

#include <new>

#include "device/motor/factory/motor_factory.hpp"
#include "device/motor/protocol/rm_protocol.hpp"

namespace mr::wheel {

namespace {

constexpr float kMinWheelRadiusM = 1e-4f;

using mr::motor::MotorFactory;
using mr::motor::MotorInstanceConfig;
using mr::motor::MotorKind;
using mr::motor::MotorModel;

}  // namespace

float RmM3508Wheel::ResolveRadius(float radius_m) {
  return (radius_m > 0.0f) ? radius_m : kMinWheelRadiusM;
}

RmM3508Wheel::RmM3508Wheel(const RmM3508WheelConfig& config)
    : radius_m_(ResolveRadius(config.radius_m)),
      motor_(nullptr),
      controller_(nullptr),
      state_{},
      controller_storage_{} {
  const MotorInstanceConfig<MotorKind::RM> motor_config{
      config.motor_param.can, config.motor_param.id, config.motor_param.reverse};
  motor_ = MotorFactory::Create<MotorKind::RM, MotorModel::M3508>(
      motor_config, config.install, config.temperature_protection);
  if (motor_ != nullptr) {
    controller_ =
        new (controller_storage_) Controller(*motor_, config.controller);
  }
}

int8_t RmM3508Wheel::Register() {
  if (controller_ == nullptr) {
    return DEVICE_ERR_NULL;
  }
  return controller_->Register();
}

int8_t RmM3508Wheel::Enable() {
  if (controller_ == nullptr) {
    return DEVICE_ERR_NULL;
  }
  return controller_->Enable();
}

int8_t RmM3508Wheel::Relax() {
  if (controller_ == nullptr) {
    return DEVICE_ERR_NULL;
  }
  const int8_t ret = controller_->Relax();
  return ret;
}

int8_t RmM3508Wheel::Update() {
  if (controller_ == nullptr) {
    return DEVICE_ERR_NULL;
  }
  const int8_t ret = controller_->Update();
  if (ret == DEVICE_OK) {
    RefreshState();
  }
  return ret;
}

int8_t RmM3508Wheel::UpdateFeedback() {
  if (controller_ == nullptr) {
    return DEVICE_ERR_NULL;
  }
  const int8_t ret = controller_->UpdateFeedback();
  if (ret == DEVICE_OK) {
    RefreshState();
  }
  return ret;
}

int8_t RmM3508Wheel::UpdateFeedback(WheelFeedback& feedback) {
  const int8_t ret = UpdateFeedback();
  if (ret != DEVICE_OK) {
    feedback = {};
    return ret;
  }

  feedback.position_rad = state_.position_rad;
  feedback.linear_velocity_mps = state_.linear_velocity_mps;
  feedback.output_torque_nm = state_.torque_nm;
  feedback.online = state_.online;
  feedback.timestamp_ms = state_.feedback_timestamp_ms;
  feedback.sequence = state_.feedback_sequence;
  return DEVICE_OK;
}

int8_t RmM3508Wheel::UpdateCommand() {
  if (controller_ == nullptr) {
    return DEVICE_ERR_NULL;
  }
  return controller_->UpdateCommand();
}

int8_t RmM3508Wheel::SetTorque(float torque_nm) {
  if (controller_ == nullptr) {
    return DEVICE_ERR_NULL;
  }
  return controller_->SetTorque(torque_nm);
}

int8_t RmM3508Wheel::SetPosition(float position_rad,
                                 float max_velocity_rad_s) {
  if (controller_ == nullptr) {
    return DEVICE_ERR_NULL;
  }
  return controller_->SetPosition(position_rad, max_velocity_rad_s);
}

int8_t RmM3508Wheel::CommitCommand() {
  if (controller_ == nullptr) {
    return DEVICE_ERR_NULL;
  }
  return controller_->CommitCommand();
}

bool RmM3508Wheel::HasPendingCommand() const {
  return controller_ != nullptr && controller_->HasPendingCommand();
}

void RmM3508Wheel::ClearPendingCommand() {
  if (controller_ == nullptr) {
    return;
  }
  controller_->ClearPendingCommand();
}

void RmM3508Wheel::RefreshState() {
  if (controller_ == nullptr) {
    state_ = {};
    return;
  }

  const mr::motor::MotorState motor_state = controller_->GetState();
  state_.position_rad = motor_state.position_rad;
  state_.angular_velocity_rad_s = motor_state.velocity_rad_s;
  state_.linear_velocity_mps = motor_state.velocity_rad_s * radius_m_;
  state_.torque_nm = motor_state.torque_nm;
  state_.temperature_c = motor_state.temperature_c;
  state_.temperature_warning = motor_state.temperature_warning;
  state_.temperature_over_limit = motor_state.temperature_over_limit;
  state_.temperature_limit_latched = motor_state.temperature_limit_latched;
  state_.online = motor_state.online;
  state_.feedback_timestamp_ms = motor_state.last_feedback_tick;
  state_.feedback_sequence = motor_state.feedback_sequence;
}

}  // namespace mr::wheel
