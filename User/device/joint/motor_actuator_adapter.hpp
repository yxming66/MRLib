#pragma once

// Optional bridge from the generic actuator contract to the motor subsystem.

#include "device/joint/joint_interface.hpp"
#include "device/motor/motor.hpp"

namespace mr {

template <typename MotorType>
class MotorActuatorAdapter final : public IActuator {
 public:
  MotorActuatorAdapter() = default;
  explicit MotorActuatorAdapter(MotorType* motor) : motor_(motor) {}

  void SetMotor(MotorType* motor) { motor_ = motor; }
  MotorType* GetMotor() const { return motor_; }

  int8_t Register() override {
    return motor_ ? motor_->Register() : DEVICE_ERR_NULL;
  }
  int8_t Enable() override {
    return motor_ ? motor_->Enable() : DEVICE_ERR_NULL;
  }
  int8_t Update() override {
    return motor_ ? motor_->Update() : DEVICE_ERR_NULL;
  }
  int8_t Relax() override {
    return motor_ ? motor_->Relax() : DEVICE_ERR_NULL;
  }
  int8_t SetZero() override {
    return motor_ ? motor_->SetZero() : DEVICE_ERR_NULL;
  }
  int8_t CommitControl() override {
    return motor_ ? motor_->CommitCommand() : DEVICE_ERR_NULL;
  }
  bool HasPendingControl() const override {
    return motor_ && motor_->HasPendingCommand();
  }
  void ClearPendingControl() override {
    if (motor_) {
      motor_->ClearPendingCommand();
    }
  }
  int8_t SetMIT(float position, float velocity, float kp, float kd,
                float torque_ff) override {
    return motor_ ? motor_->SetMIT(position, velocity, kp, kd, torque_ff)
                  : DEVICE_ERR_NULL;
  }
  int8_t SetTorque(float torque) override {
    return motor_ ? motor_->SetTorque(torque) : DEVICE_ERR_NULL;
  }
  ActuatorState GetState() const override {
    ActuatorState state{};
    if (!motor_) {
      return state;
    }
    const mr::motor::MotorState motor_state = motor_->GetState();
    state.position = motor_state.position_rad;
    state.velocity = motor_state.velocity_rad_s;
    state.torque = motor_state.torque_nm;
    state.online = motor_state.online && motor_state.feedback_valid && motor_state.position_valid &&
                   !motor_state.protocol_fault && !motor_state.temperature_limit_latched;
    state.timestamp_ms = motor_state.last_feedback_tick;
    state.sequence = motor_state.feedback_sequence;
    return state;
  }

 private:
  MotorType* motor_ = nullptr;
};

using DmJ4310Actuator = MotorActuatorAdapter<mr::motor::DmJ4310Motor>;
using DmJ4310PActuator = MotorActuatorAdapter<mr::motor::DmJ4310PMotor>;
using DmJ4340Actuator = MotorActuatorAdapter<mr::motor::DmJ4340Motor>;
using DmActuator = DmJ4310Actuator;
using LzActuator = MotorActuatorAdapter<mr::motor::LzRso3Motor>;

}  // namespace mr
