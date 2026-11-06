#pragma once

// Fixed-size actuator boundary for the three arm joints. The generic arm
// controller stops at joint coordinates; normal assembly binds IActuator
// objects supplied by the app. The legacy Initialize path is the only place
// that still knows the historical concrete motor protocols.

#include <array>
#include <stddef.h>
#include <stdint.h>

#include "device/joint/joint_interface.hpp"
#include "device/joint/motor_actuator_adapter.hpp"
#include "device/motor/core/motor_aliases.hpp"
#include "device/motor/core/motor_state.hpp"
#include "device/motor/motor.hpp"
#include "module/arm/arm_control_types.h"
#include "robotics/arm/core/types.hpp"

namespace mr::arm {

using ThreeAxisJointState =
    mr::robotics::arm::JointState<ARM_JOINT_COUNT>;
using ThreeAxisJointCommand =
    mr::robotics::arm::JointCommand<ARM_JOINT_COUNT>;

class ArmOutputAdapter final {
 public:
  ArmOutputAdapter();

  ArmOutputAdapter(const ArmOutputAdapter&) = delete;
  ArmOutputAdapter& operator=(const ArmOutputAdapter&) = delete;

  bool Initialize(const Arm_Params_t& parameters);
  bool Bind(const std::array<mr::IActuator*, ARM_JOINT_COUNT>& actuators,
            const Arm_Params_t& parameters);
  bool IsInitialized() const { return initialized_; }

  int8_t Register();
  int8_t Enable();
  int8_t Relax();
  int8_t Relax(size_t index);
  int8_t UpdateFeedback(ThreeAxisJointState* state);
  int8_t Apply(const ThreeAxisJointCommand& command);
  int8_t Commit();
  int8_t SetZero(size_t index);

  mr::motor::MotorState MotorState(size_t index) const;
  bool IsOnline(size_t index) const;

  mr::motor::LzRso3Motor* Joint1Motor() { return joint1_motor_; }
  mr::motor::DmJ4340Motor* Joint2Motor() { return joint2_motor_; }
  mr::motor::DmJ4310PMotor* Joint3Motor() { return joint3_motor_; }

 private:
  static mr::motor::MotorTemperatureProtectionConfig
  MakeTemperatureProtection(
      const MOTOR_TemperatureProtectionConfig_t& parameters);

  int8_t FirstFailure(int8_t current, int8_t next) const;
  int8_t ApplyJoint(size_t index, const ThreeAxisJointCommand& command);
  void LoadControlParameters(const Arm_Params_t& parameters);
  void RefreshMotorState(size_t index);

  mr::motor::LzRso3Motor* joint1_motor_;
  mr::motor::DmJ4340Motor* joint2_motor_;
  mr::motor::DmJ4310PMotor* joint3_motor_;
  float kp_[ARM_JOINT_COUNT];
  float kd_[ARM_JOINT_COUNT];
  float gravity_scale_[ARM_JOINT_COUNT];
  std::array<mr::IActuator*, ARM_JOINT_COUNT> actuators_{};
  std::array<mr::motor::MotorState, ARM_JOINT_COUNT> motor_states_{};
  mr::LzActuator joint1_actuator_;
  mr::DmJ4340Actuator joint2_actuator_;
  mr::DmJ4310PActuator joint3_actuator_;
  bool initialized_;
};

}  // namespace mr::arm
