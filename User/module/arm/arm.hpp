#pragma once

// Runtime orchestration for the three-axis arm.
// The public flow is: Init -> Update -> Control -> Commit.

#include <stddef.h>
#include <stdint.h>
#include <array>

#include "module/arm/arm_command_adapter.hpp"
#include "module/arm/arm_control_types.h"
#include "module/arm/arm_model_config.hpp"
#include "module/arm/arm_output_adapter.hpp"
#include "device/joint/joint_interface.hpp"
#include "robotics/arm/control/arm_controller.hpp"

namespace mr::arm {

struct RuntimeDebugData {
  bool initialized = false;
  bool enabled = false;
  bool feedback_valid = false;
  bool command_accepted = false;
  bool workspace_rejected = false;
  bool last_commit_ok = false;
  bool reached = false;
  Arm_CtrlType_t ctrl_type = ARM_CTRL_REMOTE_CARTESIAN;
  ArmControlFrame_t frame = ARM_CTRL_FRAME_WORLD;
  mr::robotics::arm::Status status =
      mr::robotics::arm::Status::kNotConfigured;
  ArmPose_t current_pose{};
  ArmPose_t target_pose{};
  ArmJointAngles_t current_joints{};
  ArmJointAngles_t target_joints{};
  float joint_velocity[ARM_JOINT_COUNT]{};
  float control_position[ARM_JOINT_COUNT]{};
  float control_velocity[ARM_JOINT_COUNT]{};
  float control_acceleration[ARM_JOINT_COUNT]{};
  float control_torque_ff[ARM_JOINT_COUNT]{};
  float joint_temperature_c[ARM_JOINT_COUNT]{};
  bool joint_online[ARM_JOINT_COUNT]{};
  bool joint_temperature_warning[ARM_JOINT_COUNT]{};
  bool joint_temperature_over_limit[ARM_JOINT_COUNT]{};
  bool joint_temperature_limit_latched[ARM_JOINT_COUNT]{};
};

class Runtime final {
 public:
  struct Config {
    const Arm_Params_t* parameters = nullptr;
    // Optional app-owned actuators. If all slots are non-null, Runtime binds
    // them and does not create concrete motor objects.
    std::array<mr::IActuator*, ARM_JOINT_COUNT> actuators{};
  };

  Runtime();
  ~Runtime();

  Runtime(const Runtime&) = delete;
  Runtime& operator=(const Runtime&) = delete;

  bool Init(const Config& config, float control_freq_hz);
  void Update();
  void SetCommand(const Arm_CMD_t& command);
  void Control(float dt_s);
  void Commit();

  int8_t RelaxJoint(size_t index);
  int8_t RelaxAll();
  int8_t SetMotorZero(size_t index);

  const RuntimeDebugData& debug() const { return debug_; }
  RuntimeDebugData& debug() { return debug_; }
  const ThreeAxisArmModel& model() const { return model_; }
  const ThreeAxisJointState& feedback() const { return feedback_; }

  mr::motor::LzRso3Motor* joint1_motor() {
    return output_adapter_.Joint1Motor();
  }
  mr::motor::DmJ4340Motor* joint2_motor() {
    return output_adapter_.Joint2Motor();
  }
  mr::motor::DmJ4310PMotor* joint3_motor() {
    return output_adapter_.Joint3Motor();
  }

 private:
  using Controller = mr::robotics::arm::control::ArmController<ARM_JOINT_COUNT>;
  using ArmOutput = mr::robotics::arm::ArmOutput<ARM_JOINT_COUNT>;

  static float ClampDt(float dt_s, float control_freq_hz);
  static void ClearTransientCommand(Arm_CMD_t* command);
  static ArmPose_t ZeroPose();
  static ArmJointAngles_t JointAnglesFromVector(
      const ThreeAxisJointVector& values);

  bool InitializeController(float control_freq_hz);
  void ResetControllerIfNeeded();
  void RefreshDebug(const ArmCommandAdapterResult* adapter_result,
                    const ArmOutput* output);

  ThreeAxisArmModel model_;
  ThreeAxisControllerConfig controller_config_;
  alignas(Controller) uint8_t controller_storage_[sizeof(Controller)];
  Controller* controller_;
  ArmCommandAdapter command_adapter_;
  ArmOutputAdapter output_adapter_;
  ThreeAxisJointState feedback_;
  Arm_CMD_t command_{};
  ArmOutput last_output_;
  ArmControlFrame_t last_frame_ = ARM_CTRL_FRAME_WORLD;
  Arm_CtrlType_t last_ctrl_type_ = ARM_CTRL_REMOTE_CARTESIAN;
  bool last_enable_ = false;
  bool initialized_ = false;
  float control_freq_hz_ = 500.0f;
  Config config_{};
  const Arm_Params_t* parameters_ = nullptr;
  RuntimeDebugData debug_{};
};

}  // namespace mr::arm
