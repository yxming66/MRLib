#pragma once
#include "arm_model_config.hpp"
#include "arm_command_adapter.hpp"
#include "module/arm/arm.hpp"

namespace mr::examples::three_axis_arm {
using Module = mr::module::arm::Arm<ARM_JOINT_COUNT>;
inline Module::Config MakeConfig(const Arm_Params_t& parameters,
    const std::array<mr::IActuator*, ARM_JOINT_COUNT>& actuators) {
  Module::Config config;
  config.model = mr::arm::BuildThreeAxisArmModel(parameters);
  config.controller = mr::arm::BuildThreeAxisControllerConfig(parameters, 0.002f);
  config.output.actuators = actuators;
  for (int i = 0; i < ARM_JOINT_COUNT; ++i) {
    config.output.gains[i] = {parameters.joint_kp[i], parameters.joint_kd[i],
                            parameters.gravity_comp_scale[i]};
  }
  return config;
}
// Call UpdateFeedback first. The app decides when to Enable/Disable and handles
// every error; input normalization/workspace policy stays outside the library.
inline int8_t Control(Module& arm, mr::arm::ArmCommandAdapter& remote,
                      const Arm_CMD_t& input, float dt_s) {
  if (!input.enable) return arm.Disable();
  const auto pose = mr::robotics::arm::kinematics::ForwardKinematics(
      arm.model(), arm.feedback().position);
  const auto adapted = remote.Build(input, arm.feedback(), pose, dt_s);
  if (!adapted.accepted) return DEVICE_ERR;
  auto result = arm.SetCommand(adapted.command);
  if (result == DEVICE_OK) result = arm.Step(dt_s);
  if (result == DEVICE_OK) result = arm.CommitOutput();
  return result;
}
}  // namespace mr::examples::three_axis_arm
