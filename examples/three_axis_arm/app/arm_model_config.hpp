#pragma once

// Three-axis robot-specific data. This file is the only place where the
// generic arm model is filled with this robot's geometry and limits.

#include "arm_control_types.h"
#include "robotics/arm/control/arm_controller.hpp"
#include "robotics/arm/model/arm_model.hpp"

namespace mr::arm {

using ThreeAxisArmModel = mr::robotics::arm::ArmModel<ARM_JOINT_COUNT>;
using ThreeAxisControllerConfig =
    mr::robotics::arm::control::ArmControllerConfig<ARM_JOINT_COUNT>;

ThreeAxisArmModel BuildThreeAxisArmModel(const Arm_Params_t& parameters);

ThreeAxisControllerConfig BuildThreeAxisControllerConfig(
    const Arm_Params_t& parameters,
    float default_dt_s);

}  // namespace mr::arm
