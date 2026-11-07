#pragma once

#include "motor_config.hpp"
#include "robot_params.hpp"
#include "device/joint/motor_actuator_adapter.hpp"
#include "device/motor/motor.hpp"
#include "module/arm/arm.hpp"

namespace app {
// Three statically owned joints. Changing the mechanism is an explicit code change.
// No heap, runtime joint factory or variable-sized joint container.
class ArmAssembly final {
 public:
  using ArmType = mr::module::arm::Arm<params::kArmJointCount>;
  static_assert(params::kArmJointCount == 3, "Update the explicit device wiring for a new mechanism");
  explicit ArmAssembly(mr::motor::CanPort& port)
      : motor_0(JointConfig(port, 0), {params::kExternalRatio}),
        motor_1(JointConfig(port, 1), {params::kExternalRatio}),
        motor_2(JointConfig(port, 2), {params::kExternalRatio}),
        actuator_0(&motor_0), actuator_1(&motor_1), actuator_2(&motor_2),
        joint_0(0, &actuator_0, JointLimits(0), params::kJoints[0].zero_rad, kFrequencyHz, params::kJoints[0].name),
        joint_1(1, &actuator_1, JointLimits(1), params::kJoints[1].zero_rad, kFrequencyHz, params::kJoints[1].name),
        joint_2(2, &actuator_2, JointLimits(2), params::kJoints[2].zero_rad, kFrequencyHz, params::kJoints[2].name) {}
  ArmAssembly(const ArmAssembly&) = delete;
  ArmAssembly& operator=(const ArmAssembly&) = delete;

  ArmType::Config Config() {
    ArmType::Config config;
    config.controller.default_dt_s = params::kControlPeriodUs / 1000000.0f;
    config.controller.enable_gravity_compensation = false;
    mr::robotics::arm::JointModel models[params::kArmJointCount]{};
    for (int i = 0; i < params::kArmJointCount; ++i) {
      const auto& p = params::kJoints[i];
      models[i].axis_in_joint_frame = Vector(p.axis);
      models[i].parent_to_joint = Transform(p.translation_m, p.rpy_rad);
      models[i].limits.position_enabled = true;
      models[i].limits.position_min = p.min_rad;
      models[i].limits.position_max = p.max_rad;
      models[i].limits.max_velocity = p.max_velocity_rad_s;
      models[i].limits.max_acceleration = p.max_acceleration_rad_s2;
      // Position offset belongs to JointT only; never subtract it twice.
      config.output.gains[i] = {p.gains.kp, p.gains.kd, p.gains.effort_scale};
    }
    config.model = {models, Transform(params::kBaseTranslationM, params::kBaseRpyRad),
                    Transform(params::kToolTranslationM, params::kToolRpyRad),
                    Vector(params::kGravityWorld)};
    config.output.actuators = {&joint_0, &joint_1, &joint_2};
    return config;
  }
  int8_t Init() { return arm.Init(Config()); }

  mr::motor::DmJ4310Motor motor_0, motor_1, motor_2;
  mr::MotorActuatorAdapter<mr::motor::DmJ4310Motor> actuator_0, actuator_1, actuator_2;
  mr::JointT joint_0, joint_1, joint_2;
  ArmType arm;

 private:
  static constexpr float kFrequencyHz = 1000000.0f / params::kControlPeriodUs;
  static mr::JointControlParams JointLimits(int i) {
    const auto& p = params::kJoints[i];
    return {p.min_rad, p.max_rad, 0, 0};
  }
  static mr::robotics::core::Vec3 Vector(const std::array<float, 3>& p) {
    const float xyz[] = {p[0], p[1], p[2]};
    return mr::robotics::core::Vec3{xyz};
  }
  static mr::robotics::core::Transform Transform(const std::array<float, 3>& xyz,
                                                const std::array<float, 3>& rpy) {
    return mr::robotics::core::MakeTransform(mr::robotics::core::RotationFromRpy(Vector(rpy)), Vector(xyz));
  }
};
}  // namespace app
