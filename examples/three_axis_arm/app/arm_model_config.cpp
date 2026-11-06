#include "arm_model_config.hpp"

#include "component/math/matrix.hpp"
#include "component/math/scalar.hpp"
#include "robotics/core/rotation.hpp"
#include "robotics/core/transform.hpp"
#include "robotics/arm/kinematics/inverse_kinematics.hpp"

namespace mr::arm {
namespace {

using mr::robotics::arm::JointLimits;
using mr::robotics::arm::JointModel;
using mr::robotics::arm::JointType;
using mr::robotics::arm::Transform;
using mr::robotics::arm::Vec3;

Vec3 MakeVec3(float x, float y, float z) {
  Vec3 value = mr::component::math::matrix::zeros<3, 1>();
  value[0][0] = x;
  value[1][0] = y;
  value[2][0] = z;
  return value;
}

Transform MakeRpyTransform(float x,
                          float y,
                          float z,
                          float roll_rad,
                          float pitch_rad,
                          float yaw_rad) {
  const Vec3 rpy = MakeVec3(roll_rad, pitch_rad, yaw_rad);
  return mr::robotics::core::MakeTransform(
      mr::robotics::core::RotationFromRpy(rpy), MakeVec3(x, y, z));
}

float PositiveOr(float value, float fallback) {
  return mr::component::math::is_positive_scalar(value) ? value : fallback;
}

JointLimits MakeLimits(const Arm_Params_t& parameters,
                       int index,
                       float fallback_min,
                       float fallback_max) {
  JointLimits limits;
  limits.position_enabled = true;
  limits.position_min = parameters.joint_soft_limit_lower[index];
  limits.position_max = parameters.joint_soft_limit_upper[index];
  if (!mr::component::math::is_finite_scalar(limits.position_min) ||
      !mr::component::math::is_finite_scalar(limits.position_max) ||
      limits.position_min >= limits.position_max) {
    limits.position_min = fallback_min;
    limits.position_max = fallback_max;
  }
  limits.max_velocity = PositiveOr(
      parameters.remote_cartesian.joint_max_velocity[index], 2.0f);
  limits.max_acceleration = PositiveOr(
      parameters.remote_cartesian.joint_max_acceleration[index], 8.0f);
  return limits;
}

JointModel MakeRevoluteJoint(const Transform& parent_to_joint,
                             const Vec3& center_of_mass,
                             float mass_kg,
                             const JointLimits& limits) {
  JointModel joint;
  joint.type = JointType::kRevolute;
  joint.parent_to_joint = parent_to_joint;
  joint.axis_in_joint_frame = MakeVec3(0.0f, 0.0f, -1.0f);
  joint.position_offset = 0.0f;
  joint.limits = limits;
  joint.inertial.mass_kg = mass_kg;
  joint.inertial.center_of_mass = center_of_mass;
  return joint;
}

}  // namespace

ThreeAxisArmModel BuildThreeAxisArmModel(const Arm_Params_t& parameters) {
  const JointLimits joint1_limits =
      MakeLimits(parameters, 0, -0.87f, 2.0f);
  const JointLimits joint2_limits =
      MakeLimits(parameters, 1, -2.20f, 2.20f);
  const JointLimits joint3_limits =
      MakeLimits(parameters, 2, -1.84f, 1.75f);

  JointModel joints[ARM_JOINT_COUNT] = {
      MakeRevoluteJoint(
          MakeRpyTransform(0.0f, 0.0f, 0.07f, 1.5708f, 0.0f, 1.57079265f),
          MakeVec3(-0.00047343f, 0.28954791f, 0.0005662f), 0.54717f,
          joint1_limits),
      MakeRevoluteJoint(
          MakeRpyTransform(-0.0005f, 0.35f, 0.0f, 0.0f, 0.0f, 0.0f),
          MakeVec3(-0.00034549f, 0.22380246f, 0.00724804f), 0.5643f,
          joint2_limits),
      MakeRevoluteJoint(
          MakeRpyTransform(-0.000357142857f, 0.249999745f, -0.002625f,
                           0.0f, 0.0f, 0.0f),
          MakeVec3(0.002693f, 0.11635f, 0.0019852f), 0.256f,
          joint3_limits),
  };

  const Transform base_to_world =
      mr::component::math::matrix::identity<4>();
  const Transform tool_to_last_joint =
      mr::robotics::core::TransformFromTranslation(
          MakeVec3(0.0034791f, 0.14696f, -0.000025f));
  const Vec3 gravity_world = MakeVec3(0.0f, 0.0f, -9.81f);
  return ThreeAxisArmModel(joints, base_to_world, tool_to_last_joint,
                           gravity_world);
}

ThreeAxisControllerConfig BuildThreeAxisControllerConfig(
    const Arm_Params_t& parameters,
    float default_dt_s) {
  ThreeAxisControllerConfig config;
  config.default_dt_s = PositiveOr(default_dt_s, 0.002f);

  const ArmCartesianRemoteParam_t& remote = parameters.remote_cartesian;
  config.max_linear_velocity_m_s =
      PositiveOr(remote.max_linear_velocity, 0.20f);
  config.max_angular_velocity_rad_s =
      PositiveOr(remote.max_angular_velocity, 1.0f);
  config.max_linear_acceleration_m_s2 =
      PositiveOr(remote.max_linear_acceleration, 0.50f);
  config.max_angular_acceleration_rad_s2 =
      PositiveOr(remote.max_angular_acceleration, 2.0f);

  config.inverse_kinematics =
      mr::robotics::arm::kinematics::MakePositionOnlyInverseKinematicsOptions();
  config.inverse_kinematics.max_iterations = 24;
  config.inverse_kinematics.position_tolerance = 2.0e-3f;
  config.inverse_kinematics.orientation_tolerance = 2.0e-2f;
  config.inverse_kinematics.damping = 5.0e-3f;
  config.inverse_kinematics.max_joint_step =
      PositiveOr(remote.max_joint_step, 0.04f);
  config.inverse_kinematics.enforce_position_limits = true;
  config.enable_gravity_compensation = true;
  config.hold_on_failure = true;
  return config;
}

}  // namespace mr::arm
