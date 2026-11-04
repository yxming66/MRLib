#pragma once

#include <cstdint>

#include "robotics/chassis/front_omni_rear_mecanum_chassis.hpp"
#include "robotics/chassis/mecanum_chassis.hpp"
#include "robotics/chassis/types.hpp"

namespace mr::robotics::chassis {

/** Kinematic models available to the generic chassis module. */
enum class ChassisModel : uint8_t {
  kMecanum = 0,
  kFrontOmniRearMecanum = 1,
};

struct ChassisGeometry {
  float wheelbase_m = 0.0f;
  float trackwidth_m = 0.0f;
};

struct ChassisKinematicsConfig {
  ChassisModel model = ChassisModel::kMecanum;
  ChassisGeometry geometry{};
};

/**
 * Fixed-storage dispatcher for supported kinematic models.
 *
 * Selecting a model changes the algorithm used by the module. It does not
 * allocate memory and it does not depend on a motor model or a board SDK.
 */
class ChassisKinematics final {
 public:
  static constexpr std::size_t kWheelCount =
      ::mr::robotics::chassis::kWheelCount;

  bool Configure(const ChassisKinematicsConfig& config) {
    model_ = config.model;
    switch (model_) {
      case ChassisModel::kMecanum:
        mecanum_.SetGeometry(
            {config.geometry.wheelbase_m, config.geometry.trackwidth_m});
        return mecanum_.IsValid();
      case ChassisModel::kFrontOmniRearMecanum:
        front_omni_rear_mecanum_.SetGeometry(
            {config.geometry.wheelbase_m, config.geometry.trackwidth_m});
        return front_omni_rear_mecanum_.IsValid();
      default:
        return false;
    }
  }

  ChassisModel model() const { return model_; }

  bool IsValid() const {
    switch (model_) {
      case ChassisModel::kMecanum:
        return mecanum_.IsValid();
      case ChassisModel::kFrontOmniRearMecanum:
        return front_omni_rear_mecanum_.IsValid();
      default:
        return false;
    }
  }

  bool InverseKinematics(const PlanarVelocity& body_velocity,
                         WheelSpeeds& wheel_speeds) const {
    switch (model_) {
      case ChassisModel::kMecanum:
        return mecanum_.InverseKinematics(body_velocity, wheel_speeds);
      case ChassisModel::kFrontOmniRearMecanum:
        return front_omni_rear_mecanum_.InverseKinematics(body_velocity,
                                                          wheel_speeds);
      default:
        wheel_speeds = {};
        return false;
    }
  }

  bool ForwardKinematics(const WheelSpeeds& wheel_speeds,
                         PlanarVelocity& body_velocity) const {
    switch (model_) {
      case ChassisModel::kMecanum:
        return mecanum_.ForwardKinematics(wheel_speeds, body_velocity);
      case ChassisModel::kFrontOmniRearMecanum:
        return front_omni_rear_mecanum_.ForwardKinematics(wheel_speeds,
                                                           body_velocity);
      default:
        body_velocity = {};
        return false;
    }
  }

  static float MaxAbsWheelSpeed(const WheelSpeeds& wheel_speeds) {
    return mr::component::math::max_abs_array(wheel_speeds.data(),
                                              wheel_speeds.size());
  }

  static bool ScaleWheelSpeedsToLimit(WheelSpeeds& wheel_speeds,
                                      float max_abs_speed) {
    if (!mr::robotics::chassis::IsFinite(wheel_speeds) ||
        !mr::component::math::is_positive_scalar(max_abs_speed)) {
      return false;
    }
    if (MaxAbsWheelSpeed(wheel_speeds) <= max_abs_speed) {
      return true;
    }
    return mr::component::math::scale_to_abs_limit(
        wheel_speeds.data(), wheel_speeds.size(), max_abs_speed);
  }

 private:
  ChassisModel model_ = ChassisModel::kMecanum;
  MecanumChassis mecanum_{};
  FrontOmniRearMecanumChassis front_omni_rear_mecanum_{};
};

}  // namespace mr::robotics::chassis
