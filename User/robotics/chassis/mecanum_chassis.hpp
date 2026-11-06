#pragma once

/*
 * 四麦克纳姆轮运动学模型。
 *
 * 快速用法：
 *   MecanumChassis chassis(geometry);
 *   MecanumChassis::WheelSpeeds wheels{};
 *   chassis.InverseKinematics({vx, vy, wz}, wheels);
 *   PlanarVelocity body{};
 *   chassis.ForwardKinematics(wheels, body);
 *
 * vx/vy 使用 m/s，wz 使用 rad/s；轮速是轮缘线速度 m/s。
 * 几何参数必须与轮子编号约定一致。
 */

// 麦克纳姆底盘运动学模型。
// 使用方式：构造 MecanumGeometry 和 MecanumKinematics，调用
// ChassisVelocityToWheelVelocity() 将车体速度转换为四轮轮缘线速度，
// 或调用 WheelVelocityToChassisVelocity() 做反馈反解。线速度单位为 m/s，
// 角速度单位为 rad/s。

#include <array>

#include "component/math/scalar.hpp"
#include "robotics/chassis/types.hpp"

namespace mr::robotics::chassis {

struct MecanumGeometry {
  // L 和 W 为车体中心到车轮接地点中心的距离。
  // 实际前后/左右轮距分别为 2 * wheelbase_m 和 2 * trackwidth_m。
  float wheelbase_m = 0.0f;
  float trackwidth_m = 0.0f;

  float RotationRadius() const { return wheelbase_m + trackwidth_m; }

  bool IsValid() const {
    return mr::component::math::is_finite_scalar(wheelbase_m) &&
           mr::component::math::is_finite_scalar(trackwidth_m) &&
           wheelbase_m >= 0.0f && trackwidth_m >= 0.0f &&
           RotationRadius() > 1e-5f;
  }
};

enum class MecanumWheel : unsigned {
  kFrontRight = 0,
  kFrontLeft = 1,
  kRearLeft = 2,
  kRearRight = 3,
};

class MecanumChassis {
 public:
  static constexpr unsigned kWheelCount = 4U;
  using WheelSpeeds = std::array<float, kWheelCount>;

  constexpr MecanumChassis() = default;
  explicit constexpr MecanumChassis(const MecanumGeometry& geometry)
      : geometry_(geometry) {}

  void SetGeometry(const MecanumGeometry& geometry) { geometry_ = geometry; }
  const MecanumGeometry& geometry() const { return geometry_; }
  bool IsValid() const { return geometry_.IsValid(); }

  bool InverseKinematics(const PlanarVelocity& body_velocity,
                         WheelSpeeds& wheel_speeds) const {
    if (!IsValid() || !IsFinite(body_velocity)) {
      wheel_speeds = {};
      return false;
    }

    const float wz_term = geometry_.RotationRadius() * body_velocity.wz_rad_s;
    wheel_speeds[MecanumIndex(MecanumWheel::kFrontRight)] =
        -body_velocity.vx_mps - body_velocity.vy_mps - wz_term;
    wheel_speeds[MecanumIndex(MecanumWheel::kFrontLeft)] =
        body_velocity.vx_mps - body_velocity.vy_mps - wz_term;
    wheel_speeds[MecanumIndex(MecanumWheel::kRearLeft)] =
        body_velocity.vx_mps + body_velocity.vy_mps - wz_term;
    wheel_speeds[MecanumIndex(MecanumWheel::kRearRight)] =
        -body_velocity.vx_mps + body_velocity.vy_mps - wz_term;
    return true;
  }

  bool ForwardKinematics(const WheelSpeeds& wheel_speeds,
                         PlanarVelocity& body_velocity) const {
    if (!IsValid() || !IsFinite(wheel_speeds)) {
      body_velocity = {};
      return false;
    }

    const float v0 = wheel_speeds[MecanumIndex(MecanumWheel::kFrontRight)];
    const float v1 = wheel_speeds[MecanumIndex(MecanumWheel::kFrontLeft)];
    const float v2 = wheel_speeds[MecanumIndex(MecanumWheel::kRearLeft)];
    const float v3 = wheel_speeds[MecanumIndex(MecanumWheel::kRearRight)];
    const float k = geometry_.RotationRadius();

    body_velocity.vx_mps = 0.25f * (-v0 + v1 + v2 - v3);
    body_velocity.vy_mps = 0.25f * (-v0 - v1 + v2 + v3);
    body_velocity.wz_rad_s = -0.25f * (v0 + v1 + v2 + v3) / k;
    return true;
  }

  static float MaxAbsWheelSpeed(const WheelSpeeds& wheel_speeds) {
    return mr::robotics::chassis::MaxAbsWheelSpeed(wheel_speeds);
  }

  static bool ScaleWheelSpeedsToLimit(WheelSpeeds& wheel_speeds,
                                      float max_abs_speed_mps) {
    return mr::robotics::chassis::ScaleWheelSpeedsToLimit(
        wheel_speeds, max_abs_speed_mps);
  }

 private:
  static constexpr unsigned MecanumIndex(MecanumWheel wheel) {
    return static_cast<unsigned>(wheel);
  }

  static bool IsFinite(const PlanarVelocity& velocity) {
    return mr::robotics::chassis::IsFinite(velocity);
  }

  static bool IsFinite(const WheelSpeeds& wheel_speeds) {
    return mr::robotics::chassis::IsFinite(wheel_speeds);
  }

  MecanumGeometry geometry_{};
};

}  // namespace mr::robotics::chassis
