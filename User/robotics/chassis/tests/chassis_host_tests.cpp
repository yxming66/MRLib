#include <cmath>
#include <cstdio>

#include "robotics/chassis/chassis_kinematics.hpp"

namespace {

using mr::robotics::chassis::ChassisKinematics;
using mr::robotics::chassis::ChassisKinematicsConfig;
using mr::robotics::chassis::ChassisModel;
using mr::robotics::chassis::PlanarVelocity;
using mr::robotics::chassis::WheelSpeeds;

int failures = 0;

void Expect(bool condition, const char* message) {
  if (!condition) {
    std::printf("FAIL: %s\n", message);
    ++failures;
  }
}

bool Near(float lhs, float rhs, float tolerance = 1.0e-5f) {
  return std::fabs(lhs - rhs) <= tolerance;
}

void TestMecanum() {
  ChassisKinematics model;
  Expect(model.Configure({ChassisModel::kMecanum, {0.25f, 0.20f}}),
         "mecanum configuration should be valid");

  const PlanarVelocity command{1.0f, 0.2f, 0.5f};
  WheelSpeeds wheels{};
  Expect(model.InverseKinematics(command, wheels),
         "mecanum inverse kinematics should succeed");

  PlanarVelocity recovered{};
  Expect(model.ForwardKinematics(wheels, recovered),
         "mecanum forward kinematics should succeed");
  Expect(Near(recovered.vx_mps, command.vx_mps) &&
             Near(recovered.vy_mps, command.vy_mps) &&
             Near(recovered.wz_rad_s, command.wz_rad_s),
         "mecanum forward and inverse kinematics should round trip");
}

void TestFrontOmniRearMecanum() {
  ChassisKinematics model;
  Expect(model.Configure(
             {ChassisModel::kFrontOmniRearMecanum, {0.25f, 0.20f}}),
         "front omni rear mecanum configuration should be valid");

  const PlanarVelocity command{0.6f, 0.3f, 0.4f};
  WheelSpeeds wheels{};
  Expect(model.InverseKinematics(command, wheels),
         "front omni rear mecanum inverse kinematics should succeed");

  PlanarVelocity recovered{};
  Expect(model.ForwardKinematics(wheels, recovered),
         "front omni rear mecanum forward kinematics should succeed");
  Expect(Near(recovered.vx_mps, command.vx_mps) &&
             Near(recovered.vy_mps, command.vy_mps) &&
             Near(recovered.wz_rad_s, command.wz_rad_s),
         "front omni rear mecanum should round trip");
}

void TestInvalidConfiguration() {
  ChassisKinematics model;
  Expect(!model.Configure({ChassisModel::kMecanum, {0.0f, 0.0f}}),
         "zero geometry should be rejected");

  Expect(model.Configure({ChassisModel::kMecanum, {0.25f, 0.2f}}),
         "valid geometry should configure after a failed attempt");
  WheelSpeeds below_limit{1.0f, -2.0f, 3.0f, -4.0f};
  Expect(ChassisKinematics::ScaleWheelSpeedsToLimit(below_limit, 4.0f),
         "a valid wheel speed vector below its limit should succeed");
  WheelSpeeds above_limit{2.0f, -4.0f, 6.0f, -8.0f};
  Expect(ChassisKinematics::ScaleWheelSpeedsToLimit(above_limit, 4.0f),
         "a wheel speed vector above its limit should scale successfully");
  Expect(Near(ChassisKinematics::MaxAbsWheelSpeed(above_limit), 4.0f),
         "wheel speed scaling should preserve the requested maximum");
}

}  // namespace

int main() {
  TestMecanum();
  TestFrontOmniRearMecanum();
  TestInvalidConfiguration();
  if (failures != 0) {
    std::printf("%d chassis host test(s) failed\n", failures);
    return 1;
  }
  std::printf("chassis host tests passed\n");
  return 0;
}
