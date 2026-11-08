#include "module/arm/arm.hpp"
#include "module/chassis/chassis.hpp"
#include "component/communication/mrlink/mrlink.h"
#include <cmath>

// Minimal desktop consumer, not a firmware entry point or a simulated robot.
// Exercises public algorithms and a compiled C/C++ dependency without HAL.
namespace {
mr::module::arm::Arm<3> arm;
mr::module::chassis::Chassis chassis;
MrLink_t link;
uint8_t rx[128], tx[128];
}
int main() {
  if (arm.lifecycle() != mr::module::arm::Lifecycle::kUninitialized) return 1;
  (void)chassis;
  // Kinematics is selected by configuration; no specialised module is needed.
  mr::robotics::chassis::ChassisKinematics kinematics;
  mr::robotics::chassis::ChassisKinematicsConfig geometry;
  geometry.model = mr::robotics::chassis::ChassisModel::kMecanum;
  geometry.geometry = {0.4f, 0.4f};
  mr::robotics::chassis::WheelSpeeds wheels{};
  mr::robotics::chassis::PlanarVelocity velocity{};
  if (!kinematics.Configure(geometry) ||
      !kinematics.InverseKinematics({1.0f, 0.0f, 0.0f}, wheels) ||
      !kinematics.ForwardKinematics(wheels, velocity) ||
      std::fabs(velocity.vx_mps - 1.0f) > 1e-5f ||
      std::fabs(velocity.vy_mps) > 1e-5f || std::fabs(velocity.wz_rad_s) > 1e-5f) return 2;
  // RotationFromRpy is compiled in robotics_core, propagated by module_arm.
  const float rpy[] = {0.0f, 0.0f, 1.57079632679f};
  const auto rotation = mr::robotics::core::RotationFromRpy(mr::robotics::core::Vec3{rpy});
  if (std::fabs(rotation(0, 0)) > 1e-5f || std::fabs(rotation(1, 0) - 1.0f) > 1e-5f) return 3;
  const MrLink_Config_t config{MRLINK_MAX_PAYLOAD_DEFAULT, true, 0, 0};
  return MrLink_Init(&link, &config, rx, sizeof(rx), tx, sizeof(tx)) == MRLINK_OK ? 0 : 4;
}
