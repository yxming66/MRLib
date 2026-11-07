#include "module/arm/arm.hpp"
#include "module/chassis/chassis.hpp"
#include "component/communication/mrlink/mrlink.h"

// Minimal desktop consumer, not a firmware entry point or a simulated robot.
// Demonstrates direct library linking and fixed storage without Python/YAML.
namespace {
mr::module::arm::Arm<3> arm;
mr::module::chassis::Chassis chassis;
MrLink_t link;
uint8_t rx[128], tx[128];
}
int main() {
  if (arm.lifecycle() != mr::module::arm::Lifecycle::kUninitialized) return 1;
  (void)chassis;
  const MrLink_Config_t config{MRLINK_MAX_PAYLOAD_DEFAULT, true, 0, 0};
  return MrLink_Init(&link, &config, rx, sizeof(rx), tx, sizeof(tx)) == MRLINK_OK ? 0 : 1;
}
