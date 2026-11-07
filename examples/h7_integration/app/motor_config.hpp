#pragma once
#include "device/motor/bus/rm_command_bus.hpp"
#include "device/motor/core/motor_instance_config.hpp"

namespace app {
// Handwritten counterpart of config.yaml until v2 generation is implemented.
inline mr::motor::MotorInstanceConfig<mr::motor::MotorKind::RM> WheelConfig(
    mr::motor::CanPort& port, mr::motor::RmCommandBus& commands, uint16_t feedback_id) {
  mr::motor::MotorInstanceConfig<mr::motor::MotorKind::RM> config;
  config.port = &port;
  config.commands = &commands;
  config.id = feedback_id;
  return config;
}
inline mr::motor::MotorInstanceConfig<mr::motor::MotorKind::DM> JointConfig(
    mr::motor::CanPort& port, uint16_t index) {
  mr::motor::MotorInstanceConfig<mr::motor::MotorKind::DM> config;
  config.port = &port;
  // indices 0..2: reserve 0x11..0x13, 0x111..0x113, 0x211..0x213,
  // plus feedback 0x321..0x323; no overlap with RM 0x200/0x201..0x204.
  config.can_id = 0x11 + index;
  config.master_id = 0x321 + index;
  config.mit_position_range_rad = 12.5f;
  config.mit_velocity_range_rad_s = 30;
  config.mit_torque_range_nm = 10;
  return config;
}
}  // namespace app
