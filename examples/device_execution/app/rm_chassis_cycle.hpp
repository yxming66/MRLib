#pragma once
#include "device/motor/bus/rm_command_bus.hpp"
#include "module/chassis/chassis.hpp"

namespace example {
struct ChassisCycleResult {
  mr::module::chassis::ChassisStatus control = mr::module::chassis::ChassisStatus::kOk;
  mr::module::chassis::ChassisStatus relax = mr::module::chassis::ChassisStatus::kOk;
  int8_t flush = 0;
  int8_t stop_flush = 0;
  bool ok() const { return control == mr::module::chassis::ChassisStatus::kOk && flush == 0; }
};

// Application policy: any chassis failure stops ALL RM groups on this bus.
// Board initializes CAN first; app owns/registers/enables chassis and devices.
// After any failure the app latches a fault and requires explicit re-enable.
inline ChassisCycleResult RunChassisCycle(mr::module::chassis::Chassis& chassis,
                                          mr::motor::RmCommandBus& bus, uint32_t now_ms,
                                          const mr::robotics::chassis::PlanarVelocity& command,
                                          float dt_s) {
  using mr::module::chassis::ChassisStatus;
  ChassisCycleResult result;
  bus.BeginCycle();
  result.control = chassis.UpdateFeedback(now_ms);
  if (result.control == ChassisStatus::kOk) result.control = chassis.SetCommand(command);
  if (result.control == ChassisStatus::kOk) result.control = chassis.Step(dt_s);
  if (result.control == ChassisStatus::kOk) result.control = chassis.CommitOutput();
  if (result.control != ChassisStatus::kOk) {
    result.relax = chassis.Relax();
    // Discard every staged effort, even if the module's Relax failed.
    bus.BeginCycle();
  }
  result.flush = bus.Flush();  // The single common exit, including errors.
  if (result.flush != 0) {
    // Do not retry an old nonzero command after a transmission failure.
    bus.BeginCycle();
    result.relax = chassis.Relax();
    result.stop_flush = bus.Flush();
  }
  return result;
}
}  // namespace example
