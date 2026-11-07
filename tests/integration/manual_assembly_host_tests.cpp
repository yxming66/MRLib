// Compile and exercise the same handwritten assembly used by the H7 firmware.
// Only the bottom CAN port is simulated; no HAL or generator is involved.
#include <cassert>
#include <cmath>
#include <type_traits>
#include "../../examples/h7_integration/app/arm_assembly.hpp"
#include "../../examples/h7_integration/app/chassis_assembly.hpp"
#include "support/fake_can_port.hpp"

int main() {
  FakeCanPort port;
  app::ChassisAssembly chassis(port);
  app::ArmAssembly arm(port);
  static_assert(std::is_same_v<app::ArmAssembly::ArmType, mr::module::arm::Arm<3>>);
  assert(chassis.Register(app::params::kControlPeriodUs) == mr::module::chassis::ChassisStatus::kOk);
  assert(arm.Init() == DEVICE_OK);  // All seven addresses register without collision.
  assert(port.sent.empty());       // Registration never enables or sends.
  auto config = arm.Config();
  assert(config.model.Validate());
  assert(config.output.actuators[0] == &arm.joint_0);
  assert(config.model.Joint(0).position_offset == 0);
  assert(std::fabs(config.model.Joint(1).parent_to_joint(0, 3) - 0.1f) < 1e-6f);
  assert(config.controller.default_dt_s == 0.001f);
  assert(app::params::ChassisConfig(2000).wheel_velocity_pid.control_freq_hz == 500.0f);
  // Real DM protocol feedback at approximately zero position/velocity/effort.
  for (const auto& joint : app::params::kJoints) {
    mr::motor::CanFrame frame;
    frame.id = joint.feedback_id;
    frame.dlc = 8;
    frame.sequence = 1;
    frame.data[0] = 0x10;  // Enabled state, no error.
    frame.data[1] = 0x7f;
    frame.data[2] = 0xff;
    frame.data[3] = 0x7f;
    frame.data[4] = 0xf7;
    frame.data[5] = 0xff;
    port.Push(frame);
  }
  assert(arm.arm.UpdateFeedback() == DEVICE_OK);
  assert(arm.arm.Enable() == DEVICE_OK);
  port.sent.clear();
  assert(arm.arm.Step(config.controller.default_dt_s) == DEVICE_OK);
  assert(arm.arm.CommitOutput() == DEVICE_OK);
  assert(port.sent.size() == 3);
  for (unsigned i = 0; i < 3; ++i) assert(port.sent[i].id == app::params::kJoints[i].control_id);
  port.now = app::params::kMotorFeedbackTimeoutMs + 1;
  assert(arm.arm.UpdateFeedback() != DEVICE_OK);
  assert(arm.arm.lifecycle() == mr::module::arm::Lifecycle::kFault);
  assert(!arm.motor_0.HasPendingCommand());
}
