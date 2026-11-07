// Integration probe driven by tools.test_robot_generation's YAML fixture.
#include <cassert>
#include <cmath>
#include <cstring>
#include "robot_config.hpp"
#include "support/fake_can_port.hpp"

using mr::motor::CanFrame;
static bool Near(float a, float b, float eps = .002f) { return std::fabs(a - b) < eps; }
static uint16_t Encode(float x, float range, unsigned bits = 16) {
  return static_cast<uint16_t>((x + range) / (2 * range) * ((1u << bits) - 1));
}
static float Decode(uint16_t raw, float range, unsigned bits = 16) {
  return raw * 2 * range / ((1u << bits) - 1) - range;
}
static uint16_t Read16(const uint8_t* p) { return static_cast<uint16_t>((p[0] << 8) | p[1]); }
static void Put16(uint8_t* p, uint16_t x) { p[0] = x >> 8; p[1] = x; }

int main(int argc, char**) {
  const bool updated = argc > 1;
  FakeCanPort port;
  mr::generated::ChassisAssembly chassis(port);
  mr::generated::ArmAssembly arm(port);
  static_assert(mr::generated::ArmAssembly::kJointCount == 2);
  static_assert(decltype(arm.motor_0)::model() == mr::motor::MotorModel::J3507);
  static_assert(decltype(arm.motor_1)::model() == mr::motor::MotorModel::RSO3);
  assert(chassis.Register() == mr::module::chassis::ChassisStatus::kOk);
  assert(arm.Init() == DEVICE_OK);
  assert(port.sent.empty());
  assert(arm.arm.lifecycle() == mr::module::arm::Lifecycle::kReady);
  assert(arm.motor_0.GetInstallConfig().external_ratio == 2);
  assert(arm.motor_1.GetInstallConfig().external_ratio == 3);
  assert(Near(arm.joint_0.GetOffset(), .2f));
  assert(Near(arm.joint_1.GetOffset(), -.1f));
  assert(std::strcmp(arm.joint_0.GetName(), "shoulder") == 0);
  assert(std::strcmp(arm.joint_1.GetName(), "elbow") == 0);
  auto config = arm.Config();
  assert(Near(config.controller.default_dt_s, updated ? .004f : .002f));
  assert(Near(config.output.gains[0].kp, 40));
  assert(Near(config.output.gains[0].kd, 4));
  assert(Near(config.output.gains[0].effort_scale, .75f));
  assert(config.output.actuators[0] == &arm.joint_0);
  assert(config.model.Joint(0).position_offset == 0);  // zero offset belongs to JointT only.
  assert(Near(config.model.Joint(0).limits.position_min, -1));
  assert(Near(config.model.Joint(0).limits.position_max, 1));
  assert(Near(config.model.Joint(0).limits.max_velocity, 1.5f));
  assert(Near(config.model.Joint(0).limits.max_acceleration, 3));
  assert(Near(config.model.Joint(1).axis_in_joint_frame[1][0], 1));
  assert(Near(config.model.Joint(1).parent_to_joint(0, 3), .4f));
  assert(Near(config.model.Joint(1).parent_to_joint(1, 1), 0));
  assert(Near(config.model.Joint(1).parent_to_joint(2, 1), 1));
  assert(Near(config.model.Gravity()[2][0], -9.81f));
  const auto& base = arm.arm.model().BaseTransform();
  assert(Near(base(0, 3), updated ? .7f : .5f));
  assert(Near(base(0, 0), 0) && Near(base(1, 0), 1));
  assert(Near(arm.arm.model().ToolTransform()(2, 3), .25f));

  // Real DM feedback: reverse and external ratio 2, then subtract joint zero .2.
  CanFrame dm; dm.id = 0x321; dm.dlc = 8; dm.sequence = 1;
  dm.data[0] = 0x10;
  Put16(dm.data + 1, Encode(-1.2f, 6));
  const auto velocity = Encode(0, 8, 12), torque = Encode(-2, 12, 12);
  dm.data[3] = velocity >> 4;
  dm.data[4] = (velocity << 4) | (torque >> 8);
  dm.data[5] = torque;
  port.Push(dm);
  // LZ native output shaft units; ratio 3 is applied exactly once.
  CanFrame lz; lz.format = CanFrame::Format::Extended; lz.dlc = 8; lz.sequence = 1;
  lz.id = (2u << 24) | (22u << 8) | 0xfd;
  Put16(lz.data, Encode(.9f, 12.57f));
  Put16(lz.data + 2, Encode(0, 20));
  Put16(lz.data + 4, Encode(2, 60));
  Put16(lz.data + 6, 250);
  port.Push(lz);
  assert(arm.arm.UpdateFeedback() == DEVICE_OK);
  assert(Near(arm.arm.feedback().position[0][0], .4f));
  assert(Near(arm.arm.feedback().position[1][0], .4f));
  assert(Near(arm.arm.feedback().effort[0][0], 4, .02f));
  assert(Near(arm.arm.feedback().effort[1][0], 6, .02f));
  assert(arm.arm.Enable() == DEVICE_OK);
  assert(port.sent.size() == 2 && port.sent[0].id == 0x11);
  assert(port.sent[1].id == ((3u << 24) | (0xfdu << 8) | 22u));
  port.sent.clear();
  assert(arm.arm.Step(arm.kDtSeconds) == DEVICE_OK);
  assert(arm.arm.CommitOutput() == DEVICE_OK);
  assert(port.sent.size() == 2);
  assert(Near(arm.joint_0.GetMotorCommandPosition(), .6f));
  assert(Near(arm.joint_1.GetMotorCommandPosition(), .3f));
  assert(Near(Decode(Read16(port.sent[0].data), 6), -1.2f));
  assert(Near(Decode(Read16(port.sent[1].data), 12.57f), .9f));
  const auto& dm_command = port.sent[0];
  const float kp = (((dm_command.data[3] & 15) << 8) | dm_command.data[4]) * 500.f / 4095;
  assert(Near(kp, 10, .13f));  // 40 Nm/rad / external_ratio^2.
  // The generated motor timeout is 7 ms, not the hand-written default 20 ms.
  port.now = 8;
  assert(arm.arm.UpdateFeedback() != DEVICE_OK);
  assert(arm.arm.lifecycle() == mr::module::arm::Lifecycle::kFault);
  assert(!arm.motor_0.HasPendingCommand() && !arm.motor_1.HasPendingCommand());
}
