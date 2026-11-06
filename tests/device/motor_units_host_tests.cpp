#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>

#include "device/motor/bus/rm_command_bus.hpp"
#include "device/motor/motor.hpp"
#include "device/wheel/wheel.hpp"
#include "rm_chassis_cycle.hpp"
#include "support/fake_can_port.hpp"

using namespace mr::motor;
namespace {
bool Near(float a, float b, float eps = 0.02f) { return std::fabs(a - b) < eps; }
uint16_t Encode(float x, float range, unsigned bits) {
  return static_cast<uint16_t>((x + range) / (2 * range) * ((1UL << bits) - 1));
}
float Decode(uint16_t raw, float range, unsigned bits) {
  return raw * 2 * range / ((1UL << bits) - 1) - range;
}
uint16_t Read16(const uint8_t* p) { return static_cast<uint16_t>((p[0] << 8) | p[1]); }
void Put16(uint8_t* p, uint16_t x) {
  p[0] = x >> 8;
  p[1] = x;
}
CanFrame DmFeedback(uint32_t id, uint32_t seq = 1) {
  CanFrame f;
  f.id = id;
  f.sequence = seq;
  f.dlc = 8;
  f.data[0] = 0x11;
  Put16(f.data + 1, Encode(2, 6, 16));
  const auto v = Encode(4, 8, 12), t = Encode(3, 12, 12);
  f.data[3] = v >> 4;
  f.data[4] = (v << 4) | (t >> 8);
  f.data[5] = t;
  f.data[6] = 25;
  f.data[7] = 25;
  return f;
}
void TestDmUnits() {
  FakeCanPort port;
  MotorInstanceConfig<MotorKind::DM> c;
  c.port = &port;
  c.can_id = 1;
  c.master_id = 0x301;
  c.reverse = true;
  c.mit_position_range_rad = 6;
  c.mit_velocity_range_rad_s = 8;
  c.mit_torque_range_nm = 12;
  // J3507 has internal reducer=7 and non-unit torque constant. Neither applies
  // again to the native DM output-shaft wire units.
  DmJ3507Motor motor(c, {2});
  assert(motor.Register() == 0);
  port.Push(DmFeedback(c.master_id));
  assert(motor.Update() == 0);
  auto state = motor.GetState();
  assert(Near(state.position_rad, -1) && Near(state.velocity_rad_s, -2) &&
         Near(state.torque_nm, -6));
  assert(motor.SetMIT(1, 1.5f, 40, 4, 8) == 0 && motor.CommitCommand() == 0);
  const auto& f = port.sent.back();
  assert(Near(Decode(Read16(f.data), 6, 16), -2));
  assert(Near(Decode((f.data[2] << 4) | (f.data[3] >> 4), 8, 12), -3));
  const float kp = ((f.data[3] & 15) << 8 | f.data[4]) * 500.0f / 4095;
  const float kd = ((f.data[5] << 4) | (f.data[6] >> 4)) * 5.0f / 4095;
  assert(Near(kp, 10, 0.13f) && Near(kd, 1));
  assert(Near(Decode(((f.data[6] & 15) << 8) | f.data[7], 12, 12), -4));
  assert(motor.SetMIT(0, 0, -1, 0, 0) != 0 && !motor.HasPendingCommand());
  assert(motor.SetPosition(1, 2) == 0 && motor.CommitCommand() == 0);
  float position = 0, speed = 0;
  std::memcpy(&position, port.sent.back().data, 4);
  std::memcpy(&speed, port.sent.back().data + 4, 4);
  assert(Near(position, -2) && Near(speed, 4));  // Positive magnitude even reversed.
  assert(motor.SetTorque(2) == 0 && motor.SetZero() == 0 && !motor.HasPendingCommand());
  assert(motor.Update() != 0 && !motor.GetState().position_valid);
  auto fault = DmFeedback(c.master_id, 2);
  fault.data[0] = 0x81;
  port.Push(fault);
  assert(motor.Update() != 0 && motor.GetState().protocol_fault != 0);
  assert(motor.ClearFault() == 0 && port.sent.back().id == 1 && port.sent.back().data[7] == 0xfb);
  assert(motor.GetState().protocol_fault != 0);  // Wait for feedback to confirm.
  auto hot = DmFeedback(c.master_id, 3);
  hot.data[7] = 90;
  port.Push(hot);
  assert(motor.Update() != 0 && motor.IsTemperatureLimitActive());
  assert(motor.SetTorque(1) != 0 && !motor.HasPendingCommand());
  assert(port.sent.back().data[7] == 0xfd);  // Relax success is not control success.
  motor.DisableTemperatureProtection();
  assert(!motor.IsTemperatureLimitActive() && motor.SetTorque(1) == 0);
  port.now = 21;
  assert(motor.Update() != 0 && !motor.HasPendingCommand());
}
void TestLzUnits() {
  FakeCanPort port;
  MotorInstanceConfig<MotorKind::LZ> c;
  c.port = &port;
  c.motor_id = 3;
  c.host_id = 0xfd;
  c.reverse = true;
  LzRso3Motor motor(c, {2});
  assert(motor.Register() == 0);
  CanFrame f;
  f.format = CanFrame::Format::Extended;
  f.dlc = 8;
  f.sequence = 1;
  f.id = (2UL << 24) | (3UL << 8) | 0xfd;
  Put16(f.data, Encode(2, 12.57f, 16));
  Put16(f.data + 2, Encode(4, 20, 16));
  Put16(f.data + 4, Encode(3, 60, 16));
  Put16(f.data + 6, 250);
  port.Push(f);
  assert(motor.Update() == 0);
  const auto state = motor.GetState();
  assert(Near(state.position_rad, -1) && Near(state.velocity_rad_s, -2) &&
         Near(state.torque_nm, -6));
  assert(motor.SetMIT(1, 1.5f, 400, 40, 8) == 0 && motor.CommitCommand() == 0);
  const auto& out = port.sent.back();
  assert(Near(Decode(Read16(out.data), 12.57f, 16), -2));
  assert(Near(Decode(Read16(out.data + 2), 20, 16), -3));
  assert(Near(Read16(out.data + 4) * 5000.0f / 65535, 100, 0.1f));
  assert(Near(Read16(out.data + 6) * 100.0f / 65535, 10));
  assert(Near(Decode(static_cast<uint16_t>(out.id >> 8), 60, 16), -4));
  assert(motor.SetPosition(1, 2) == DEVICE_ERR_UNSUPPORTED && !motor.HasPendingCommand());
  assert(motor.SetPosition(1) == 0 && motor.CommitCommand() == 0);
  assert(Near(Decode(Read16(port.sent.back().data + 2), 20, 16), 0));
  f.id |= 1UL << 16;
  f.sequence = 2;
  port.Push(f);
  assert(motor.Update() != 0 && motor.GetState().protocol_fault != 0);
  assert(motor.ClearFault() == 0 && !motor.HasPendingCommand());
  assert((port.sent.back().id >> 24) == 4 && port.sent.back().data[0] == 1);
  assert(motor.GetState().protocol_fault != 0);
}
void TestOwnership() {
  FakeCanPort port;
  MotorInstanceConfig<MotorKind::DM> c;
  c.port = &port;
  c.can_id = 1;
  c.master_id = 0x301;
  {
    DmJ4310Motor first(c);
    assert(first.Register() == 0);
    DmJ4310Motor duplicate(c);
    assert(duplicate.Register() != 0);
    c.can_id = 2;
    DmJ4310Motor duplicate_rx(c);
    assert(duplicate_rx.Register() != 0);
    c.can_id = 1;
    c.master_id = 0x302;
    DmJ4310Motor duplicate_tx(c);
    assert(duplicate_tx.Register() != 0);
  }
  c.can_id = 1;
  c.master_id = 0x301;
  DmJ4310Motor replacement(c);
  assert(replacement.Register() == 0);
  MotorInstanceConfig<MotorKind::LZ> l;
  l.port = &port;
  l.motor_id = 3;
  LzRso3Motor lz(l);
  assert(lz.Register() == 0);
  l.host_id = 0xfe;
  LzRso3Motor same_target(l);
  assert(same_target.Register() != 0);
  RmCommandBus bus(port), second_bus(port);
  MotorInstanceConfig<MotorKind::RM> r;
  r.port = &port;
  r.commands = &bus;
  r.id = 0x201;
  RmM3508Motor rm(r);  // DM can_id=1 reserves 0x201 for its velocity mode.
  assert(rm.Register() != 0);
  r.id = 0x205;
  RmM3508Motor other_group(r);
  assert(other_group.Register() == 0);
  r.id = 0x206;
  r.commands = &second_bus;
  RmM3508Motor duplicate_group(r);
  assert(duplicate_group.Register() != 0);
  FakeCanPort other_port;
  c.port = &other_port;
  DmJ4310Motor same_address_other_bus(c);
  assert(same_address_other_bus.Register() == 0);
  c.can_id = 2;
  c.mit_position_range_rad = std::numeric_limits<float>::quiet_NaN();
  DmJ4310Motor invalid(c);
  assert(invalid.Register() != 0);
}
void TestRegistrationRollbackAndCapacity() {
  FakeCanPort port;
  MotorInstanceConfig<MotorKind::DM> c;
  c.port = &port;
  c.can_id = 1;
  c.master_id = 0x301;
  DmJ4310Motor failed(c);
  port.fail_registration = true;
  assert(failed.Register() != 0);
  port.fail_registration = false;
  DmJ4310Motor succeeding(c);
  assert(succeeding.Register() == 0);  // Failed subscription released all claims.
  FakeCanPort capacity;
  int owner_a = 0, owner_b = 0;
  uint32_t keys[63];
  for (uint32_t i = 0; i < 63; ++i) keys[i] = i;
  assert(capacity.ClaimEndpoints(&owner_a, keys, 63));
  const uint32_t conflict[] = {63, 0};
  assert(!capacity.ClaimEndpoints(&owner_b, conflict, 2));
  uint32_t key = 63;
  assert(capacity.ClaimEndpoints(&owner_b, &key, 1));
  key = 64;
  assert(!capacity.ClaimEndpoints(&owner_b, &key, 1));
  capacity.ReleaseEndpoints(&owner_b);
  assert(capacity.ClaimEndpoints(&owner_b, &key, 1));
}
void TestChassisExecution() {
  using namespace mr::module::chassis;
  FakeCanPort port;
  RmCommandBus bus(port);
  MotorInstanceConfig<MotorKind::RM> c;
  c.port = &port;
  c.commands = &bus;
  c.id = 0x201;
  RmM3508Motor m0(c);
  c.id = 0x202;
  RmM3508Motor m1(c);
  c.id = 0x203;
  RmM3508Motor m2(c);
  c.id = 0x204;
  RmM3508Motor m3(c);
  mr::wheel::MotorWheel<RmM3508Motor> w0(m0, 0.1f), w1(m1, 0.1f), w2(m2, 0.1f), w3(m3, 0.1f);
  Chassis chassis;
  ChassisConfig cfg;
  cfg.kinematics.model = mr::robotics::chassis::ChassisModel::kMecanum;
  cfg.kinematics.geometry = {0.4f, 0.4f};
  cfg.max_body_velocity = {2, 2, 2};
  cfg.max_wheel_speed_mps = 3;
  cfg.wheel_velocity_pid.gains = {1, 1, 0, 0};
  cfg.wheel_velocity_pid.output_limit = mr::comp::cntlr::SymmetricLimit::Abs(2);
  cfg.wheel_velocity_pid.control_freq_hz = 1000;
  assert(chassis.Register(cfg, {&w0, &w1, &w2, &w3}) == ChassisStatus::kOk);
  assert(chassis.Enable() == ChassisStatus::kOk);
  for (uint32_t id = 0x201; id <= 0x204; ++id) {
    CanFrame f;
    f.id = id;
    f.dlc = 8;
    f.sequence = 1;
    port.Push(f);
  }
  assert(example::RunChassisCycle(chassis, bus, port.now, {1, 0, 0}, 0.001f).ok());
  assert(port.sent.size() == 1 && port.sent.back().id == 0x200);
  assert(port.sent.back().data[0] != 0 || port.sent.back().data[1] != 0);
  port.fail_send = true;
  const auto failed = example::RunChassisCycle(chassis, bus, port.now, {1, 0, 0}, 0.001f);
  assert(!failed.ok() && failed.flush != 0 && failed.stop_flush == 0);
  for (auto byte : port.sent.back().data) assert(byte == 0);
  assert(chassis.Enable() == ChassisStatus::kOk);
  port.now = 21;
  const auto expired = example::RunChassisCycle(chassis, bus, port.now, {1, 0, 0}, 0.001f);
  assert(!expired.ok() && expired.flush == 0);
  for (auto byte : port.sent.back().data) assert(byte == 0);
}
}  // namespace
int main() {
  TestDmUnits();
  TestLzUnits();
  TestOwnership();
  TestRegistrationRollbackAndCapacity();
  TestChassisExecution();
}
