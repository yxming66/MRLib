#include <cassert>
#include <cmath>
#include <cstdio>

#include "module/shoot/shoot.hpp"
#include "device/motor/core/motor_aliases.hpp"

namespace {
using namespace mr::module::shoot;

class FakeMotor final : public MotorDevice {
 public:
  int8_t Register() override { ++register_count; return register_result; }
  int8_t Enable() override { ++enable_count; return DEVICE_OK; }
  int8_t Update() override { return update_result; }
  mr::motor::MotorState State() const override { return state; }
  int8_t SetTorque(float value) override { torque_command = value; return set_result; }
  int8_t CommitCommand() override { ++commit_count; return commit_result; }
  void ClearPendingCommand() override { ++clear_count; }
  int8_t Relax() override { ++relax_count; return DEVICE_OK; }
  float PeakTorqueNm() const override { return peak_torque; }
  float MaxVelocityRadS() const override { return max_velocity; }
  float CurrentA() const override { return current_a; }

  void SetOnline(uint32_t timestamp) {
    state.online = true;
    state.feedback_valid = true;
    state.position_valid = true;
    state.last_feedback_tick = timestamp;
  }

  mr::motor::MotorState state{};
  int8_t register_result = DEVICE_OK, update_result = DEVICE_OK;
  int8_t set_result = DEVICE_OK, commit_result = DEVICE_OK;
  float peak_torque = 6.0f, max_velocity = 50.0f, current_a = 0.0f;
  float torque_command = 0.0f;
  unsigned register_count = 0, enable_count = 0, commit_count = 0;
  unsigned clear_count = 0, relax_count = 0;
};

MrPidConfig Pid(float p, float output_limit = 0.0f) {
  MrPidConfig config = MrPid_DefaultConfig();
  config.gains = {1.0f, p, 0.0f, 0.0f};
  config.output_limit = output_limit;
  config.control_freq_hz = 1000.0f;
  return config;
}

Config MakeConfig() {
  Config c{};
  c.friction_count = 2;
  c.friction_level = {1, 1};
  c.level_velocity_ratio = {1.0f, 1.0f};
  c.friction_target_velocity_rad_s = 100.0f;
  c.friction_pid = Pid(0.1f);
  c.trigger_teeth_per_revolution = 8;
  c.trigger_max_velocity_rad_s = 10.0f;
  c.trigger_position_pid = Pid(1.0f);
  c.trigger_velocity_pid = Pid(0.1f);
  c.shot_frequency_hz = 3.0f;
  c.burst_shots = 3;
  c.feedback_timeout_ms = 20;
  return c;
}

Devices MakeDevices(FakeMotor& left, FakeMotor& right, FakeMotor& trigger) {
  Devices d{};
  d.friction[0] = &left;
  d.friction[1] = &right;
  d.trigger = &trigger;
  return d;
}

void Feed(Shoot& shoot, FakeMotor& a, FakeMotor& b, FakeMotor& t, uint32_t now) {
  a.SetOnline(now);
  b.SetOnline(now);
  t.SetOnline(now);
  assert(shoot.UpdateFeedback(now) == Status::kOk);
}

void TestRegistrationAndSafeOutput() {
  FakeMotor a, b, t;
  Shoot shoot;
  auto config = MakeConfig();
  a.SetOnline(100); b.SetOnline(100); t.SetOnline(100);
  assert(shoot.Register(config, MakeDevices(a, b, t)) == Status::kOk);
  assert(a.register_count == 1 && b.register_count == 1 && t.register_count == 1);
  assert(shoot.Enable(100) == Status::kOk);
  Feed(shoot, a, b, t, 100);
  assert(shoot.Update(0.002f, 102, {Mode::kSafe, false, false}) == Status::kOk);
  assert(a.torque_command == 0 && b.torque_command == 0 && t.torque_command == 0);
}

void TestInvalidMotorLimitsRejected() {
  FakeMotor a, b, t;
  Shoot shoot;
  auto config = MakeConfig();
  auto devices = MakeDevices(a, b, t);
  a.peak_torque = NAN;
  assert(shoot.Register(config, devices) == Status::kInvalidArgument);

  FakeMotor c, d, trigger;
  Shoot second;
  devices = MakeDevices(c, d, trigger);
  trigger.max_velocity = INFINITY;
  assert(second.Register(config, devices) == Status::kInvalidArgument);
}

void TestFeedStepAndHeatFallback() {
  FakeMotor a, b, t;
  Shoot shoot;
  auto config = MakeConfig();
  config.heat.enabled = true;
  a.SetOnline(100); b.SetOnline(100); t.SetOnline(100);
  assert(shoot.Register(config, MakeDevices(a, b, t)) == Status::kOk);
  assert(shoot.Enable(100) == Status::kOk);
  Feed(shoot, a, b, t, 100);
  assert(shoot.SetHeatData({false, 100, 10, 10, 100}) == Status::kOk);
  assert(shoot.Update(0.002f, 102, {Mode::kContinuous, true, true}) == Status::kOk);
  assert(shoot.debug().actual_frequency_hz == 3.0f);
  assert(shoot.debug().running_state == RunningState::kFiring);
  assert(shoot.debug().shots_remaining == UINT16_MAX);
  for (uint32_t ms = 104; ms <= 440; ms += 2) {
    Feed(shoot, a, b, t, ms);
    assert(shoot.Update(0.002f, ms, {Mode::kContinuous, true, true}) == Status::kOk);
  }
  assert(std::fabs(shoot.debug().trigger_target_position_rad - 6.28318530718f / 8.0f) < 1e-5f);
  assert(shoot.debug().shots_remaining == UINT16_MAX);

  assert(shoot.SetHeatData({true, 100, 0, 10, 100}) == Status::kOk);
  Feed(shoot, a, b, t, 442);
  assert(shoot.Update(0.002f, 442, {Mode::kContinuous, true, true}) == Status::kOk);
  assert(shoot.debug().actual_frequency_hz == 0.0f);
}

void TestStaleFeedbackAndDeviceFailure() {
  FakeMotor a, b, t;
  Shoot shoot;
  a.SetOnline(100); b.SetOnline(100); t.SetOnline(100);
  assert(shoot.Register(MakeConfig(), MakeDevices(a, b, t)) == Status::kOk);
  assert(shoot.Enable(100) == Status::kOk);
  Feed(shoot, a, b, t, 100);
  assert(shoot.Update(0.002f, 121, {Mode::kSingle, true, false}) == Status::kStaleFeedback);
  Feed(shoot, a, b, t, 122);
  b.commit_result = DEVICE_ERR;
  assert(shoot.Update(0.002f, 122, {Mode::kSingle, true, false}) == Status::kDeviceError);
  assert(shoot.debug().running_state == RunningState::kIdle);
  assert(a.clear_count > 0 && t.clear_count > 0);
}

void TestSingleShot() {
  FakeMotor a, b, t;
  Shoot shoot;
  auto config = MakeConfig();
  config.shot_frequency_hz = 100.0f;
  config.trigger_teeth_per_revolution = 10;
  a.SetOnline(100); b.SetOnline(100); t.SetOnline(100);
  assert(shoot.Register(config, MakeDevices(a, b, t)) == Status::kOk);
  assert(shoot.Enable(100) == Status::kOk);
  Feed(shoot, a, b, t, 102);
  assert(shoot.Update(0.002f, 102, {Mode::kSingle, true, false}) == Status::kOk);
  Feed(shoot, a, b, t, 104);
  assert(shoot.Update(0.002f, 104, {Mode::kSingle, true, true}) == Status::kOk);
  assert(shoot.debug().shots_remaining == 1);
  Feed(shoot, a, b, t, 114);
  assert(shoot.Update(0.010f, 114, {Mode::kSingle, true, true}) == Status::kOk);
  assert(shoot.debug().shots_remaining == 0);
  assert(shoot.debug().running_state == RunningState::kReady);
  const float one_shot_target = 6.28318530718f / 10.0f;
  assert(std::fabs(shoot.debug().trigger_target_position_rad - one_shot_target) < 1.0e-5f);
  Feed(shoot, a, b, t, 124);
  assert(shoot.Update(0.010f, 124, {Mode::kSingle, true, true}) == Status::kOk);
  assert(std::fabs(shoot.debug().trigger_target_position_rad - one_shot_target) < 1.0e-5f);
}

void TestJamRecovery() {
  FakeMotor a, b, t;
  Shoot shoot;
  auto config = MakeConfig();
  config.trigger_teeth_per_revolution = 10;
  config.jam_detection_enabled = true;
  config.jam_current_threshold_a = 2.0f;
  config.jam_suspected_time_s = 0.004f;
  config.jam_recovery_time_s = 0.006f;
  config.jam_recovery_current_a = 0.1f;
  config.shot_frequency_hz = 1.0f;
  a.SetOnline(100); b.SetOnline(100); t.SetOnline(100);
  assert(shoot.Register(config, MakeDevices(a, b, t)) == Status::kOk);
  assert(shoot.Enable(100) == Status::kOk);
  Feed(shoot, a, b, t, 102);
  assert(shoot.Update(0.002f, 102, {Mode::kContinuous, true, false}) == Status::kOk);
  Feed(shoot, a, b, t, 104);
  t.current_a = 3.0f;
  assert(shoot.Update(0.002f, 104, {Mode::kContinuous, true, true}) == Status::kOk);
  Feed(shoot, a, b, t, 106);
  assert(shoot.Update(0.002f, 106, {Mode::kContinuous, true, true}) == Status::kOk);
  Feed(shoot, a, b, t, 108);
  assert(shoot.Update(0.002f, 108, {Mode::kContinuous, true, true}) == Status::kOk);
  Feed(shoot, a, b, t, 110);
  assert(shoot.Update(0.002f, 110, {Mode::kContinuous, true, true}) == Status::kOk);
  assert(shoot.debug().jam_state == JamState::kRecovering);
  const float reverse_target = shoot.debug().trigger_target_position_rad;
  assert(reverse_target < 0.0f);
  Feed(shoot, a, b, t, 112);
  assert(shoot.Update(0.002f, 112, {Mode::kContinuous, true, true}) == Status::kOk);
  assert(shoot.debug().trigger_target_position_rad == reverse_target);
  t.current_a = 0.0f;
  Feed(shoot, a, b, t, 120);
  assert(shoot.Update(0.008f, 120, {Mode::kContinuous, true, true}) == Status::kOk);
  assert(shoot.debug().jam_state == JamState::kNormal);
}

void TestMotorAdapterUsesConfiguredOutputAxis() {
  mr::motor::MotorInstanceConfig<mr::motor::MotorKind::RM> config{};
  mr::motor::RmM3508Motor motor(config, {2.0f});
  MotorDeviceAdapter<mr::motor::RmM3508Motor> adapter(motor);
  assert(std::fabs(adapter.PeakTorqueNm() - 12.0f) < 1.0e-5f);
  assert(std::fabs(adapter.MaxVelocityRadS() - 49.11f / 2.0f) < 1.0e-5f);
}
}  // namespace

int main() {
  TestRegistrationAndSafeOutput();
  TestInvalidMotorLimitsRejected();
  TestFeedStepAndHeatFallback();
  TestStaleFeedbackAndDeviceFailure();
  TestSingleShot();
  TestJamRecovery();
  TestMotorAdapterUsesConfiguredOutputAxis();
  std::puts("Shoot module host tests passed");
  return 0;
}
