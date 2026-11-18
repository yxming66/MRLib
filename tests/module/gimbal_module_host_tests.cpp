#include <cassert>
#include <cmath>
#include <limits>
#include <type_traits>

#include "module/gimbal/gimbal.hpp"

namespace {
using namespace mr::module::gimbal;
bool Near(float a, float b) { return std::fabs(a - b) < 1.0e-5f; }

struct FakeActuator final : mr::IActuator {
  int8_t register_result = DEVICE_OK, enable_result = DEVICE_OK;
  int8_t update_result = DEVICE_OK, set_result = DEVICE_OK, commit_result = DEVICE_OK;
  int registered = 0, enabled = 0, relaxed = 0, commits = 0, writes = 0;
  bool pending = false;
  mr::ActuatorState state{0, 0, 0, true};
  float target = 0, kp = 0, kd = 0;
  int8_t Register() override { ++registered; return register_result; }
  int8_t Enable() override { ++enabled; return enable_result; }
  int8_t Update() override { return update_result; }
  int8_t Relax() override { ++relaxed; pending = false; return DEVICE_OK; }
  int8_t SetZero() override { return DEVICE_OK; }
  int8_t CommitControl() override { ++commits; pending = false; return commit_result; }
  bool HasPendingControl() const override { return pending; }
  void ClearPendingControl() override { pending = false; }
  int8_t SetMIT(float position, float, float p, float d, float) override {
    ++writes; target = position; kp = p; kd = d; pending = true; return set_result;
  }
  int8_t SetTorque(float) override { return DEVICE_ERR_UNSUPPORTED; }
  mr::ActuatorState GetState() const override { return state; }
};

YawPitchGimbal::Config TwoConfig(FakeActuator& yaw, FakeActuator& pitch) {
  YawPitchGimbal::Config config{};
  config.axes = {&yaw, &pitch};
  config.gains = {{{20.0f, 1.0f}, {16.0f, 0.8f}}};
  config.limits[1] = {-0.5f, 0.75f};
  return config;
}

YawPitchPitchGimbal::Config ThreeConfig(FakeActuator& yaw, FakeActuator& p1,
                                       FakeActuator& p2) {
  YawPitchPitchGimbal::Config config{};
  config.axes = {&yaw, &p1, &p2};
  config.gains = {{{20.0f, 1.0f}, {16.0f, 0.8f}, {12.0f, 0.6f}}};
  config.limits[1] = {0.0f, 1.5707963f};
  config.limits[2] = {-0.8f, 1.5707963f};
  config.low_fire_pitch1_rad = 0.4f;
  config.high_fire_pitch1_rad = 1.1f;
  return config;
}

void TestTwoAxis() {
  FakeActuator yaw, pitch;
  yaw.state.position = 0.2f;
  pitch.state.position = 0.3f;
  YawPitchGimbal gimbal;
  assert(gimbal.SetTarget({0, 0}) == Status::kInvalidState);
  assert(gimbal.SetMode(YawPitchGimbal::Mode::kNormal) == Status::kInvalidState);
  assert(gimbal.Update(0.002f) == Status::kInvalidState);
  const auto config = TwoConfig(yaw, pitch);
  assert(gimbal.Register(config) == Status::kOk);
  assert(gimbal.Register(config) == Status::kInvalidState);
  assert(yaw.enabled == 0 && yaw.commits == 0);
  assert(gimbal.Enable() == Status::kOk);
  assert(gimbal.Update(0.002f) == Status::kOk);
  assert(Near(yaw.target, 0.2f) && Near(pitch.target, 0.3f));
  assert(gimbal.SetTarget({1.0f, 0.5f}) == Status::kOk);
  assert(gimbal.Update(0.002f) == Status::kOk);
  assert(Near(yaw.target, 1.0f) && Near(pitch.target, 0.5f));
  assert(Near(yaw.kp, 20.0f) && Near(pitch.kd, 0.8f));
  assert(yaw.commits == 2 && pitch.commits == 2 && !yaw.pending && !pitch.pending);
  assert(gimbal.SetTarget({4.0f, 0.0f}) == Status::kOk);
  assert(gimbal.Update(0.002f) == Status::kOk);
  assert(gimbal.lifecycle() == Lifecycle::kEnabled);
  assert(Near(yaw.target, config.limits[0].max_rad));
  assert(gimbal.SetTarget({0.0f, std::numeric_limits<float>::quiet_NaN()}) ==
         Status::kInvalidArgument);
  assert(gimbal.SetMode(static_cast<YawPitchGimbal::Mode>(99)) == Status::kInvalidArgument);
  assert(Near(gimbal.target().yaw_rad, config.limits[0].max_rad));
  assert(gimbal.Disable() == Status::kOk);
  assert(gimbal.Update(0.002f) == Status::kInvalidState);
  assert(yaw.relaxed > 0 && pitch.relaxed > 0);
  assert(gimbal.Enable() == Status::kOk);
  assert(gimbal.Update(0.002f) == Status::kOk);
  assert(Near(yaw.target, yaw.state.position));  // Re-enable captures measured hold target.
  assert(gimbal.Update(0.0f) == Status::kInvalidArgument);
  assert(gimbal.lifecycle() == Lifecycle::kFault);
}

void TestThreeAxisModes() {
  FakeActuator yaw, p1, p2;
  YawPitchPitchGimbal gimbal;
  const auto config = ThreeConfig(yaw, p1, p2);
  assert(gimbal.Register(config) == Status::kOk);
  assert(gimbal.Enable() == Status::kOk);
  auto follow = [&]() {
    yaw.state.position = yaw.target;
    p1.state.position = p1.target;
    p2.state.position = p2.target;
  };
  auto tick = [&]() { follow(); assert(gimbal.Update(0.01f) == Status::kOk); };
  auto finish = [&](YawPitchPitchGimbal::Stage stage) {
    for (int i = 0; i < 1000 && gimbal.stage() != stage; ++i) tick();
    assert(gimbal.stage() == stage);
  };
  finish(YawPitchPitchGimbal::Stage::kFolded);
  assert(Near(p1.target, 0) && Near(p2.target, 0) && Near(yaw.target, 0));
  // Folded mode ignores all input motion targets, but keeps the motors enabled.
  assert(gimbal.SetTarget({0.3f, 0.7f, -0.2f}) == Status::kOk);
  tick();
  assert(Near(yaw.target, 0) && Near(p1.target, 0) && Near(p2.target, 0));
  assert(gimbal.SetMode(YawPitchPitchGimbal::Mode::kLowFire) == Status::kOk);
  assert(!gimbal.pitch2_control_ready());
  for (int i = 0; i < 1000 && !gimbal.pitch2_control_ready(); ++i) {
    // Even a constantly changing pitch2 target must not bypass leveling.
    assert(gimbal.SetTarget({0.3f, 0.7f, -0.2f}) == Status::kOk);
    const float old = p1.target;
    follow();
    const float measured_p1 = p1.state.position;
    assert(gimbal.Update(0.01f) == Status::kOk);
    assert(std::fabs(p1.target - old) <= config.posture_speed_rad_s * 0.01f + 1e-5f);
    if (!gimbal.pitch2_control_ready())
      assert(std::fabs(p2.target - measured_p1) < 0.007f);
  }
  assert(gimbal.pitch2_control_ready());
  assert(Near(p1.target, config.low_fire_pitch1_rad));
  assert(Near(p2.target, config.low_fire_pitch1_rad)); // No old target replay.
  assert(gimbal.SetTarget({0.3f, 0.7f, 0.1f}) == Status::kOk);
  tick();
  assert(Near(p2.target, 0.1f) && Near(p1.target, config.low_fire_pitch1_rad));
  // High fire starts by leveling at the old posture before moving pitch1.
  follow();
  assert(gimbal.SetMode(YawPitchPitchGimbal::Mode::kHighFire) == Status::kOk);
  assert(!gimbal.pitch2_control_ready());
  tick();
  assert(gimbal.stage() == YawPitchPitchGimbal::Stage::kLeveling);
  assert(Near(p1.target, config.low_fire_pitch1_rad));
  assert(p2.target > 0.1f && p2.target < config.low_fire_pitch1_rad);
  finish(YawPitchPitchGimbal::Stage::kWorking);
  assert(Near(p1.target, config.high_fire_pitch1_rad));
  assert(Near(p2.target, config.high_fire_pitch1_rad));
  // Mode changes mid-motion restart from feedback, not old trajectory state.
  assert(gimbal.SetMode(YawPitchPitchGimbal::Mode::kLowFire) == Status::kOk);
  for (int i = 0; i < 30; ++i) tick();
  assert(gimbal.SetMode(YawPitchPitchGimbal::Mode::kFolded) == Status::kOk);
  finish(YawPitchPitchGimbal::Stage::kFolded);
  assert(Near(p1.target, 0) && Near(p2.target, 0) && Near(yaw.target, 0));
  assert(gimbal.SetMode(static_cast<YawPitchPitchGimbal::Mode>(99)) == Status::kInvalidArgument);
  assert(gimbal.Disable() == Status::kOk);
  assert(gimbal.stage() == YawPitchPitchGimbal::Stage::kInactive);
}

void TestFoldYawAndLevelThenPitchZero() {
  FakeActuator yaw, p1, p2;
  yaw.state.position = 0.3f;
  p1.state.position = 0.4f;
  p2.state.position = 0.1f;
  YawPitchPitchGimbal gimbal;
  const auto config = ThreeConfig(yaw, p1, p2);
  assert(gimbal.Register(config) == Status::kOk);
  assert(gimbal.Enable() == Status::kOk);
  assert(gimbal.Update(0.01f) == Status::kOk);
  assert(yaw.target < 0.3f && p2.target > 0.1f); // Parallel yaw and leveling.
  assert(Near(p1.target, 0.4f));
  // A zero yaw command alone cannot release pitch1 while measured yaw is off zero.
  for (int i = 0; i < 100; ++i) {
    p2.state.position = p2.target;
    assert(gimbal.Update(0.01f) == Status::kOk);
  }
  assert(gimbal.stage() == YawPitchPitchGimbal::Stage::kFoldYaw);
  assert(Near(p1.target, 0.4f));
  yaw.state.position = 0.03f; // Residual error accepted by the relaxed fold threshold.
  for (int i = 0; i < 20; ++i) {
    p2.state.position = p2.target;
    assert(gimbal.Update(0.01f) == Status::kOk);
  }
  assert(gimbal.stage() == YawPitchPitchGimbal::Stage::kFoldPitch);
  assert(p1.target < 0.4f);
  // Head error must not pause either joint; both targets move at the same speed.
  p1.state.position = p1.target;
  p2.state.position = p1.state.position - 0.2f;
  const float previous_p1 = p1.target;
  const float previous_p2 = p2.target;
  assert(gimbal.Update(0.01f) == Status::kOk);
  assert(Near(previous_p1 - p1.target, config.posture_speed_rad_s * 0.01f));
  assert(Near(previous_p2 - p2.target, config.posture_speed_rad_s * 0.01f));
  for (int i = 0; i < 1000 && gimbal.stage() != YawPitchPitchGimbal::Stage::kFolded; ++i) {
    // A persistent 0.03 rad pitch offset should not prevent completion.
    p1.state.position = p1.target + 0.03f;
    p2.state.position = p2.target + 0.03f;
    assert(gimbal.Update(0.01f) == Status::kOk);
  }
  assert(gimbal.stage() == YawPitchPitchGimbal::Stage::kFolded);
  assert(Near(yaw.target, 0) && Near(p1.target, 0) && Near(p2.target, 0));
}

void TestTransitionRequiresFeedbackAndSettling() {
  FakeActuator yaw, p1, p2;
  YawPitchPitchGimbal gimbal;
  auto config = ThreeConfig(yaw, p1, p2);
  assert(gimbal.Register(config) == Status::kOk);
  assert(gimbal.Enable() == Status::kOk);
  assert(gimbal.SetMode(YawPitchPitchGimbal::Mode::kHighFire) == Status::kOk);
  for (int i = 0; i < 49; ++i) assert(gimbal.Update(0.01f) == Status::kOk);
  assert(!gimbal.pitch2_control_ready());
  // With stationary feedback, the ramp must build error beyond arrival tolerance
  // to develop enough MIT torque to overcome a load.
  assert(p1.target > config.position_tolerance_rad);
  assert(p1.target <= config.posture_speed_rad_s * 0.49f);
  const float old_p1 = p1.target, old_p2 = p2.target;
  p2.state.position = -0.2f; // Lose level mid-motion without blocking either ramp.
  assert(gimbal.Update(0.01f) == Status::kOk);
  assert(Near(p1.target - old_p1, config.posture_speed_rad_s * 0.01f));
  assert(Near(p2.target - old_p2, config.posture_speed_rad_s * 0.01f));
  // Stalled but valid feedback must remain enabled beyond the former 10 s timeout.
  for (int i = 0; i < 1200; ++i) assert(gimbal.Update(0.01f) == Status::kOk);
  assert(gimbal.lifecycle() == Lifecycle::kEnabled);
  assert(!gimbal.pitch2_control_ready());
  assert(p1.relaxed == 0 && p2.relaxed == 0);
  assert(Near(p1.target, config.high_fire_pitch1_rad));
  assert(Near(p2.target, config.high_fire_pitch1_rad));

  FakeActuator y, a, b;
  YawPitchPitchGimbal settling;
  assert(settling.Register(ThreeConfig(y, a, b)) == Status::kOk);
  assert(settling.Enable() == Status::kOk);
  assert(settling.SetMode(YawPitchPitchGimbal::Mode::kLowFire) == Status::kOk);
  for (int i = 0; i < 200 && settling.stage() != YawPitchPitchGimbal::Stage::kSettling; ++i) {
    a.state.position = a.target; b.state.position = b.target;
    assert(settling.Update(0.01f) == Status::kOk);
  }
  assert(settling.stage() == YawPitchPitchGimbal::Stage::kSettling);
  a.state.position = a.target; b.state.position = b.target;
  a.state.position = a.target + 0.2f;
  a.state.velocity = 1.0f; // Velocity no longer participates in arrival checks.
  for (int i = 0; i < 30; ++i) assert(settling.Update(0.01f) == Status::kOk);
  assert(!settling.pitch2_control_ready());
  for (int i = 0; i < 12; ++i) {
    a.state.position = a.target + 0.04f; b.state.position = b.target - 0.04f;
    assert(settling.Update(0.01f) == Status::kOk);
  }
  assert(settling.pitch2_control_ready());
}
void TestConfiguration() {
  FakeActuator yaw, pitch, p2;
  YawPitchGimbal gimbal;
  auto config = TwoConfig(yaw, pitch);
  config.axes[1] = &yaw;
  assert(gimbal.Register(config) == Status::kInvalidConfiguration);
  assert(yaw.registered == 0);
  config = TwoConfig(yaw, pitch);
  config.limits[1] = {1, 0};
  assert(gimbal.Register(config) == Status::kInvalidConfiguration);
  config.limits[1] = {0, std::numeric_limits<float>::infinity()};
  assert(gimbal.Register(config) == Status::kInvalidConfiguration);
  config.axes[1] = nullptr;
  assert(gimbal.Register(config) == Status::kInvalidConfiguration);

  YawPitchPitchGimbal triple;
  auto three = ThreeConfig(yaw, pitch, p2);
  three.high_fire_pitch1_rad = 2.0f;
  assert(triple.Register(three) == Status::kInvalidConfiguration);
  three.high_fire_pitch1_rad = std::numeric_limits<float>::quiet_NaN();
  assert(triple.Register(three) == Status::kInvalidConfiguration);
  assert(yaw.registered == 0);

  three = ThreeConfig(yaw, pitch, p2);
  three.limits[2].max_rad = 0.8f; // High-fire compensation cannot fit.
  assert(triple.Register(three) == Status::kInvalidConfiguration);
  assert(yaw.registered == 0);
}

void TestFaults() {
  FakeActuator a, b;
  b.register_result = -7;
  YawPitchGimbal partial;
  assert(partial.Register(TwoConfig(a, b)) == Status::kDeviceError);
  assert(a.relaxed == 1 && b.relaxed == 1);
  assert(partial.Disable() == Status::kInvalidState);
  assert(partial.Enable() == Status::kInvalidState);

  FakeActuator p0, p1, p2;
  p0.register_result = -8;
  YawPitchPitchGimbal incomplete;
  assert(incomplete.Register(ThreeConfig(p0, p1, p2)) == Status::kDeviceError);
  assert(incomplete.Disable() == Status::kInvalidState);
  assert(incomplete.Enable() == Status::kInvalidState);
  assert(incomplete.lifecycle() == Lifecycle::kFault && p1.enabled == 0);

  FakeActuator c, d;
  YawPitchGimbal offline;
  assert(offline.Register(TwoConfig(c, d)) == Status::kOk);
  d.state.online = false;
  assert(offline.Enable() == Status::kInvalidFeedback);
  assert(c.enabled == 0 && offline.lifecycle() == Lifecycle::kFault);

  FakeActuator x, y;
  YawPitchGimbal output;
  assert(output.Register(TwoConfig(x, y)) == Status::kOk);
  assert(output.Enable() == Status::kOk);
  y.set_result = -9;
  assert(output.Update(0.002f) == Status::kDeviceError);
  assert(output.lifecycle() == Lifecycle::kFault && !x.pending && !y.pending);
  assert(x.commits == 0 && y.commits == 0);
  assert(output.last_device_error() == -9 && output.last_relax_error() == DEVICE_OK);

  FakeActuator u, v;
  YawPitchGimbal commit;
  assert(commit.Register(TwoConfig(u, v)) == Status::kOk);
  assert(commit.Enable() == Status::kOk);
  v.commit_result = -10;
  assert(commit.Update(0.002f) == Status::kDeviceError);
  assert(commit.lifecycle() == Lifecycle::kFault && !u.pending && !v.pending);
  assert(u.relaxed > 0 && v.relaxed > 0 && commit.last_device_error() == -10);
}
}

int main() {
  static_assert(!std::is_copy_constructible_v<mr::module::gimbal::YawPitchGimbal>);
  static_assert(!std::is_copy_constructible_v<mr::module::gimbal::YawPitchPitchGimbal>);
  TestTwoAxis();
  TestThreeAxisModes();
  TestFoldYawAndLevelThenPitchZero();
  TestTransitionRequiresFeedbackAndSettling();
  TestConfiguration();
  TestFaults();
}
