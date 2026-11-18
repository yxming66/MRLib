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
  config.limits[2] = {-0.8f, 0.8f};
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
  assert(gimbal.SetTarget({4.0f, 0.0f}) == Status::kInvalidArgument);
  assert(gimbal.SetTarget({0.0f, std::numeric_limits<float>::quiet_NaN()}) ==
         Status::kInvalidArgument);
  assert(gimbal.SetMode(static_cast<YawPitchGimbal::Mode>(99)) == Status::kInvalidArgument);
  assert(Near(gimbal.target().yaw_rad, 1.0f));
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
  assert(gimbal.SetTarget({0.3f, 0.7f, -0.2f}) == Status::kOk);
  assert(gimbal.Update(0.002f) == Status::kOk);
  assert(Near(yaw.target, 0.3f) && Near(p1.target, 0.7f) && Near(p2.target, -0.2f));
  assert(gimbal.SetMode(YawPitchPitchGimbal::Mode::kLowFire) == Status::kOk);
  assert(gimbal.Update(0.002f) == Status::kOk);
  assert(Near(p1.target, config.low_fire_pitch1_rad));
  assert(Near(yaw.target, 0.3f) && Near(p2.target, -0.2f));
  assert(Near(gimbal.target().pitch1_rad, 0.7f));  // Requested and applied angles differ.
  assert(gimbal.SetMode(YawPitchPitchGimbal::Mode::kHighFire) == Status::kOk);
  assert(gimbal.Update(0.002f) == Status::kOk);
  assert(Near(p1.target, config.high_fire_pitch1_rad));
  assert(gimbal.SetMode(YawPitchPitchGimbal::Mode::kNormal) == Status::kOk);
  assert(gimbal.Update(0.002f) == Status::kOk);
  assert(Near(p1.target, 0.7f));
  assert(Near(gimbal.command()[1], 0.7f) && Near(p2.kp, 12.0f));
  assert(yaw.commits == 4 && p1.commits == 4 && p2.commits == 4);
  assert(gimbal.SetTarget({0, 0, -0.9f}) == Status::kInvalidArgument);
  assert(gimbal.SetMode(static_cast<YawPitchPitchGimbal::Mode>(99)) == Status::kInvalidArgument);
  assert(gimbal.mode() == YawPitchPitchGimbal::Mode::kNormal);
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
  TestConfiguration();
  TestFaults();
}
