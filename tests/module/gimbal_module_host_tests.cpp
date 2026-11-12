#include <cassert>
#include <cmath>
#include <limits>

#include "module/gimbal/gimbal.hpp"

namespace {
bool Near(float lhs, float rhs, float tolerance = 1.0e-5f) {
  return std::fabs(lhs - rhs) <= tolerance;
}

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

mr::module::gimbal::GimbalConfig MakeConfig(FakeActuator& yaw, FakeActuator& pitch) {
  mr::module::gimbal::GimbalConfig config;
  config.axes = {&yaw, &pitch};
  config.gains = {{{20.0f, 1.0f}, {16.0f, 0.8f}}};
  config.kinematics.yaw_pitch.min_yaw_rad = -3.14159265f;
  config.kinematics.yaw_pitch.max_yaw_rad = 3.14159265f;
  config.kinematics.yaw_pitch.min_pitch_rad = -0.5f;
  config.kinematics.yaw_pitch.max_pitch_rad = 0.75f;
  return config;
}
}

int main() {
  using mr::module::gimbal::Gimbal;
  using mr::module::gimbal::Lifecycle;
  using mr::module::gimbal::Status;

  FakeActuator yaw, pitch;
  Gimbal gimbal;
  auto config = MakeConfig(yaw, pitch);
  assert(gimbal.SetTargetAngles({0, 0}) == Status::kInvalidState);
  assert(gimbal.Register(config) == Status::kOk);
  assert(yaw.registered == 1 && pitch.registered == 1 && yaw.enabled == 0);
  assert(gimbal.Enable() == Status::kInvalidFeedback);
  assert(gimbal.UpdateFeedback() == Status::kOk);
  assert(gimbal.Enable() == Status::kOk && gimbal.lifecycle() == Lifecycle::kEnabled);
  assert(Near(gimbal.command().yaw_rad, 0) && Near(gimbal.command().pitch_rad, 0));
  assert(gimbal.SetTargetDirection({0, 1, 0.5f}) == Status::kOk);
  assert(Near(gimbal.command().yaw_rad, 1.5707963f));
  assert(Near(gimbal.command().pitch_rad, 0.4636476f));
  assert(gimbal.Step(0.002f) == Status::kOk);
  assert(yaw.writes == 1 && pitch.writes == 1 && yaw.pending && pitch.pending);
  assert(Near(yaw.target, gimbal.command().yaw_rad) && Near(yaw.kp, 20) && Near(pitch.kd, 0.8f));
  assert(gimbal.UpdateFeedback() == Status::kInvalidState); // Do not discard staged output.
  assert(gimbal.CommitOutput() == Status::kOk && yaw.commits == 1 && pitch.commits == 1);
  assert(gimbal.CommitOutput() == Status::kInvalidState);
  assert(gimbal.SetTargetDirection({0, 0, 1}) == Status::kKinematicsError);
  assert(gimbal.SetTargetAngles({4.0f, 0.0f}) == Status::kInvalidArgument);
  assert(gimbal.UpdateFeedback() == Status::kOk);
  assert(gimbal.Step(0.0f) == Status::kInvalidArgument);
  assert(gimbal.lifecycle() == Lifecycle::kFault && !yaw.pending && !pitch.pending);
  assert(yaw.relaxed > 0 && pitch.relaxed > 0);

  FakeActuator a, b;
  b.register_result = -7;
  Gimbal failed_registration;
  assert(failed_registration.Register(MakeConfig(a, b)) == Status::kDeviceError);
  assert(failed_registration.lifecycle() == Lifecycle::kFault && a.relaxed == 1);

  FakeActuator c, d;
  Gimbal bad_feedback;
  assert(bad_feedback.Register(MakeConfig(c, d)) == Status::kOk);
  d.state.online = false;
  assert(bad_feedback.UpdateFeedback() == Status::kInvalidFeedback);
  assert(bad_feedback.lifecycle() == Lifecycle::kFault);

  FakeActuator e, f;
  Gimbal failed_output;
  assert(failed_output.Register(MakeConfig(e, f)) == Status::kOk);
  assert(failed_output.UpdateFeedback() == Status::kOk && failed_output.Enable() == Status::kOk);
  f.set_result = -9;
  assert(failed_output.Step(0.002f) == Status::kDeviceError);
  assert(failed_output.lifecycle() == Lifecycle::kFault && !e.pending && !f.pending);
  assert(failed_output.last_device_error() == -9 && failed_output.last_relax_error() == DEVICE_OK);

  FakeActuator x, z;
  Gimbal invalid_config;
  auto duplicate = MakeConfig(x, z);
  duplicate.axes[1] = duplicate.axes[0];
  assert(invalid_config.Register(duplicate) == Status::kInvalidConfiguration);
  assert(x.registered == 0);
}
