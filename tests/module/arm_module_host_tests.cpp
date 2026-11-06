#include <cassert>
#include <limits>
#include "module/arm/arm.hpp"
#include "assembly.hpp"

using mr::module::arm::Arm;
using mr::module::arm::Lifecycle;
struct Fake final : mr::IActuator {
  int8_t register_result = 0, enable_result = 0, update_result = 0, apply_result = 0, commit_result = 0;
  int registers = 0, enables = 0, relaxes = 0, commits = 0, writes = 0;
  bool pending = false;
  mr::ActuatorState state{0, 0, 0, true};
  int8_t Register() override { ++registers; return register_result; }
  int8_t Enable() override { ++enables; return enable_result; }
  int8_t Update() override { return update_result; }
  int8_t Relax() override { ++relaxes; pending = false; return 0; }
  int8_t SetZero() override { return 0; }
  int8_t CommitControl() override { ++commits; pending = false; return commit_result; }
  bool HasPendingControl() const override { return pending; }
  void ClearPendingControl() override { pending = false; }
  int8_t SetMIT(float, float, float, float, float) override {
    ++writes; pending = true; return apply_result;
  }
  int8_t SetTorque(float) override { ++writes; pending = true; return apply_result; }
  mr::ActuatorState GetState() const override { return state; }
};
template<int N> typename Arm<N>::Model MakeModel() {
  mr::robotics::arm::JointModel joints[N];
  for (auto& joint : joints) {
    joint.limits.max_velocity = 2;
    joint.limits.max_acceleration = 4;
  }
  using mr::component::math::matrix::identity;
  return {joints, identity<4>(), identity<4>(),
          mr::component::math::matrix::zeros<3, 1>()};
}
template<int N> void CheckSize() {
  Fake devices[N];
  typename Arm<N>::Config config;
  config.model = MakeModel<N>();
  for (int i = 0; i < N; ++i) config.output.actuators[i] = &devices[i];
  Arm<N> arm;
  assert(arm.Init(config) == 0);
  assert(arm.lifecycle() == Lifecycle::kReady && devices[0].enables == 0);
  assert(arm.Enable() != 0); // No fabricated initial feedback.
  assert(arm.UpdateFeedback() == 0 && arm.Enable() == 0);
  assert(arm.Step(0.002f) == 0 && arm.CommitOutput() == 0);
  assert(arm.CommitOutput() != 0 && devices[0].commits == 1);
  assert(arm.Init(config) == DEVICE_ERR_INITED);
  assert(arm.SetZero(0) != 0);
  assert(arm.Disable() == 0 && arm.SetZero(0) == 0);
  assert(arm.Enable() != 0); // Zeroing invalidates feedback.
}
int main() {
  CheckSize<2>(); CheckSize<3>(); CheckSize<6>();
  Fake a, b;
  Arm<2>::Config config;
  config.model = MakeModel<2>();
  config.output.actuators = {&a, nullptr};
  Arm<2> missing;
  assert(missing.Init(config) != 0 && a.registers == 0);
  config.output.actuators = {&a, &a};
  assert(missing.Init(config) != 0 && a.registers == 0);
  config.output.actuators = {&a, &b};
  config.output.gains[1].kp = std::numeric_limits<float>::quiet_NaN();
  assert(missing.Init(config) != 0 && a.registers == 0);
  config.output.gains[1].kp = 1;
  Arm<2> arm;
  assert(arm.Init(config) == 0 && arm.UpdateFeedback() == 0);
  b.enable_result = -9;
  assert(arm.Enable() == -9 && arm.lifecycle() == Lifecycle::kFault);
  assert(a.relaxes > 0 && b.relaxes > 0 && arm.last_device_error() == -9);
  b.enable_result = 0;
  assert(arm.Enable() != 0); // Fault invalidates snapshot.
  assert(arm.UpdateFeedback() == 0 && arm.Enable() == 0);
  b.apply_result = -8;
  assert(arm.Step(0.002f) == -8 && !a.pending && !b.pending);
  assert(arm.CommitOutput() != 0 && a.commits == 0);
  b.apply_result = 0;
  assert(arm.UpdateFeedback() == 0 && arm.Enable() == 0);
  assert(arm.Step(0.002f) == 0);
  b.commit_result = -7;
  assert(arm.CommitOutput() == -7 && arm.lifecycle() == Lifecycle::kFault);
  assert(!a.pending && !b.pending);
  b.commit_result = 0;
  assert(arm.UpdateFeedback() == 0 && arm.Enable() == 0);
  a.state.position = std::numeric_limits<float>::quiet_NaN();
  assert(arm.UpdateFeedback() != 0 && !arm.feedback().valid);
  assert(arm.Step(0.002f) != 0);
  a.state.position = 0;
  assert(arm.UpdateFeedback() == 0 && arm.Enable() == 0);
  assert(arm.Step(-1) != 0);
  Fake c, d;
  c.register_result = -6;
  config.output.actuators = {&c, &d};
  Arm<2> registration_failed;
  assert(registration_failed.Init(config) == -6);
  assert(registration_failed.UpdateFeedback() == 0 && registration_failed.Enable() == -6);
  // Project-owned three-axis preset and remote adapter remain buildable.
  Arm_Params_t parameters{};
  Fake x, y, z;
  const auto project_config = mr::examples::three_axis_arm::MakeConfig(parameters, {&x, &y, &z});
  Arm<3> project;
  assert(project.Init(project_config) == 0);
}
