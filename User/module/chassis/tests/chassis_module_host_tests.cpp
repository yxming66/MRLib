#include <array>
#include <cmath>
#include <cstdio>
#include <limits>

#include "module/chassis/chassis.hpp"

namespace {

using mr::module::chassis::Chassis;
using mr::module::chassis::ChassisConfig;
using mr::module::chassis::ChassisState;
using mr::module::chassis::ChassisStatus;
using mr::robotics::chassis::ChassisModel;
using mr::robotics::chassis::PlanarVelocity;
using mr::wheel::WheelDevice;
using mr::wheel::WheelFeedback;

class FakeWheel final : public WheelDevice {
 public:
  int8_t Register() override {
    registered = true;
    return register_ok ? 0 : -1;
  }
  int8_t Enable() override {
    enabled = enable_ok;
    return enable_ok ? 0 : -1;
  }
  int8_t Relax() override {
    enabled = false;
    torque_command_nm = 0.0f;
    return 0;
  }
  int8_t UpdateFeedback(WheelFeedback& out) override {
    out = feedback;
    return feedback_ok ? 0 : -1;
  }
  int8_t SetTorque(float torque_nm) override {
    torque_command_nm = torque_nm;
    return set_command_ok ? 0 : -1;
  }
  int8_t UpdateCommand() override { return set_command_ok ? 0 : -1; }
  int8_t CommitCommand() override {
    ++commit_count;
    return commit_ok ? 0 : -1;
  }

  WheelFeedback feedback{0.0f, 0.0f, 0.0f, true};
  float torque_command_nm = 0.0f;
  unsigned commit_count = 0U;
  bool register_ok = true;
  bool enable_ok = true;
  bool feedback_ok = true;
  bool set_command_ok = true;
  bool commit_ok = true;
  bool registered = false;
  bool enabled = false;
};

int failures = 0;

void Expect(bool condition, const char* message) {
  if (!condition) {
    std::printf("FAIL: %s\n", message);
    ++failures;
  }
}

ChassisConfig MakeConfig(ChassisModel model) {
  ChassisConfig config{};
  config.kinematics.model = model;
  config.kinematics.geometry.wheelbase_m = 0.25f;
  config.kinematics.geometry.trackwidth_m = 0.20f;
  config.max_body_velocity = {2.0f, 2.0f, 5.0f};
  config.max_wheel_speed_mps = 4.0f;
  config.wheel_velocity_pid.gains = {1.0f, 20.0f, 0.0f, 0.0f};
  config.wheel_velocity_pid.output_limit =
      mr::comp::cntlr::SymmetricLimit::Abs(8.0f);
  config.wheel_velocity_pid.control_freq_hz = 500.0f;
  return config;
}

void TestMecanumModuleFlow() {
  std::array<FakeWheel, Chassis::kWheelCount> storage{};
  std::array<WheelDevice*, Chassis::kWheelCount> devices{};
  for (std::size_t index = 0; index < devices.size(); ++index) {
    devices[index] = &storage[index];
  }

  Chassis chassis;
  Expect(chassis.Register(MakeConfig(ChassisModel::kMecanum), devices) ==
             ChassisStatus::kOk,
         "mecanum chassis registration should succeed");
  Expect(chassis.state() == ChassisState::kRegistered,
         "registered chassis should report the registered state");
  Expect(chassis.Enable() == ChassisStatus::kOk,
         "all bound wheels should enable");
  Expect(chassis.UpdateFeedback() == ChassisStatus::kOk,
         "fresh wheel feedback should be accepted");
  Expect(chassis.SetCommand({1.0f, 0.2f, 0.5f}) == ChassisStatus::kOk,
         "finite body velocity should be accepted");
  Expect(chassis.Step(0.002f) == ChassisStatus::kOk,
         "control step should compute wheel references and torque");
  Expect(chassis.output().valid, "successful control step should expose output");
  Expect(chassis.CommitOutput() == ChassisStatus::kOk,
         "wheel commands should commit");
  Expect(chassis.CommitOutput() == ChassisStatus::kInvalidState,
         "an output should not be committed twice");
  for (const auto& wheel : storage) {
    Expect(wheel.commit_count == 1U,
           "each wheel should receive one committed command");
    Expect(std::fabs(wheel.torque_command_nm) <= 8.0f,
           "wheel output torque should respect the configured PID limit");
  }
}

void TestModelSelectionAndValidation() {
  std::array<FakeWheel, Chassis::kWheelCount> storage{};
  std::array<WheelDevice*, Chassis::kWheelCount> devices{};
  for (std::size_t index = 0; index < devices.size(); ++index) {
    devices[index] = &storage[index];
  }

  Chassis chassis;
  Expect(chassis.Register(
             MakeConfig(ChassisModel::kFrontOmniRearMecanum), devices) ==
             ChassisStatus::kOk,
         "front omni rear mecanum should register through the same module");
  Expect(chassis.model() == ChassisModel::kFrontOmniRearMecanum,
         "module should expose its selected robotics model");
  Expect(chassis.Enable() == ChassisStatus::kOk,
         "selected model should use the common enable API");
  Expect(chassis.Step(0.002f) == ChassisStatus::kNoFreshFeedback,
         "control step should require new wheel feedback");
  Expect(chassis.UpdateFeedback() == ChassisStatus::kOk,
         "selected model should use the common feedback API");
  Expect(chassis.SetCommand({0.4f, 0.3f, 0.2f}) == ChassisStatus::kOk,
         "selected model should use the common command API");
  Expect(chassis.Step(0.002f) == ChassisStatus::kOk,
         "selected model should use the common control API");

  Expect(chassis.SetCommand(
             {std::numeric_limits<float>::quiet_NaN(), 0.0f, 0.0f}) ==
             ChassisStatus::kInvalidArgument,
         "non-finite command should be rejected and fault the module");
  Expect(chassis.state() == ChassisState::kFault,
         "invalid command should leave the module faulted");
  for (const auto& wheel : storage) {
    Expect(wheel.torque_command_nm == 0.0f,
           "fault handling should relax all registered wheel devices");
  }
}

void TestEnableFailureRelaxesRegisteredDevices() {
  std::array<FakeWheel, Chassis::kWheelCount> storage{};
  std::array<WheelDevice*, Chassis::kWheelCount> devices{};
  for (std::size_t index = 0; index < devices.size(); ++index) {
    devices[index] = &storage[index];
  }
  storage[2].enable_ok = false;

  Chassis chassis;
  Expect(chassis.Register(MakeConfig(ChassisModel::kMecanum), devices) ==
             ChassisStatus::kOk,
         "wheel devices should register before enable failure test");
  Expect(chassis.Enable() == ChassisStatus::kDeviceError,
         "partial wheel enable failure should be reported");
  Expect(chassis.state() == ChassisState::kFault,
         "partial enable failure should fault the module");
  for (const auto& wheel : storage) {
    Expect(!wheel.enabled,
           "partial enable failure should relax every registered wheel");
  }
}

}  // namespace

int main() {
  TestMecanumModuleFlow();
  TestModelSelectionAndValidation();
  TestEnableFailureRelaxesRegisteredDevices();
  if (failures != 0) {
    std::printf("%d chassis module host test(s) failed\n", failures);
    return 1;
  }
  std::printf("chassis module host tests passed\n");
  return 0;
}
