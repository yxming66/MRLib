#include <cstdio>
#include <limits>

#include "device/joint/joint.hpp"

namespace {

struct FakeActuator final : mr::IActuator {
  int8_t update_result = DEVICE_OK;
  int8_t set_mit_result = DEVICE_OK;
  int8_t set_torque_result = DEVICE_OK;
  int mit_calls = 0;
  int torque_calls = 0;
  float last_position = 0.0f;
  float last_kp = 0.0f;
  float last_kd = 0.0f;
  mr::ActuatorState state{};

  int8_t Register() override { return DEVICE_OK; }
  int8_t Enable() override { return DEVICE_OK; }
  int8_t Update() override { return update_result; }
  int8_t Relax() override { return DEVICE_OK; }
  int8_t SetZero() override { return DEVICE_OK; }
  int8_t CommitControl() override { return DEVICE_OK; }
  bool HasPendingControl() const override { return false; }
  void ClearPendingControl() override {}
  int8_t SetMIT(float position, float, float kp, float kd,
                float) override {
    ++mit_calls;
    last_position = position;
    last_kp = kp;
    last_kd = kd;
    return set_mit_result;
  }
  int8_t SetTorque(float) override {
    ++torque_calls;
    return set_torque_result;
  }
  mr::ActuatorState GetState() const override { return state; }
};

int failures = 0;

void Expect(bool condition, const char* message) {
  if (!condition) {
    std::printf("FAIL: %s\n", message);
    ++failures;
  }
}

mr::JointT MakeJoint(FakeActuator* actuator, float kp = 3.0f,
                     float kd = 0.4f) {
  return mr::JointT(1U, actuator, {-1.0f, 1.0f, kp, kd}, 0.0f, 1000.0f);
}

void TestExplicitGainsAndRejectedCache() {
  FakeActuator actuator;
  actuator.state.online = true;
  mr::JointT joint = MakeJoint(&actuator, 0.0f, 0.0f);
  Expect(joint.Update() == DEVICE_OK,
         "joint should accept a finite online actuator snapshot");
  Expect(joint.PositionControl(0.5f, 0.001f) == DEVICE_OK,
         "zero gains should be passed through explicitly");
  Expect(actuator.last_kp == 0.0f && actuator.last_kd == 0.0f,
         "joint must not infer gains from its numeric id");
  Expect(joint.GetMotorCommandPosition() == 0.5f,
         "accepted position command should update the cache");

  actuator.set_mit_result = DEVICE_ERR;
  Expect(joint.PositionControl(-0.5f, 0.001f) == DEVICE_ERR,
         "actuator failure should be returned");
  Expect(joint.GetMotorCommandPosition() == 0.5f,
         "rejected position command must not update the cache");
}

void TestTorqueValidationAndRejectedCache() {
  FakeActuator actuator;
  mr::JointT joint = MakeJoint(&actuator);
  Expect(joint.TorqueControl(1.25f) == DEVICE_OK,
         "finite torque should be accepted");
  Expect(joint.GetMotorCommandTorque() == 1.25f,
         "accepted torque should update the cache");
  actuator.set_torque_result = DEVICE_ERR;
  Expect(joint.TorqueControl(2.0f) == DEVICE_ERR,
         "torque actuator failure should be returned");
  Expect(joint.GetMotorCommandTorque() == 1.25f,
         "rejected torque must not update the cache");
  Expect(joint.TorqueControl(std::numeric_limits<float>::quiet_NaN()) ==
             DEVICE_ERR,
         "non-finite torque should be rejected before reaching the actuator");
  Expect(joint.GetMotorCommandTorque() == 1.25f,
         "invalid torque must not update the cache");
}

void TestInvalidGainsAndSetters() {
  FakeActuator actuator;
  actuator.state.online = true;
  mr::JointT invalid_gain_joint = MakeJoint(&actuator, -1.0f, 0.2f);
  Expect(invalid_gain_joint.Update() == DEVICE_OK,
         "joint should refresh feedback before position control");
  Expect(invalid_gain_joint.PositionControl(0.2f, 0.001f) == DEVICE_ERR,
         "negative joint gains should be rejected");
  Expect(actuator.mit_calls == 0,
         "invalid gain configuration must fail before actuator submission");

  mr::JointT joint = MakeJoint(&actuator);
  joint.SetTargetAngle(std::numeric_limits<float>::quiet_NaN());
  joint.SetFeedforwardTorque(std::numeric_limits<float>::infinity());
  Expect(joint.GetTargetAngle() == 0.0f &&
             joint.GetFeedforwardTorque() == 0.0f,
         "void setters should preserve their last finite values");
}

void TestFailedFeedbackInvalidatesJointAndPreservesSnapshot() {
  FakeActuator actuator;
  actuator.state.position = 0.4f;
  actuator.state.velocity = 0.1f;
  actuator.state.torque = 0.2f;
  actuator.state.online = true;
  mr::JointT joint = MakeJoint(&actuator);
  Expect(joint.Update() == DEVICE_OK && joint.IsOnline(),
         "valid actuator feedback should mark the joint online");
  const int mit_calls_before = actuator.mit_calls;

  actuator.state.position = 0.8f;
  actuator.state.velocity = 0.3f;
  actuator.state.torque = 0.5f;
  actuator.update_result = DEVICE_ERR_NO_DEV;
  Expect(joint.Update() == DEVICE_ERR_NO_DEV && !joint.IsOnline(),
         "failed actuator feedback should mark the joint offline");
  Expect(joint.GetCurrentAngle() == 0.4f,
         "failed feedback must preserve the last known numeric snapshot");
  Expect(joint.PositionControl(0.6f, 0.001f) == DEVICE_ERR_NO_DEV &&
             actuator.mit_calls == mit_calls_before,
         "position control must not use a stale snapshot while offline");

  actuator.update_result = DEVICE_OK;
  Expect(joint.Update() == DEVICE_OK && joint.IsOnline() &&
             joint.GetCurrentAngle() == 0.8f,
         "joint should recover when the actuator provides a fresh snapshot");
}

}  // namespace

int main() {
  TestExplicitGainsAndRejectedCache();
  TestTorqueValidationAndRejectedCache();
  TestInvalidGainsAndSetters();
  TestFailedFeedbackInvalidatesJointAndPreservesSnapshot();
  if (failures != 0) {
    std::printf("%d joint host test(s) failed\n", failures);
    return 1;
  }
  std::printf("Joint host tests passed\n");
  return 0;
}
