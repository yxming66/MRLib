#include <cmath>
#include <cstdio>

#include "module/stair_assist/stair_assist.hpp"

namespace {
using namespace mr::module::stair_assist;

class FakeAxis final : public AxisDevice {
 public:
  int8_t Register() override { ++register_count; return 0; }
  int8_t Enable() override { ++enable_count; return 0; }
  int8_t UpdateFeedback(AxisFeedback& out) override { out = feedback; return 0; }
  int8_t SetPosition(float value, float speed) override {
    position = value;
    position_speed = speed;
    return 0;
  }
  int8_t SetVelocity(float value) override { velocity = value; return 0; }
  int8_t UpdateCommand() override { ++update_count; return 0; }
  int8_t CommitCommand() override { ++commit_count; return 0; }
  void ClearPendingCommand() override {}
  int8_t Relax() override { return 0; }

  AxisFeedback feedback{0.0f, 0.0f, 100U, true};
  float position = 0.0f;
  float position_speed = 0.0f;
  float velocity = 0.0f;
  unsigned register_count = 0;
  unsigned enable_count = 0;
  unsigned update_count = 0;
  unsigned commit_count = 0;
};

int failures = 0;
void Expect(bool condition, const char* message) {
  if (condition) return;
  std::printf("FAIL: %s\n", message);
  ++failures;
}

void TestCommandsArePreparedBeforeCommit() {
  FakeAxis front, support, left, right;
  StairAssist module;
  const Config config{-1.0f, 1.0f, 2.0f, 0.0f, 300.0f, 50.0f, 20.0f, 20U};
  const Devices devices{&front, &support, &left, &right};
  Expect(module.Register(config, devices) == Status::kOk, "register axes");
  Expect(module.Enable() == Status::kOk, "enable axes");
  Expect(module.UpdateFeedback(100U) == Status::kOk, "read fresh feedback");
  const Command command{0.5f, 120.0f, 3.0f, -4.0f};
  Expect(module.SetCommand(command) == Status::kOk, "accept in-range command");
  Expect(module.PrepareOutput() == Status::kOk, "prepare actuator commands");
  Expect(front.position == 0.5f && support.position == 120.0f,
         "map pivot and support positions");
  Expect(left.velocity == 3.0f && right.velocity == -4.0f,
         "map independent track speeds");
  Expect(front.commit_count == 0U && support.commit_count == 0U,
         "preparation must not commit device output");
  Expect(module.CommitOutput() == Status::kOk, "commit prepared outputs");
  Expect(front.commit_count == 1U && right.commit_count == 1U,
         "commit each axis once");
}

void TestOutOfRangeCommandRejected() {
  FakeAxis front, support, left, right;
  StairAssist module;
  const Config config{-1.0f, 1.0f, 2.0f, 0.0f, 300.0f, 50.0f, 20.0f, 20U};
  Expect(module.Register(config, {&front, &support, &left, &right}) == Status::kOk,
         "register for range test");
  Expect(module.Enable() == Status::kOk, "enable for range test");
  Expect(module.SetCommand({1.1f, 100.0f, 0.0f, 0.0f}) == Status::kInvalidArgument,
         "reject pivot outside configured travel");
}
}  // namespace

int main() {
  TestCommandsArePreparedBeforeCommit();
  TestOutOfRangeCommandRejected();
  if (failures) std::printf("%d stair assist module test(s) failed\n", failures);
  return failures ? 1 : 0;
}
