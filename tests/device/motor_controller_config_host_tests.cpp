#include <cstdio>

#include "device/motor/packages/controller/motor_controller.hpp"

namespace {

int failures = 0;

uint64_t FakeClockUs() {
  return 123456U;
}

void Expect(bool condition, const char* message) {
  if (!condition) {
    std::printf("FAIL: %s\n", message);
    ++failures;
  }
}

}  // namespace

int main() {
  mr::motor::MotorControllerConfig default_config{};
  Expect(default_config.clock_us == nullptr,
         "default motor controller config should not assume a board clock");

  mr::motor::MotorControllerConfig injected_config(
      nullptr, nullptr, 1000.0f, 0.0f, 0.0f, &FakeClockUs);
  Expect(injected_config.clock_us == &FakeClockUs,
         "motor controller config should preserve the injected clock callback");

  mr::motor::MotorControllerConfig legacy_config(
      nullptr, nullptr, 1000.0f, 0.0f, 0.0f, 0.0f, 0.0f);
  Expect(legacy_config.clock_us == nullptr,
         "legacy motor controller config constructor should remain available");

  if (failures != 0) {
    std::printf("%d motor controller config host test(s) failed\n", failures);
    return 1;
  }
  std::printf("Motor controller config host tests passed\n");
  return 0;
}
