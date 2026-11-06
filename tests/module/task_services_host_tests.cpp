#include "module/attitude/attitude.hpp"
#include "module/audio/alarm.hpp"
#include <cmath>
#include <cstdio>

#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); return false; } } while (0)
static bool Near(float a, float b, float eps = 1e-5f) { return std::isfinite(a) && std::fabs(a-b) <= eps; }
static bool Calibration() {
  using namespace mr::component::attitude;
  GyroBiasCalibration calibration;
  CHECK(!calibration.Start(0));
  CHECK(calibration.Start(3));
  CHECK(!calibration.Add({1, 2, 3}, true));
  CHECK(!calibration.Add({1, 2, 3}, false) && calibration.count() == 0);
  CHECK(!calibration.Add({1, 2, 3}, true));
  CHECK(!calibration.Add({NAN, 0, 0}, true) && calibration.count() == 0);
  CHECK(!calibration.Add({1, 2, 3}, true));
  CHECK(!calibration.Add({2, 3, 4}, true));
  CHECK(calibration.Add({3, 4, 5}, true));
  CHECK(calibration.state() == CalibrationState::kComplete && Near(calibration.bias()[0], 2));
  CHECK(calibration.Start(2)); calibration.Cancel();
  CHECK(calibration.state() == CalibrationState::kCancelled && !calibration.Add({1, 1, 1}, true));
  CHECK(calibration.Start(2));
  const float largest = mr::component::math::kFiniteLimit;
  CHECK(!calibration.Add({largest, 0, 0}, true));
  CHECK(calibration.Add({-largest, 0, 0}, true));
  CHECK(calibration.bias()[0] == 0);
  return true;
}
static MrImuSample Sample(uint32_t sequence, uint64_t timestamp, float gyro_z = 0) {
  return {{0, 0, 9.81f}, {0, 0, gyro_z}, timestamp, sequence, true};
}
static bool Attitude() {
  using namespace mr::module::attitude;
  Estimator estimator;
  CHECK(estimator.Step(Sample(0, 0), 0, true) == Status::kNotConfigured);
  Config config{}; config.calibration_samples = 3; config.gyro_bias_rad_s = {0, 0, 0.1f};
  CHECK(estimator.Configure(config) == Status::kOk);
  CHECK(estimator.Step(Sample(0, 0, 0.1f), 0, true) == Status::kOk);
  CHECK(estimator.feedback().valid && Near(estimator.feedback().euler_rad.yaw, 0));
  CHECK(estimator.Step(Sample(0, 0), 0, true) == Status::kNoNewSample);
  CHECK(!estimator.feedback().valid);
  CHECK(estimator.Step(Sample(1, 1000), 500, true) == Status::kStaleSample);
  CHECK(estimator.Step(Sample(1, 1000), 22000, true) == Status::kStaleSample);
  auto bad = Sample(1, 1000); bad.angular_velocity_rad_s.x = NAN;
  CHECK(estimator.Step(bad, 1000, true) == Status::kInvalidSample);
  CHECK(estimator.Step(Sample(1, 1000, 0.1f), 1000, true) == Status::kOk);
  CHECK(estimator.Step(Sample(2, 50000), 50000, true) == Status::kStaleSample);
  CHECK(estimator.Step(Sample(3, 51000, 0.1f), 51000, true) == Status::kOk);
  CHECK(estimator.StartCalibration());
  CHECK(estimator.Step(Sample(4, 52000, 0.2f), 52000, true) == Status::kCalibrating);
  CHECK(estimator.Step(Sample(5, 53000, 0.2f), 53000, false) == Status::kCalibrating);
  CHECK(estimator.calibration().count() == 0);
  for (uint32_t i = 6; i <= 8; ++i)
    CHECK(estimator.Step(Sample(i, 48000 + i*1000, 0.2f), 48000 + i*1000, true) == Status::kCalibrating);
  CHECK(Near(estimator.bias()[2], 0.2f) && !estimator.feedback().valid);
  CHECK(estimator.Step(Sample(9, 57000, 0.2f), 57000, true) == Status::kOk);
  CHECK(Near(estimator.feedback().angular_velocity_rad_s.z, 0));
  // Recalibration uses raw gyro, never the already corrected signal.
  CHECK(estimator.StartCalibration());
  for (uint32_t i = 10; i <= 12; ++i)
    CHECK(estimator.Step(Sample(i, 48000+i*1000, 0.2f), 48000+i*1000, true) == Status::kCalibrating);
  CHECK(Near(estimator.bias()[2], 0.2f));
  config.gyro_bias_rad_s = {};
  CHECK(estimator.Configure(config) == Status::kOk);
  // Measured dt: 501 frames at 2 ms integrate about 1 rad, despite nominal 1 kHz.
  for (uint32_t i = 0; i <= 500; ++i)
    CHECK(estimator.Step(Sample(i, i*2000, 1), i*2000, false) == Status::kOk);
  CHECK(Near(estimator.feedback().euler_rad.yaw, 1.001f, 0.002f));
  config.sample_freq_hz = 0;
  CHECK(estimator.Configure(config) == Status::kInvalidConfig);
  CHECK(!estimator.feedback().valid && !estimator.StartCalibration());
  return true;
}
static bool AlarmTiming() {
  using namespace mr::module::audio;
  Alarm alarm;
  CHECK(!alarm.Step({AlarmLevel::kCritical, 0, 0}, 0).active);
  AlarmConfig config{};
  CHECK(alarm.Configure(config));
  auto output = alarm.Step({AlarmLevel::kWarning, 100, 0}, 100);
  CHECK(output.active && output.frequency_hz == 1000);
  CHECK(alarm.Step({AlarmLevel::kWarning, 180, 0}, 180).frequency_hz == 800);
  CHECK(!alarm.Step({AlarmLevel::kWarning, 100, 0}, 351).active);
  CHECK(!alarm.Step({AlarmLevel::kCritical, 400, 0}, 399).active);
  CHECK(alarm.Step({AlarmLevel::kCritical, 500, 500}, 500).active);
  CHECK(alarm.Step({AlarmLevel::kWarning, 600, 0}, 600).level == AlarmLevel::kCritical);
  CHECK(alarm.Step({}, 999).level == AlarmLevel::kCritical);
  CHECK(!alarm.Step({}, 1000).active);
  CHECK(alarm.Step({AlarmLevel::kWarning, 1100, 1000}, 1100).active);
  CHECK(alarm.Step({AlarmLevel::kCritical, 1101, 0}, 1101).level == AlarmLevel::kCritical);
  CHECK(!alarm.Step({}, 1102).active); // New priority starts its own minimum hold.
  alarm.Reset();
  CHECK(alarm.Step({AlarmLevel::kWarning, UINT32_MAX-10, 0}, UINT32_MAX-10).active);
  CHECK(alarm.Step({AlarmLevel::kWarning, UINT32_MAX-10, 0}, 69).frequency_hz == 800);
  CHECK(!alarm.Step({AlarmLevel::kWarning, UINT32_MAX-10, 0}, 240).active);
  config.tone_period_ms = 0;
  CHECK(!alarm.Configure(config) && !alarm.Step({AlarmLevel::kCritical, 0, 0}, 0).active);
  return true;
}
int main() {
  if (!Calibration() || !Attitude() || !AlarmTiming()) return 1;
  std::puts("Task service tests passed (calibration, attitude freshness/dt, alarm timing)");
}
