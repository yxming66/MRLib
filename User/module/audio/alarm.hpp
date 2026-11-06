#pragma once

#include <cstdint>
#include "component/math/scalar.hpp"

namespace mr::module::audio {
enum class AlarmLevel : uint8_t { kNone, kWarning, kCritical };
struct AlarmRequest {
  AlarmLevel level = AlarmLevel::kNone;
  uint32_t timestamp_ms = 0;
  uint32_t min_duration_ms = 0;
};
struct AlarmConfig {
  uint32_t request_timeout_ms = 250;
  uint32_t tone_period_ms = 80;
  float low_frequency_hz = 800.0f;
  float high_frequency_hz = 1000.0f;
  float duty = 0.5f;
};
struct AlarmOutput {
  AlarmLevel level = AlarmLevel::kNone;
  float frequency_hz = 0.0f;
  float duty = 0.0f;
  bool active = false;
};

/* Deterministic alarm timing from the former blink task. All times are ms,
 * independent of OS tick rate; polls/requests must be less than 2^31 ms apart.
 * The app owns music/calibration priority and writes this output to its buzzer.
 * No received request survives forever: timeout and a start-based minimum hold.
 */
class Alarm {
 public:
  bool Configure(const AlarmConfig& config) {
    configured_ = false;
    Reset();
    if (config.request_timeout_ms == 0 || config.request_timeout_ms > INT32_MAX ||
        config.tone_period_ms == 0 || config.tone_period_ms > INT32_MAX ||
        !component::math::is_positive_scalar(config.low_frequency_hz) ||
        !component::math::is_positive_scalar(config.high_frequency_hz) ||
        !component::math::is_finite_scalar(config.duty) || config.duty <= 0 || config.duty > 1)
      return false;
    config_ = config;
    configured_ = true;
    return true;
  }
  void Reset() { output_ = {}; start_ms_ = 0; minimum_ms_ = 0; }
  AlarmOutput Step(const AlarmRequest& request, uint32_t now_ms) {
    if (!configured_) return {};
    const uint32_t age = now_ms - request.timestamp_ms;
    const bool valid_level = request.level == AlarmLevel::kWarning || request.level == AlarmLevel::kCritical;
    const bool fresh = valid_level && age <= config_.request_timeout_ms &&
                       request.min_duration_ms <= INT32_MAX;
    const bool holding = output_.active && now_ms - start_ms_ < minimum_ms_;
    AlarmLevel target = fresh ? request.level : AlarmLevel::kNone;
    // A lower priority request cannot shorten an active minimum critical hold.
    if (holding && static_cast<uint8_t>(target) < static_cast<uint8_t>(output_.level))
      target = output_.level;
    if (target == AlarmLevel::kNone) { Reset(); return output_; }
    if (!output_.active || output_.level != target) {
      start_ms_ = now_ms;
      minimum_ms_ = fresh ? request.min_duration_ms : 0;
    }
    const bool high = ((now_ms - start_ms_) / config_.tone_period_ms) % 2U == 0;
    output_ = {target, high ? config_.high_frequency_hz : config_.low_frequency_hz,
               config_.duty, true};
    return output_;
  }
 private:
  AlarmConfig config_{};
  AlarmOutput output_{};
  uint32_t start_ms_ = 0, minimum_ms_ = 0;
  bool configured_ = false;
};
}  // namespace mr::module::audio
