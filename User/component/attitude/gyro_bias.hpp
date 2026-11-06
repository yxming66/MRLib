#pragma once

#include <array>
#include <cstdint>
#include "component/math/scalar.hpp"

namespace mr::component::attitude {

using GyroVector = std::array<float, 3>;
enum class CalibrationState { kIdle, kCollecting, kComplete, kCancelled };

/* Requires consecutive stationary, valid samples. The caller defines what
 * stationary means using its own sensors/robot state; no wheel or button input.
 * Input is raw sensor gyro, BEFORE any prior bias subtraction.
 */
class GyroBiasCalibration {
 public:
  bool Start(uint32_t samples) {
    if (samples == 0) return false;
    required_ = samples;
    count_ = 0;
    mean_ = {};
    state_ = CalibrationState::kCollecting;
    return true;
  }
  void Cancel() {
    if (state_ == CalibrationState::kCollecting) {
      count_ = 0;
      state_ = CalibrationState::kCancelled;
    }
  }
  void BreakWindow() {
    if (state_ == CalibrationState::kCollecting) { count_ = 0; mean_ = {}; }
  }
  bool Add(const GyroVector& raw, bool stationary) {
    if (state_ != CalibrationState::kCollecting) return false;
    for (float v : raw) {
      if (!math::is_finite_scalar(v)) { BreakWindow(); return false; }
    }
    if (!stationary) { BreakWindow(); return false; }
    const uint32_t next_count = count_ + 1;
    GyroVector next{};
    for (unsigned i = 0; i < 3; ++i) {
      // Divide before subtracting to avoid overflowing opposite finite inputs.
      next[i] = mean_[i] + (raw[i] / next_count - mean_[i] / next_count);
      if (!math::is_finite_scalar(next[i])) { BreakWindow(); return false; }
    }
    mean_ = next;
    count_ = next_count;
    if (count_ == required_) { state_ = CalibrationState::kComplete; return true; }
    return false;
  }
  uint32_t count() const { return count_; }
  CalibrationState state() const { return state_; }
  const GyroVector& bias() const { return mean_; }
 private:
  GyroVector mean_{};
  uint32_t count_ = 0, required_ = 0;
  CalibrationState state_ = CalibrationState::kIdle;
};
}  // namespace mr::component::attitude
