#pragma once

#include "component/ahrs.h"
#include "component/attitude/gyro_bias.hpp"
#include "device/imu/imu_sample.h"

namespace mr::module::attitude {

enum class Status { kOk, kInvalidConfig, kInvalidSample, kNoNewSample,
                    kStaleSample, kCalibrating, kNotConfigured };
struct Config {
  float sample_freq_hz = 1000.0f;
  uint32_t max_sample_age_us = 20000;
  uint32_t max_interval_us = 20000;
  uint32_t calibration_samples = 5000;
  component::attitude::GyroVector gyro_bias_rad_s{};
};
struct Feedback {
  AHRS_Quaternion_t quaternion{};
  AHRS_Eulr_t euler_rad{};
  MrVector3 angular_velocity_rad_s{};
  uint64_t timestamp_us = 0;
  uint32_t sequence = 0;
  bool valid = false;
};

/* IMU-only estimator extracted from the former atti_esti task. Axis conversion,
 * sensor reads, heater PID, button/RC requests, queue publication and calibration
 * sounds belong to board/app. No implicit dynamic Z bias tied to wheel RPM.
 * Step consumes each frame once. During calibration feedback is invalid;
 * completing calibration resets attitude to identity (explicit IMU-only reset).
 */
class Estimator {
 public:
  Status Configure(const Config& config) {
    configured_ = false;
    feedback_ = {};
    if (!component::math::is_positive_scalar(config.sample_freq_hz) ||
        config.sample_freq_hz > 1000000.0f || config.max_sample_age_us == 0 ||
        config.max_interval_us == 0 || config.calibration_samples == 0)
      return Status::kInvalidConfig;
    for (float v : config.gyro_bias_rad_s)
      if (!component::math::is_finite_scalar(v)) return Status::kInvalidConfig;
    config_ = config;
    bias_ = config.gyro_bias_rad_s;
    calibration_ = {};
    consumed_ = false;
    configured_ = AHRS_Init(&ahrs_, nullptr, config.sample_freq_hz) == 0;
    return configured_ ? Status::kOk : Status::kInvalidConfig;
  }
  bool StartCalibration() {
    if (!configured_) return false;
    feedback_.valid = false;
    return calibration_.Start(config_.calibration_samples);
  }
  void CancelCalibration() { calibration_.Cancel(); }
  Status Step(const MrImuSample& sample, uint64_t now_us, bool stationary) {
    if (!configured_) return Status::kNotConfigured;
    feedback_.valid = false;
    if (!sample.valid || !Finite(sample.acceleration_mps2) ||
        !Finite(sample.angular_velocity_rad_s)) {
      calibration_.BreakWindow();
      return Status::kInvalidSample;
    }
    if (sample.timestamp_us > now_us ||
        now_us - sample.timestamp_us > config_.max_sample_age_us) {
      calibration_.BreakWindow();
      return Status::kStaleSample;
    }
    if (consumed_ && (sample.sequence == last_sequence_ ||
                      sample.timestamp_us <= last_timestamp_))
      return Status::kNoNewSample;
    const uint64_t interval = consumed_ ? sample.timestamp_us - last_timestamp_ : 0;
    last_sequence_ = sample.sequence;
    last_timestamp_ = sample.timestamp_us;
    consumed_ = true;
    if (interval > config_.max_interval_us) {
      calibration_.BreakWindow();
      return Status::kStaleSample;
    }
    const auto& raw = sample.angular_velocity_rad_s;
    if (calibration_.state() == component::attitude::CalibrationState::kCollecting) {
      if (calibration_.Add({raw.x, raw.y, raw.z}, stationary)) {
        bias_ = calibration_.bias();
        AHRS_Init(&ahrs_, nullptr, config_.sample_freq_hz);
      }
      return Status::kCalibrating;
    }
    ahrs_.inv_sample_freq = interval ? static_cast<float>(interval) * 1e-6f
                                     : 1.0f / config_.sample_freq_hz;
    const AHRS_Accl_t acceleration = {sample.acceleration_mps2.x,
                                    sample.acceleration_mps2.y,
                                    sample.acceleration_mps2.z};
    const AHRS_Gyro_t gyro = {raw.x - bias_[0], raw.y - bias_[1], raw.z - bias_[2]};
    AHRS_t next = ahrs_;
    AHRS_Eulr_t angles{};
    if (AHRS_Update(&next, &acceleration, &gyro, nullptr) != 0 ||
        !FiniteQuaternion(next.quat) || AHRS_GetEulr(&angles, &next) != 0 ||
        !Finite({angles.rol, angles.pit, angles.yaw})) return Status::kInvalidSample;
    ahrs_ = next;
    feedback_.quaternion = next.quat;
    feedback_.euler_rad = angles;
    feedback_.angular_velocity_rad_s = {gyro.x, gyro.y, gyro.z};
    feedback_.timestamp_us = sample.timestamp_us;
    feedback_.sequence = sample.sequence;
    feedback_.valid = true;
    return Status::kOk;
  }
  const Feedback& feedback() const { return feedback_; }
  const component::attitude::GyroVector& bias() const { return bias_; }
  const component::attitude::GyroBiasCalibration& calibration() const { return calibration_; }
 private:
  static bool Finite(const MrVector3& v) {
    return component::math::is_finite_scalar(v.x) &&
           component::math::is_finite_scalar(v.y) && component::math::is_finite_scalar(v.z);
  }
  static bool FiniteQuaternion(const AHRS_Quaternion_t& q) {
    return Finite({q.q0, q.q1, q.q2}) && component::math::is_finite_scalar(q.q3);
  }
  Config config_{};
  Feedback feedback_{};
  AHRS_t ahrs_{};
  component::attitude::GyroVector bias_{};
  component::attitude::GyroBiasCalibration calibration_{};
  uint64_t last_timestamp_ = 0;
  uint32_t last_sequence_ = 0;
  bool consumed_ = false, configured_ = false;
};
}  // namespace mr::module::attitude
