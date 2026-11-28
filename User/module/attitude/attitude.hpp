#pragma once

#include <algorithm>
#include <cmath>

#include "component/ahrs.h"
#include "component/attitude/gyro_bias.hpp"
#include "component/attitude/quaternion_ekf.hpp"
#include "device/imu/imu_sample.h"

namespace mr::module::attitude {

enum class Status { kOk, kInvalidConfig, kInvalidSample, kNoNewSample,
                    kStaleSample, kCalibrating, kNotConfigured };
enum class Algorithm { kUnspecified, kMadgwick, kQuaternionEkf };
struct Config {
  Algorithm algorithm = Algorithm::kUnspecified;
  float sample_freq_hz = 1000.0f;
  uint32_t max_sample_age_us = 20000;
  uint32_t max_interval_us = 20000;
  uint32_t calibration_samples = 5000;
  component::attitude::GyroVector gyro_bias_rad_s{};
  component::attitude::QuaternionEkf::Config qekf{};
};
struct Feedback {
  AHRS_Quaternion_t quaternion{};
  AHRS_Eulr_t euler_rad{};
  float yaw_total_rad = 0.0f;
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
    if ((config.algorithm != Algorithm::kMadgwick && config.algorithm != Algorithm::kQuaternionEkf) ||
        !component::math::is_positive_scalar(config.sample_freq_hz) ||
        config.sample_freq_hz > 1000000.0f || config.max_sample_age_us == 0 ||
        config.max_interval_us == 0 || config.calibration_samples == 0)
      return Status::kInvalidConfig;
    for (float v : config.gyro_bias_rad_s)
      if (!component::math::is_finite_scalar(v)) return Status::kInvalidConfig;
    config_ = config;
    bias_ = config.gyro_bias_rad_s;
    calibration_ = {};
    consumed_ = false;
    yaw_turns_ = 0; last_yaw_rad_ = 0.0f; yaw_initialized_ = false;
    if (config.algorithm == Algorithm::kQuaternionEkf)
      configured_ = qekf_.Configure(config.qekf);
    else
      configured_ = AHRS_Init(&ahrs_, nullptr, config.sample_freq_hz) == 0;
    return configured_ ? Status::kOk : Status::kInvalidConfig;
  }
  bool StartCalibration() {
    if (!configured_ || config_.algorithm == Algorithm::kQuaternionEkf) return false;
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
    if (config_.algorithm == Algorithm::kQuaternionEkf) {
      const float gyro[3] = {raw.x, raw.y, raw.z};
      const float accel[3] = {sample.acceleration_mps2.x,
                              sample.acceleration_mps2.y,
                              sample.acceleration_mps2.z};
      const float dt = interval ? static_cast<float>(interval) * 1.0e-6f
                                : 1.0f / config_.sample_freq_hz;
      if (!qekf_.Step(gyro, accel, dt)) return Status::kInvalidSample;
      const auto& x = qekf_.state();
      const float q0=x(0,0), q1=x(1,0), q2=x(2,0), q3=x(3,0);
      feedback_.quaternion = {q0,q1,q2,q3};
      feedback_.euler_rad.yaw = std::atan2(2.0f*(q0*q3+q1*q2),
                                           2.0f*(q0*q0+q1*q1)-1.0f);
      feedback_.euler_rad.pit = std::atan2(2.0f*(q0*q1+q2*q3),
                                           2.0f*(q0*q0+q3*q3)-1.0f);
      const float roll_arg = std::clamp(-2.0f*(q1*q3-q0*q2), -1.0f, 1.0f);
      feedback_.euler_rad.rol = std::asin(roll_arg);
      UpdateYawTotal(feedback_.euler_rad.yaw);
      feedback_.angular_velocity_rad_s = {raw.x-x(4,0), raw.y-x(5,0), raw.z};
      feedback_.timestamp_us = sample.timestamp_us;
      feedback_.sequence = sample.sequence;
      feedback_.valid = true;
      return Status::kOk;
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
    UpdateYawTotal(angles.yaw);
    feedback_.angular_velocity_rad_s = {gyro.x, gyro.y, gyro.z};
    feedback_.timestamp_us = sample.timestamp_us;
    feedback_.sequence = sample.sequence;
    feedback_.valid = true;
    return Status::kOk;
  }
  const Feedback& feedback() const { return feedback_; }
  const component::attitude::GyroVector& bias() const { return bias_; }
  component::attitude::GyroVector estimated_bias_rad_s() const {
    if (config_.algorithm == Algorithm::kQuaternionEkf)
      return {qekf_.gyro_bias_x_rad_s(), qekf_.gyro_bias_y_rad_s(), 0.0f};
    return bias_;
  }
  const component::attitude::GyroBiasCalibration& calibration() const { return calibration_; }
 private:
  void UpdateYawTotal(float yaw_rad) {
    if (yaw_initialized_) {
      const float delta = yaw_rad - last_yaw_rad_;
      if (delta > component::math::kPi) yaw_turns_--;
      else if (delta < -component::math::kPi) yaw_turns_++;
    } else {
      yaw_initialized_ = true;
    }
    last_yaw_rad_ = yaw_rad;
    feedback_.yaw_total_rad = static_cast<float>(yaw_turns_) * component::math::kTwoPi + yaw_rad;
  }
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
  component::attitude::QuaternionEkf qekf_{};
  component::attitude::GyroVector bias_{};
  component::attitude::GyroBiasCalibration calibration_{};
  uint64_t last_timestamp_ = 0;
  uint32_t last_sequence_ = 0;
  int32_t yaw_turns_ = 0;
  float last_yaw_rad_ = 0.0f;
  bool yaw_initialized_ = false;
  bool consumed_ = false, configured_ = false;
};
}  // namespace mr::module::attitude
