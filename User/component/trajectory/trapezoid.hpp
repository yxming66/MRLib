#pragma once

/*
 * 梯形速度曲线规划器。
 *
 * 控制律（三段式）：
 *   Phase 1 (0 <= t < t_acc): 匀加速
 *     acceleration = a
 *     velocity = a * t
 *     position = 0.5 * a * t^2
 *
 *   Phase 2 (t_acc <= t < t_acc + t_cruise): 匀速
 *     acceleration = 0
 *     velocity = v_peak
 *     position = 0.5 * a * t_acc^2 + v_peak * (t - t_acc)
 *
 *   Phase 3 (t >= t_acc + t_cruise): 匀减速
 *     acceleration = -a
 *     velocity = v_peak - a * (t - t_acc - t_cruise)
 *     position = d_acc + v_peak * (t - t_acc) - 0.5 * a * (t - t_acc)^2
 *
 * 适用于：
 *   - 单轴位置运动规划
 *   - 梯形加减速曲线：匀加速 -> 匀速 -> 匀减速
 *   - 速度约束、加速约束下的最短时间轨迹
 *
 * 使用示例：
 *   // 初始化：配置规划器
 *   auto planner = mr::comp::traj::TrapezoidProfile();
 *   planner.Configure({distance, max_velocity, max_acceleration});
 *   float start_time = get_current_time();
 *
 *   // 控制循环中（每周期执行）：
 *   float elapsed = get_current_time() - start_time;
 *   auto sample = planner.Sample(elapsed);
 *   // sample.position, sample.velocity, sample.acceleration
 */

#include <cmath>

#include "trajectory_types.hpp"

namespace mr::comp::traj {

struct TrapezoidProfileConfig {
  Scalar distance = 0.0f;
  Scalar max_velocity = 0.0f;
  Scalar max_acceleration = 0.0f;
  Scalar epsilon = kDefaultEpsilon;
};

class TrapezoidProfile {
 public:
  using Config = TrapezoidProfileConfig;

  TrapezoidProfile()
      : distance_(0.0f),
        direction_(1.0f),
        abs_distance_(0.0f),
        peak_velocity_(0.0f),
        acceleration_(0.0f),
        accel_time_(0.0f),
        cruise_time_(0.0f),
        duration_(0.0f),
        epsilon_(kDefaultEpsilon),
        valid_(false) {}

  static Scalar MinDuration(const Config& config) {
    const double d = std::fabs(static_cast<double>(config.distance));
    const double v = config.max_velocity, a = config.max_acceleration;
    if (!is_finite_scalar(config.distance) || !is_finite_scalar(config.max_velocity) ||
        !is_finite_scalar(config.max_acceleration) || !is_finite_scalar(config.epsilon) ||
        config.epsilon <= 0 || v < 0 || a < 0) return -1;
    if (d <= config.epsilon) return 0;
    if (v <= 0 || a <= 0) return -1;
    const double t = d <= v*v/a ? 2*std::sqrt(d/a) : d/v + v/a;
    Scalar result = static_cast<Scalar>(t);
    if (result < t) result = std::nextafter(result, INFINITY);
    return is_finite_scalar(result) ? result : -1;
  }

  bool Configure(const Config& config) {
    return ConfigureForDuration(config, MinDuration(config));
  }

  bool ConfigureForDuration(const Config& config, Scalar duration) {
    reset();
    const Scalar min_time = MinDuration(config);
    if (min_time < 0 || !is_finite_scalar(duration) || duration < min_time) return false;
    distance_ = config.distance;
    direction_ = distance_ >= 0 ? 1 : -1;
    abs_distance_ = abs_scalar(distance_);
    duration_ = duration;
    epsilon_ = config.epsilon;
    if (min_time == 0) { valid_ = true; return true; }
    const double d = abs_distance_, a = config.max_acceleration, v = config.max_velocity;
    const double ta = std::fmin(v/a, std::sqrt(d/a));
    const double tc = std::fmax(0.0, d/(a*ta)-ta);
    const double stretch = duration / (2*ta+tc);
    accel_time_ = static_cast<Scalar>(ta*stretch);
    cruise_time_ = static_cast<Scalar>(tc*stretch);
    peak_velocity_ = static_cast<Scalar>(a*ta/stretch);
    acceleration_ = static_cast<Scalar>(a/(stretch*stretch));
    valid_ = is_finite_scalar(acceleration_) && is_finite_scalar(peak_velocity_);
    return valid_;
  }

  ScalarMotionSample Sample(Scalar elapsed) const {
    ScalarMotionSample out;
    if (!valid_ || !is_finite_scalar(elapsed)) {
      return out;
    }

    out.valid = true;
    if (duration_ <= epsilon_) {
      out.position = distance_;
      out.finished = true;
      return out;
    }

    const Scalar t = clamp_scalar(elapsed, 0.0f, duration_);
    out.finished = (elapsed >= duration_);

    Scalar pos_abs = abs_distance_;
    Scalar vel_abs = 0.0f;
    Scalar acc_abs = 0.0f;

    if (abs_distance_ <= epsilon_ || acceleration_ == 0) {
      pos_abs = abs_distance_;
    } else if (t <= accel_time_) {
      pos_abs = 0.5f * acceleration_ * t * t;
      vel_abs = acceleration_ * t;
      acc_abs = acceleration_;
    } else if (t <= accel_time_ + cruise_time_) {
      const Scalar cruise_t = t - accel_time_;
      pos_abs = 0.5f * acceleration_ * accel_time_ * accel_time_ +
                peak_velocity_ * cruise_t;
      vel_abs = peak_velocity_;
    } else if (t < duration_) {
      const Scalar decel_t = t - accel_time_ - cruise_time_;
      const Scalar before_decel =
          0.5f * acceleration_ * accel_time_ * accel_time_ +
          peak_velocity_ * cruise_time_;
      pos_abs = before_decel + peak_velocity_ * decel_t -
                0.5f * acceleration_ * decel_t * decel_t;
      vel_abs = peak_velocity_ - acceleration_ * decel_t;
      acc_abs = -acceleration_;
    }

    if (out.finished) {
      out.position = distance_;
      out.velocity = 0.0f;
      out.acceleration = 0.0f;
      return out;
    }

    out.position = direction_ * pos_abs;
    out.velocity = direction_ * vel_abs;
    out.acceleration = direction_ * acc_abs;
    return out;
  }

  Scalar Duration() const { return duration_; }
  Scalar Distance() const { return distance_; }
  Scalar PeakVelocity() const { return peak_velocity_; }
  Scalar Acceleration() const { return acceleration_; }
  Scalar AccelerationTime() const { return accel_time_; }
  Scalar CruiseTime() const { return cruise_time_; }
  bool IsValid() const { return valid_; }

 private:
  void reset() {
    distance_ = 0.0f;
    direction_ = 1.0f;
    abs_distance_ = 0.0f;
    peak_velocity_ = 0.0f;
    acceleration_ = 0.0f;
    accel_time_ = 0.0f;
    cruise_time_ = 0.0f;
    duration_ = 0.0f;
    epsilon_ = kDefaultEpsilon;
    valid_ = false;
  }

  Scalar distance_;
  Scalar direction_;
  Scalar abs_distance_;
  Scalar peak_velocity_;
  Scalar acceleration_;
  Scalar accel_time_;
  Scalar cruise_time_;
  Scalar duration_;
  Scalar epsilon_;
  bool valid_;
};

}  // namespace mr::comp::traj
