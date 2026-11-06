#pragma once

// Symmetric rest-to-rest seven-phase jerk-limited trajectory.
// Synchronization stretches time: velocity, acceleration and jerk scale as 1/s, 1/s², 1/s³.
#include <cmath>
#include "trajectory_types.hpp"
#include "synchronized_profile.hpp"

namespace mr::comp::traj {
struct SCurveProfileConfig {
  Scalar distance = 0;
  Scalar max_velocity = 0;
  Scalar max_acceleration = 0;
  Scalar max_jerk = 0;
  Scalar epsilon = kDefaultEpsilon;
};

class SCurveProfile {
 public:
  using Config = SCurveProfileConfig;
  static Scalar MinDuration(const Config& config) {
    SCurveProfile p;
    return p.Configure(config) ? p.Duration() : -1;
  }
  bool Configure(const Config& c) {
    *this = SCurveProfile{};
    if (!is_finite_scalar(c.distance) || !is_finite_scalar(c.epsilon) || c.epsilon <= 0 ||
        !is_finite_scalar(c.max_velocity) || !is_finite_scalar(c.max_acceleration) ||
        !is_finite_scalar(c.max_jerk) || c.max_velocity < 0 || c.max_acceleration < 0 ||
        c.max_jerk < 0) return false;
    distance_ = c.distance;
    const double d = std::fabs(static_cast<double>(c.distance));
    if (d <= c.epsilon) { valid_ = true; return true; }
    if (c.max_velocity <= 0 || c.max_acceleration <= 0 || c.max_jerk <= 0) return false;
    const double v = c.max_velocity, a = c.max_acceleration, j = c.max_jerk;
    double tj = std::fmin(a / j, std::sqrt(v / j));
    double ta = std::fmax(0.0, v / (j * tj) - tj);
    double tc = 0;
    const double ramp_distance = v * (2 * tj + ta);
    if (d >= ramp_distance) {
      tc = (d - ramp_distance) / v;
    } else if (d >= 2 * a * (a / j) * (a / j)) {
      tj = a / j;
      ta = 0.5 * (std::sqrt(tj * tj + 4 * d / a) - 3 * tj);
    } else {
      tj = std::cbrt(d / (2 * j));
      ta = 0;
    }
    times_[0] = times_[2] = times_[4] = times_[6] = tj;
    times_[1] = times_[5] = std::fmax(0.0, ta);
    times_[3] = tc;
    base_duration_ = 4 * tj + 2 * times_[1] + tc;
    duration_ = static_cast<Scalar>(base_duration_);
    if (duration_ < base_duration_) duration_ = std::nextafter(duration_, INFINITY);
    jerk_ = j;
    valid_ = std::isfinite(base_duration_) && is_finite_scalar(duration_) && duration_ > 0;
    return valid_;
  }
  bool ConfigureForDuration(const Config& config, Scalar duration) {
    if (!Configure(config)) return false;
    if (!is_finite_scalar(duration) || duration < duration_) { valid_ = false; return false; }
    duration_ = duration;
    return true;
  }
  ScalarMotionSample Sample(Scalar elapsed) const {
    ScalarMotionSample out;
    if (!valid_ || !is_finite_scalar(elapsed)) return out;
    out.valid = true;
    out.finished = elapsed >= duration_;
    if (out.finished || base_duration_ == 0) { out.position = distance_; return out; }
    if (elapsed <= 0) return out;
    const double scale = static_cast<double>(duration_) / base_duration_;
    double remaining = elapsed / scale;
    double p = 0, v = 0, a = 0;
    constexpr int signs[] = {1, 0, -1, 0, -1, 0, 1};
    for (int i = 0; i < 7 && remaining > 0; ++i) {
      const double t = std::fmin(remaining, times_[i]);
      const double j = signs[i] * jerk_;
      p += v*t + a*t*t/2 + j*t*t*t/6;
      v += a*t + j*t*t/2;
      a += j*t;
      remaining -= t;
    }
    const double direction = distance_ >= 0 ? 1 : -1;
    out.position = static_cast<Scalar>(direction * p);
    out.velocity = static_cast<Scalar>(direction * v / scale);
    out.acceleration = static_cast<Scalar>(direction * a / (scale*scale));
    out.valid = is_finite_scalar(out.position) && is_finite_scalar(out.velocity) &&
                is_finite_scalar(out.acceleration);
    return out;
  }
  Scalar Duration() const { return duration_; }
  bool IsValid() const { return valid_; }
  // Compatibility entry points; same implementation and contracts.
  bool configure(Scalar d, Scalar v, Scalar a, Scalar j, Scalar epsilon = kDefaultEpsilon) {
    return Configure({d, v, a, j, epsilon});
  }
  ScalarMotionSample sample(Scalar elapsed) const { return Sample(elapsed); }
  Scalar duration() const { return Duration(); }
  bool valid() const { return IsValid(); }
 private:
  double times_[7]{};
  double base_duration_ = 0;
  double jerk_ = 0;
  Scalar duration_ = 0;
  Scalar distance_ = 0;
  bool valid_ = false;
};

template <int N>
class SynchronizedSCurveProfile {
 public:
  bool configure(const Scalar* d, const Scalar* v, const Scalar* a, const Scalar* j,
                 Scalar epsilon = kDefaultEpsilon) {
    sync_ = {};
    if (!d || !v || !a || !j) return false;
    std::array<SCurveProfileConfig, N> configs{};
    for (int i = 0; i < N; ++i) configs[i] = {d[i], v[i], a[i], j[i], epsilon};
    return sync_.Configure(configs);
  }
  ScalarMotionSample sample_axis(uint16_t i, Scalar t) const { return sync_.SampleAxis(i, t); }
  Scalar duration() const { return sync_.Duration(); }
  bool valid() const { return sync_.IsValid(); }
 private:
  SynchronizedProfile<N, SCurveProfile> sync_;
};
}  // namespace mr::comp::traj
