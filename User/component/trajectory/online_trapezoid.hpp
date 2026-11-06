#pragma once

// 在线梯形速度规划器：目标位置可在运行中改变，并自动限制速度和加速度。
// 使用方式：配置 OnlineTrapezoidAxisConfig，构造 OnlineTrapezoidAxis，
// 每周期调用 Update(current, target, dt)，读取返回的 position/velocity/valid/reached。

#include <cmath>

#include "component/trajectory/trajectory_types.hpp"

namespace mr::comp::traj {

struct OnlineTrapezoidAxisConfig {
  Scalar max_velocity = 0.0f;
  Scalar max_acceleration = 0.0f;
  Scalar position_epsilon = 0.002f;
};

struct OnlineTrapezoidAxisSample {
  Scalar position = 0.0f;
  Scalar velocity = 0.0f;
  bool valid = false;
  bool reached = false;
};

inline OnlineTrapezoidAxisSample update_trapezoid_axis(Scalar current, Scalar target,
                                                       Scalar* velocity, Scalar max_velocity,
                                                       Scalar max_acceleration, Scalar dt,
                                                       Scalar position_epsilon = 0.002f) {
  OnlineTrapezoidAxisSample out{};
  out.position = current;
  if (velocity == nullptr || !is_finite_scalar(current) || !is_finite_scalar(target)) {
    return out;
  }

  out.velocity = *velocity;
  if (!is_finite_scalar(*velocity) || !is_finite_scalar(max_velocity) ||
      !is_finite_scalar(max_acceleration) || !is_finite_scalar(dt) || dt <= 0 ||
      !is_finite_scalar(position_epsilon) || position_epsilon < 0 || max_velocity <= 0 ||
      max_acceleration <= 0 || abs_scalar(*velocity) > max_velocity)
    return out;

  const double delta = static_cast<double>(max_acceleration) * dt;
  const double error = static_cast<double>(target) - current;
  // Discrete braking bound for semi-implicit integration. Crossing the target never
  // resets a moving axis: an infeasible target change is followed by bounded braking.
  const double adt = delta;
  const double speed = 2 * max_acceleration * std::fabs(error) /
                       (std::sqrt(adt * adt + 2 * max_acceleration * std::fabs(error)) + adt);
  const double desired = std::copysign(std::fmin(max_velocity, speed), error);
  double next_v = *velocity + std::fmax(-delta, std::fmin(delta, desired - *velocity));
  // Rounding the velocity to float must not exceed the acceleration budget.
  float quantized_v = static_cast<float>(next_v);
  if (std::fabs(static_cast<double>(quantized_v) - *velocity) > delta)
    quantized_v = std::nextafter(quantized_v, *velocity);
  next_v = quantized_v;
  double next_p = current + next_v * dt;
  // Position snapping is confined to the configured tolerance; velocity can only
  // reach zero when the one-step acceleration bound permits it.
  if (std::fabs(error) <= position_epsilon && std::fabs(*velocity) <= delta) {
    next_v = 0;
    next_p = target;
    out.reached = true;
  }
  out.position = static_cast<Scalar>(next_p);
  out.velocity = static_cast<Scalar>(next_v);
  out.valid = is_finite_scalar(out.position) && is_finite_scalar(out.velocity);
  if (out.valid) *velocity = out.velocity;

  return out;
}

class OnlineTrapezoidAxis {
 public:
  OnlineTrapezoidAxis() = default;

  explicit OnlineTrapezoidAxis(const OnlineTrapezoidAxisConfig& config) : config_(config) {}

  void Configure(const OnlineTrapezoidAxisConfig& config) { config_ = config; }

  void Reset(Scalar velocity = 0.0f) { velocity_ = velocity; }

  OnlineTrapezoidAxisSample Update(Scalar current, Scalar target, Scalar dt) {
    return update_trapezoid_axis(current, target, &velocity_, config_.max_velocity,
                                 config_.max_acceleration, dt, config_.position_epsilon);
  }

  Scalar velocity() const { return velocity_; }
  const OnlineTrapezoidAxisConfig& config() const { return config_; }

 private:
  OnlineTrapezoidAxisConfig config_{};
  Scalar velocity_ = 0.0f;
};

}  // namespace mr::comp::traj
