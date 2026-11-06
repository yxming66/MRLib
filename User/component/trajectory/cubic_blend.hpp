#pragma once

/*
 * 三次混合曲线规划。
 *
 * 控制律（三次Hermite插值）：
 *   u = elapsed / duration  (归一化时间，范围 [0, 1])
 *   s = 3*u^2 - 2*u^3     (位置，三次S曲线)
 *   ds = 6*u*(1-u)/T       (速度)
 *   dds = 6*(1-2*u)/T^2    (加速度)
 *
 * 特性：
 *   - 返回归一化进度 s ∈ [0, 1]，乘以目标距离得到实际位置
 *   - 端点速度为零；区间内端点加速度为 ±6/T²，不能用于要求端点零加速度的拼接
 *   - 适用于笛卡尔空间直线/旋转轨迹的平滑插值
 *
 * 使用示例：
 *   float duration = mr::comp::traj::cubic_blend_duration_from_limits(
 *       distance, max_velocity, max_acceleration);
 *
 *   float elapsed = get_current_time() - start_time;
 *   auto sample = mr::comp::traj::sample_cubic_blend(elapsed, duration);
 *   float position = sample.s * total_distance;  // 实际位置
 *   float velocity = sample.ds * total_distance;   // 实际速度
 */

#include <math.h>

#include "trajectory_types.hpp"

namespace mr::comp::traj {

struct CubicBlendSample {
  Scalar s;
  Scalar ds;
  Scalar dds;
  bool finished;
  bool valid = false;

  CubicBlendSample() : s(0.0f), ds(0.0f), dds(0.0f), finished(false) {}
};

inline CubicBlendSample sample_cubic_blend(Scalar elapsed, Scalar duration,
                                           Scalar epsilon = kDefaultEpsilon) {
  CubicBlendSample sample;
  if (!is_finite_scalar(elapsed) || !is_finite_scalar(duration) || duration < 0 ||
      !is_finite_scalar(epsilon) || epsilon <= 0)
    return sample;
  sample.valid = true;
  if (duration <= epsilon) {
    sample.s = (elapsed >= 0.0f) ? 1.0f : 0.0f;
    sample.finished = (elapsed >= 0.0f);
    return sample;
  }

  const Scalar u = clamp_scalar(elapsed / duration, 0.0f, 1.0f);
  const Scalar uu = u * u;
  const Scalar inv_duration = 1.0f / duration;
  sample.s = (3.0f * uu) - (2.0f * uu * u);
  sample.ds = (6.0f * u * (1.0f - u)) * inv_duration;
  sample.dds = (6.0f - 12.0f * u) * inv_duration * inv_duration;
  sample.finished = (elapsed >= duration);

  if (sample.finished) {
    sample.s = 1.0f;
    sample.ds = 0.0f;
    sample.dds = 0.0f;
  } else if (elapsed <= 0.0f) {
    sample.s = 0.0f;
    sample.ds = 0.0f;
    sample.dds = 0.0f;
  }

  return sample;
}

inline Scalar cubic_blend_duration_from_limits(Scalar distance, Scalar max_velocity,
                                               Scalar max_acceleration,
                                               Scalar epsilon = kDefaultEpsilon) {
  if (!is_finite_scalar(epsilon) || epsilon <= 0 || !is_finite_scalar(distance) ||
      !is_finite_scalar(max_velocity) || !is_finite_scalar(max_acceleration) || max_velocity < 0 ||
      max_acceleration < 0)
    return -1;
  const Scalar abs_distance = abs_scalar(distance);
  if (abs_distance <= epsilon) {
    return 0.0f;
  }
  if (max_velocity <= epsilon || max_acceleration <= epsilon || !is_finite_scalar(abs_distance) ||
      !is_finite_scalar(max_velocity) || !is_finite_scalar(max_acceleration)) {
    return -1.0f;
  }

  const Scalar velocity_bound = 1.5f * abs_distance / max_velocity;
  const Scalar acceleration_bound = sqrtf(6.0f * abs_distance / max_acceleration);
  const Scalar result = (velocity_bound > acceleration_bound) ? velocity_bound : acceleration_bound;
  return is_finite_scalar(result) ? result : -1;
}

}  // namespace mr::comp::traj
