#pragma once

// 多轴同步轨迹包装器。使用方式：为每个轴准备同类单轴 profile，调用同步器
// Configure 后，在统一时间基准下调用 SampleAxis/Sample 获取各轴结果。

/*
 * 多轴同步轨迹包装器。
 *
 * ScalarProfile 必须提供以下单轴接口：
 *   using Config = ...;
 *   static Scalar MinDuration(const Config& config);
 *   bool ConfigureForDuration(const Config& config, Scalar duration);
 *   ScalarMotionSample Sample(Scalar elapsed) const;
 *
 * 同步器本身不缩放已有轨迹。它先取每轴最短可行时间的最大值，再要求每个
 * 单轴规划器以该统一时长重新构建轨迹。因此梯形、S 曲线或满足同一接口的
 * 其他单轴规划器均可复用这套同步逻辑。
 */

#include <array>

#include "trajectory_types.hpp"

namespace mr::comp::traj {

template <int N, typename ScalarProfile>
class SynchronizedProfile {
 public:
  static_assert(N > 0, "SynchronizedProfile requires at least one axis");
  using AxisConfig = typename ScalarProfile::Config;

  SynchronizedProfile() : duration_(0.0f), valid_(false) {}

  bool Configure(const std::array<AxisConfig, N>& axis_configs) {
    valid_ = false;
    duration_ = 0.0f;

    for (uint16_t i = 0; i < static_cast<uint16_t>(N); ++i) {
      const Scalar min_time = ScalarProfile::MinDuration(axis_configs[i]);
      if (!is_finite_scalar(min_time) || min_time < 0.0f) {
        return false;
      }
      if (min_time > duration_) {
        duration_ = min_time;
      }
    }

    for (uint16_t i = 0; i < static_cast<uint16_t>(N); ++i) {
      if (!profiles_[i].ConfigureForDuration(axis_configs[i], duration_)) {
        valid_ = false;
        duration_ = 0.0f;
        return false;
      }
    }

    valid_ = true;
    return true;
  }

  ScalarMotionSample SampleAxis(uint16_t index, Scalar elapsed) const {
    if (!valid_ || index >= static_cast<uint16_t>(N)) {
      return ScalarMotionSample();
    }
    return profiles_[index].Sample(elapsed);
  }

  const ScalarProfile* Axis(uint16_t index) const {
    return valid_ && index < N ? &profiles_[index] : nullptr;
  }

  Scalar Duration() const { return duration_; }
  bool IsValid() const { return valid_; }

 private:
  ScalarProfile profiles_[N];
  Scalar duration_;
  bool valid_;
};

}  // namespace mr::comp::traj
