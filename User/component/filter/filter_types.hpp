#pragma once

// 滤波器公共类型。通常由具体滤波器头文件间接包含；使用者通过具体滤波器的
// Build/Update 接口使用，不需要直接操作内部 biquad 状态。

/*
 * 滤波器组件共用类型和工具函数。
 *
 * 业务代码通常包含具体滤波器头文件或聚合头文件即可：
 *
 *   #include "component/filter/filter.hpp"
 *
 * 本文件保存通用标量别名、采样周期清洗工具，以及 low_pass 和 notch 共用的
 * 内部 biquad 状态。
 */

#include <stdint.h>

#include "component/math/scalar.hpp"
#include "component/filter/biquad.h"

namespace mr::comp::filter {

using Scalar = mr::component::math::Scalar;

constexpr Scalar kDefaultSampleFreqHz = 1000.0f;
constexpr Scalar kDefaultDtS = 1.0f / kDefaultSampleFreqHz;
constexpr Scalar kMinDtS = 1.0e-6f;
constexpr Scalar kMaxDtS = 1.0f;

inline bool is_usable_scalar(Scalar value) {
  return mr::component::math::is_finite_scalar(value);
}

inline Scalar sanitize_dt(Scalar dt_s, Scalar fallback_dt_s = kDefaultDtS) {
  const Scalar fallback = mr::component::math::sanitize_positive(
      fallback_dt_s, kDefaultDtS, kMinDtS, kMaxDtS);
  if (!is_usable_scalar(dt_s) || dt_s <= 0.0f) {
    dt_s = fallback;
  }
  return mr::component::math::clamp_scalar(dt_s, kMinDtS, kMaxDtS);
}

inline Scalar dt_from_sample_freq(Scalar sample_freq_hz,
                                  Scalar fallback_hz = kDefaultSampleFreqHz) {
  const Scalar freq = mr::component::math::sanitize_positive(
      sample_freq_hz, fallback_hz, mr::component::math::kDefaultEpsilon,
      mr::component::math::kFiniteLimit);
  const Scalar fallback_freq = mr::component::math::sanitize_positive(
      fallback_hz, kDefaultSampleFreqHz,
      mr::component::math::kDefaultEpsilon,
      mr::component::math::kFiniteLimit);
  return sanitize_dt(1.0f / freq, 1.0f / fallback_freq);
}

inline Scalar sample_freq_from_dt(Scalar dt_s,
                                  Scalar fallback_hz = kDefaultSampleFreqHz) {
  const Scalar fallback_dt = dt_from_sample_freq(fallback_hz);
  return 1.0f / sanitize_dt(dt_s, fallback_dt);
}

struct sample_config {
  Scalar sample_freq_hz = kDefaultSampleFreqHz;

  Scalar dt_s() const { return dt_from_sample_freq(sample_freq_hz); }
};

namespace detail {

inline bool enabled_frequency(Scalar frequency_hz) {
  return mr::component::math::is_positive_scalar(
      frequency_hz, mr::component::math::kDefaultEpsilon);
}

inline Scalar sanitize_sample_freq(Scalar sample_freq_hz) {
  return mr::component::math::sanitize_positive(
      sample_freq_hz, kDefaultSampleFreqHz,
      mr::component::math::kDefaultEpsilon,
      mr::component::math::kFiniteLimit);
}

inline Scalar nyquist_frequency(Scalar sample_freq_hz) {
  return 0.5f * sanitize_sample_freq(sample_freq_hz);
}

using biquad_coefficients = MrBiquadCoefficients;

class biquad_state {
 public:
  void Reset(Scalar output = 0.0f) {
    const auto bypass = MrBiquad_Bypass();
    MrBiquad_Reset(&state_, &bypass, output);
  }
  void Reset(const biquad_coefficients& coeff, Scalar output) {
    MrBiquad_Reset(&state_, &coeff, output);
  }
  Scalar Update(const biquad_coefficients& coeff, Scalar sample) {
    return MrBiquad_Update(&state_, &coeff, sample);
  }
  Scalar output() const { return state_.output; }
  bool initialized() const { return state_.initialized; }
 private:
  MrBiquadState state_{};
};

}  // namespace detail

}  // namespace mr::comp::filter
