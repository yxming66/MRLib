/** @file scalar.hpp
 *  C++ entry points for the shared C scalar core.
 *  Original scalar utilities: XuMing Yuan.
 */
#pragma once

#include <cmath>
#include "component/math/scalar.h"

namespace mr::component::math {

using Scalar = float;
constexpr Scalar kDefaultEpsilon = MR_MATH_DEFAULT_EPSILON;
constexpr Scalar kFiniteLimit = MR_MATH_FINITE_LIMIT;
constexpr Scalar kPi = MR_MATH_PI;
constexpr Scalar kHalfPi = MR_MATH_HALF_PI;
constexpr Scalar kTwoPi = MR_MATH_TWO_PI;

inline Scalar abs_scalar(Scalar value) {
  return mr_math_abs_scalar(value);
}

inline bool is_finite_scalar(Scalar value) {
  return mr_math_is_finite_scalar(value);
}

inline bool is_positive_scalar(Scalar value,
                               Scalar epsilon = kDefaultEpsilon) {
  return mr_math_is_positive_scalar(value, epsilon);
}

inline Scalar clamp_scalar(Scalar value, Scalar lower, Scalar upper) {
  return mr_math_clamp_scalar(value, lower, upper);
}

inline Scalar abs_clip_scalar(Scalar value, Scalar limit) {
  return mr_math_abs_clip_scalar(value, limit);
}

inline Scalar positive_or_zero(Scalar value) {
  return mr_math_positive_or_zero(value);
}

inline Scalar positive_or(Scalar value, Scalar fallback) {
  return mr_math_positive_or(value, fallback);
}

inline Scalar square_scalar(Scalar value) {
  return mr_math_square_scalar(value);
}

inline Scalar uint_to_float(uint32_t raw,
                            Scalar min_value,
                            Scalar max_value,
                            uint8_t bits) {
  return mr_math_uint_to_float(raw, min_value, max_value, bits);
}

inline Scalar sanitize_positive(Scalar value,
                                Scalar fallback,
                                Scalar lower = kDefaultEpsilon,
                                Scalar upper = kFiniteLimit) {
  return mr_math_sanitize_positive(value, fallback, lower, upper);
}

inline Scalar sanitize_dt(Scalar dt,
                          Scalar fallback,
                          Scalar min_dt,
                          Scalar max_dt) {
  return mr_math_sanitize_dt(dt, fallback, min_dt, max_dt);
}

inline Scalar wrap_to_range(Scalar value,
                            Scalar lower,
                            Scalar upper,
                            Scalar epsilon = kDefaultEpsilon) {
  return mr_math_wrap_to_range(value, lower, upper, epsilon);
}

inline Scalar wrap_to_pi(Scalar angle) {
  return mr_math_wrap_to_pi(angle);
}

inline Scalar wrap_error(Scalar error, Scalar range) {
  return mr_math_wrap_error(error, range);
}

inline Scalar angle_error(Scalar target, Scalar current) {
  return mr_math_angle_error(target, current);
}

inline Scalar move_towards(Scalar current, Scalar target, Scalar max_step) {
  return mr_math_move_towards(current, target, max_step);
}

inline Scalar apply_delta_limit(Scalar value,
                                Scalar previous,
                                Scalar max_delta) {
  return mr_math_apply_delta_limit(value, previous, max_delta);
}

inline Scalar max_abs_array(const Scalar* values, uint32_t count) {
  return mr_math_max_abs_array(values, count);
}

inline bool values_are_finite(const Scalar* values, uint32_t count) {
  return mr_math_values_are_finite(values, count);
}

inline bool scale_to_abs_limit(Scalar* values,
                               uint32_t count,
                               Scalar limit) {
  return mr_math_scale_to_abs_limit(values, count, limit);
}

}  // namespace mr::component::math
