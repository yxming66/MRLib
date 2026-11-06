#pragma once

/* Shared float operations for C11 and C++17. No platform dependencies.
 * clamp requires ordered bounds. Invalid wrap ranges leave the value unchanged.
 * wrap_to_pi uses [-pi, pi); wrap_error preserves the sign at half a period.
 * is_finite_scalar also enforces the library control magnitude limit (1e30).
 */
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MR_MATH_DEFAULT_EPSILON (1.0e-6f)
#define MR_MATH_FINITE_LIMIT (1.0e30f)
#define MR_MATH_PI (3.14159265358979323846f)
#define MR_MATH_HALF_PI (0.5f * MR_MATH_PI)
#define MR_MATH_TWO_PI (2.0f * MR_MATH_PI)
#define MR_MATH_DEG_TO_RAD (MR_MATH_PI / 180.0f)
#define MR_MATH_RAD_TO_DEG (180.0f / MR_MATH_PI)

static inline float mr_math_abs_scalar(float value) {
  return fabsf(value);
}

static inline bool mr_math_is_finite_scalar(float value) {
  return (value == value) && (mr_math_abs_scalar(value) <= MR_MATH_FINITE_LIMIT);
}

static inline bool mr_math_is_positive_scalar(float value,
                               float epsilon) {
  return mr_math_is_finite_scalar(value) && value > epsilon;
}

static inline float mr_math_clamp_scalar(float value, float lower, float upper) {
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

static inline float mr_math_abs_clip_scalar(float value, float limit) {
  const float positive_limit = mr_math_abs_scalar(limit);
  return mr_math_clamp_scalar(value, -positive_limit, positive_limit);
}

static inline float mr_math_positive_or_zero(float value) {
  return (mr_math_is_finite_scalar(value) && value > 0.0f) ? value : 0.0f;
}

static inline float mr_math_positive_or(float value, float fallback) {
  return (mr_math_is_finite_scalar(value) && value > 0.0f) ? value : fallback;
}

static inline float mr_math_square_scalar(float value) {
  return value * value;
}

static inline float mr_math_uint_to_float(uint32_t raw,
                            float min_value,
                            float max_value,
                            uint8_t bits) {
  if (bits == 0U || bits >= 32U) {
    return min_value;
  }
  const float span = max_value - min_value;
  const uint32_t raw_max = (1UL << bits) - 1UL;
  const uint32_t bounded_raw = raw > raw_max ? raw_max : raw;
  return (((float)bounded_raw) * span / ((float)raw_max)) +
         min_value;
}

static inline float mr_math_sanitize_positive(float value,
                                float fallback,
                                float lower,
                                float upper) {
  if (!mr_math_is_finite_scalar(lower) || lower <= 0.0f) {
    lower = MR_MATH_DEFAULT_EPSILON;
  }
  if (lower > MR_MATH_FINITE_LIMIT) {
    lower = MR_MATH_FINITE_LIMIT;
  }
  if (!mr_math_is_finite_scalar(upper) || upper < lower) {
    upper = MR_MATH_FINITE_LIMIT;
  }
  if (upper < lower) {
    upper = lower;
  }
  if (!mr_math_is_finite_scalar(value) || value < lower) {
    value = fallback;
  }
  if (!mr_math_is_finite_scalar(value) || value < lower) {
    value = lower;
  }
  return mr_math_clamp_scalar(value, lower, upper);
}

static inline float mr_math_sanitize_dt(float dt,
                          float fallback,
                          float min_dt,
                          float max_dt) {
  if (!mr_math_is_finite_scalar(min_dt) || min_dt <= 0.0f) {
    min_dt = MR_MATH_DEFAULT_EPSILON;
  }
  if (!mr_math_is_finite_scalar(max_dt) || max_dt < min_dt) {
    max_dt = MR_MATH_FINITE_LIMIT;
  }
  if (!mr_math_is_finite_scalar(fallback) || fallback < min_dt) {
    fallback = min_dt;
  }
  if (!mr_math_is_finite_scalar(dt) || dt <= 0.0f) {
    dt = fallback;
  }
  return mr_math_clamp_scalar(dt, min_dt, max_dt);
}

static inline float mr_math_wrap_to_range(float value,
                            float lower,
                            float upper,
                            float epsilon) {
  const float width = upper - lower;
  if (!mr_math_is_finite_scalar(value) || !mr_math_is_finite_scalar(lower) ||
      !mr_math_is_finite_scalar(upper) || width <= epsilon) {
    return value;
  }

  float wrapped = fmodf(value - lower, width);
  if (wrapped < 0.0f) {
    wrapped += width;
  }
  return lower + wrapped;
}

static inline float mr_math_wrap_to_pi(float angle) {
  float wrapped =
      mr_math_wrap_to_range(angle, -MR_MATH_PI, MR_MATH_PI, MR_MATH_DEFAULT_EPSILON);
  if (wrapped >= MR_MATH_PI) {
    wrapped -= MR_MATH_TWO_PI;
  }
  return wrapped;
}

static inline float mr_math_wrap_error(float error, float range) {
  if (!mr_math_is_finite_scalar(error) || !mr_math_is_finite_scalar(range) ||
      range <= MR_MATH_DEFAULT_EPSILON) {
    return error;
  }

  const float half_range = 0.5f * range;
  if (error >= -half_range && error <= half_range) {
    return error;
  }

  float wrapped = mr_math_wrap_to_range(error + half_range, 0.0f, range,
                                       MR_MATH_DEFAULT_EPSILON);
  wrapped -= half_range;
  return (wrapped == -half_range && error > 0.0f) ? half_range : wrapped;
}

static inline float mr_math_angle_error(float target, float current) {
  return mr_math_wrap_to_pi(target - current);
}

static inline float mr_math_move_towards(float current, float target, float max_step) {
  if (max_step <= 0.0f) {
    return target;
  }

  const float delta = target - current;
  if (delta > max_step) {
    return current + max_step;
  }
  if (delta < -max_step) {
    return current - max_step;
  }
  return target;
}

static inline float mr_math_apply_delta_limit(float value,
                                float previous,
                                float max_delta) {
  return previous + mr_math_abs_clip_scalar(value - previous, max_delta);
}

static inline float mr_math_max_abs_array(const float* values, uint32_t count) {
  if (values == NULL) {
    return 0.0f;
  }

  float max_abs = 0.0f;
  for (uint32_t i = 0U; i < count; ++i) {
    const float abs_value = mr_math_abs_scalar(values[i]);
    if (abs_value > max_abs) {
      max_abs = abs_value;
    }
  }
  return max_abs;
}

static inline bool mr_math_values_are_finite(const float* values, uint32_t count) {
  if (values == NULL) {
    return false;
  }

  for (uint32_t i = 0U; i < count; ++i) {
    if (!mr_math_is_finite_scalar(values[i])) {
      return false;
    }
  }
  return true;
}

static inline bool mr_math_scale_to_abs_limit(float* values,
                               uint32_t count,
                               float limit) {
  if (values == NULL || !mr_math_is_finite_scalar(limit) || limit <= 0.0f) {
    return false;
  }

  const float max_abs = mr_math_max_abs_array(values, count);
  if (max_abs <= limit || max_abs <= 0.0f) {
    return false;
  }

  const float scale = limit / max_abs;
  for (uint32_t i = 0U; i < count; ++i) {
    values[i] *= scale;
  }
  return true;
}

/* Normalization scale: zero/invalid squared norms return zero, avoiding
 * 0 * infinity when a correction vector is already zero.
 */
static inline float mr_math_normalization_factor(float squared_norm) {
  return isfinite(squared_norm) && squared_norm > 0.0f
             ? 1.0f / sqrtf(squared_norm) : 0.0f;
}
