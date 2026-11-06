#pragma once

/* One coefficient/state implementation for the C and C++ filter APIs.
 * Configure changes coefficients only. Reset seeds a constant output; Clear
 * leaves the first valid sample to seed the state (no startup transient).
 */
#include "component/math/scalar.h"

typedef enum {
  MR_FILTER_INVALID = -1,
  MR_FILTER_DISABLED = 0,
  MR_FILTER_ENABLED = 1
} MrFilterStatus;

typedef struct {
  float b0, b1, b2, a1, a2;
  bool bypass;
} MrBiquadCoefficients;

typedef struct {
  float delay_1, delay_2, output;
  bool initialized;
} MrBiquadState;

static inline MrBiquadCoefficients MrBiquad_Bypass(void) {
  const MrBiquadCoefficients result = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, true};
  return result;
}

static inline MrFilterStatus MrBiquad_LowPass(MrBiquadCoefficients *coeff,
                                             float sample_hz, float cutoff_hz) {
  if (coeff == NULL) return MR_FILTER_INVALID;
  *coeff = MrBiquad_Bypass();
  if (!isfinite(sample_hz) || sample_hz <= 0.0f || !isfinite(cutoff_hz))
    return MR_FILTER_INVALID;
  if (cutoff_hz <= 0.0f) return MR_FILTER_DISABLED;
  if (cutoff_hz >= sample_hz * 0.5f) return MR_FILTER_INVALID;
  const float omega = tanf(MR_MATH_PI * cutoff_hz / sample_hz);
  const float squared = omega * omega;
  const float sqrt2 = 1.41421356237309504880f;
  const float divisor = 1.0f + sqrt2 * omega + squared;
  if (!isfinite(omega) || !isfinite(divisor) || divisor <= 0.0f)
    return MR_FILTER_INVALID;
  coeff->b0 = squared / divisor;
  coeff->b1 = 2.0f * coeff->b0;
  coeff->b2 = coeff->b0;
  coeff->a1 = 2.0f * (squared - 1.0f) / divisor;
  coeff->a2 = (1.0f - sqrt2 * omega + squared) / divisor;
  coeff->bypass = false;
  return MR_FILTER_ENABLED;
}

static inline MrFilterStatus MrBiquad_Notch(MrBiquadCoefficients *coeff,
                                           float sample_hz, float notch_hz,
                                           float bandwidth_hz) {
  if (coeff == NULL) return MR_FILTER_INVALID;
  *coeff = MrBiquad_Bypass();
  if (!isfinite(sample_hz) || sample_hz <= 0.0f ||
      !isfinite(notch_hz) || !isfinite(bandwidth_hz)) return MR_FILTER_INVALID;
  if (notch_hz <= 0.0f || bandwidth_hz <= 0.0f) return MR_FILTER_DISABLED;
  if (notch_hz >= sample_hz * 0.5f || bandwidth_hz >= sample_hz * 0.5f)
    return MR_FILTER_INVALID;
  const float alpha = tanf(MR_MATH_PI * bandwidth_hz / sample_hz);
  const float beta = -cosf(MR_MATH_TWO_PI * notch_hz / sample_hz);
  if (!isfinite(alpha) || !isfinite(beta) || alpha < 0.0f)
    return MR_FILTER_INVALID;
  const float inverse = 1.0f / (alpha + 1.0f);
  coeff->b0 = inverse;
  coeff->b1 = 2.0f * beta * inverse;
  coeff->b2 = inverse;
  coeff->a1 = coeff->b1;
  coeff->a2 = (1.0f - alpha) * inverse;
  coeff->bypass = false;
  return MR_FILTER_ENABLED;
}

static inline void MrBiquad_Clear(MrBiquadState *state) {
  if (state == NULL) return;
  const MrBiquadState empty = {0.0f, 0.0f, 0.0f, false};
  *state = empty;
}

static inline void MrBiquad_Reset(MrBiquadState *state,
                                   const MrBiquadCoefficients *coeff,
                                   float output) {
  if (state == NULL || coeff == NULL || !mr_math_is_finite_scalar(output)) return;
  const float gain = coeff->b0 + coeff->b1 + coeff->b2;
  float delay = output;
  if (fabsf(gain) > MR_MATH_DEFAULT_EPSILON) delay = output / gain;
  if (!mr_math_is_finite_scalar(delay)) delay = output;
  state->delay_1 = delay;
  state->delay_2 = delay;
  state->output = output;
  state->initialized = true;
}

/* Checked entry point lets a controller reject the whole step on filter error.
 * The float API below retains its hold-last-output behavior.
 */
static inline bool MrBiquad_TryUpdate(MrBiquadState *state,
                                      const MrBiquadCoefficients *coeff,
                                      float sample) {
  if (state == NULL || coeff == NULL || !mr_math_is_finite_scalar(sample)) return false;
  if (coeff->bypass || !state->initialized) {
    MrBiquad_Reset(state, coeff, sample);
    return true;
  }
  const float delay = sample - state->delay_1 * coeff->a1 - state->delay_2 * coeff->a2;
  const float output = delay * coeff->b0 + state->delay_1 * coeff->b1 +
                       state->delay_2 * coeff->b2;
  if (!mr_math_is_finite_scalar(delay) || !mr_math_is_finite_scalar(output))
    return false;
  state->delay_2 = state->delay_1;
  state->delay_1 = delay;
  state->output = output;
  return true;
}
static inline float MrBiquad_Update(MrBiquadState *state,
                                     const MrBiquadCoefficients *coeff,
                                     float sample) {
  MrBiquad_TryUpdate(state, coeff, sample);
  return state ? state->output : 0.0f;
}
