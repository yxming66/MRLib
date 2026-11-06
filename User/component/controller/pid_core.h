#pragma once

/* Shared C11/C++17 PID implementation. No heap, platform calls or global state.
 * Limits are nonnegative: zero disables a limit. Integral limit is in OUTPUT
 * units. Explicit dt must be in [1 us, 1 s]; errors hold the last output and
 * do not advance the filter, integral or derivative history.
 * Derived from the former PX4-inspired component/pid.c implementation.
 */
#include "component/filter/biquad.h"

typedef enum {
  MR_PID_D_NONE = 0,
  MR_PID_D_MEASUREMENT = 1,
  MR_PID_D_EXTERNAL = 2
} MrPidDerivativeMode;

typedef struct { float k, p, i, d; } MrPidGains;
typedef struct {
  MrPidGains gains;
  float output_limit;
  float integral_output_limit;
  float output_rate_limit;
  float error_period;
  MrPidDerivativeMode derivative_mode;
  float derivative_cutoff_hz;
  float control_freq_hz;
  bool hold_integral_on_saturation;
} MrPidConfig;
typedef struct {
  float output, setpoint, feedback, error, feedforward, p, i, d;
  bool valid, saturated;
} MrPidStep;
typedef struct {
  MrPidConfig config;
  MrPidStep last;
  MrBiquadCoefficients coefficients;
  MrBiquadState filter;
  float integral_state, last_filtered_feedback, active_sample_freq_hz;
  bool configured, derivative_initialized;
} MrPid;

static inline MrPidConfig MrPid_DefaultConfig(void) {
  const MrPidConfig config = {{1, 0, 0, 0}, 0, 0, 0, 0,
                             MR_PID_D_MEASUREMENT, 0, 1000, true};
  return config;
}
static inline float MrPid_Limit(float value, float limit) {
  return limit > 0 ? mr_math_clamp_scalar(value, -limit, limit) : value;
}
static inline bool MrPid_ConfigValid(const MrPidConfig *config) {
  if (config == NULL) return false;
  const float fields[] = {config->gains.k, config->gains.p, config->gains.i,
      config->gains.d, config->output_limit, config->integral_output_limit,
      config->output_rate_limit, config->error_period, config->derivative_cutoff_hz,
      config->control_freq_hz};
  if (!mr_math_values_are_finite(fields, 10) || config->output_limit < 0 ||
      config->integral_output_limit < 0 || config->output_rate_limit < 0 ||
      config->error_period < 0 || config->control_freq_hz < 1 ||
      config->control_freq_hz > 1000000) return false;
  if (config->derivative_mode != MR_PID_D_NONE &&
      config->derivative_mode != MR_PID_D_MEASUREMENT &&
      config->derivative_mode != MR_PID_D_EXTERNAL) return false;
  MrBiquadCoefficients coefficients;
  return MrBiquad_LowPass(&coefficients, config->control_freq_hz,
                         config->derivative_cutoff_hz) != MR_FILTER_INVALID;
}
static inline bool MrPid_Reset(MrPid *pid, float output) {
  if (pid == NULL || !mr_math_is_finite_scalar(output)) return false;
  if (pid->configured) output = MrPid_Limit(output, pid->config.output_limit);
  const MrPidStep step = {output, 0, 0, 0, 0, 0, 0, 0, false, false};
  pid->last = step;
  pid->integral_state = 0;
  pid->last_filtered_feedback = 0;
  pid->active_sample_freq_hz = 0;
  pid->derivative_initialized = false;
  pid->coefficients = MrBiquad_Bypass();
  MrBiquad_Clear(&pid->filter);
  return true;
}
static inline bool MrPid_Configure(MrPid *pid, const MrPidConfig *config) {
  if (pid == NULL) return false;
  // Configuration failure invalidates this instance; no old control continues.
  const bool valid = MrPid_ConfigValid(config);
  const MrPidConfig next_config = valid ? *config : MrPid_DefaultConfig();
  pid->configured = false;
  pid->config = next_config;
  MrPid_Reset(pid, 0);
  pid->configured = valid;
  return valid;
}
static inline bool MrPid_ResetIntegral(MrPid *pid, float integral_output) {
  if (pid == NULL || !pid->configured || !mr_math_is_finite_scalar(integral_output))
    return false;
  const float output = MrPid_Limit(integral_output, pid->config.integral_output_limit);
  const float state = pid->config.gains.i != 0 ? output / pid->config.gains.i : 0;
  if (!mr_math_is_finite_scalar(state)) return false;
  pid->integral_state = state;
  return true;
}
static inline MrPidStep MrPid_FailedStep(MrPid *pid) {
  const MrPidStep empty = {0, 0, 0, 0, 0, 0, 0, 0, false, false};
  if (pid == NULL) return empty;
  pid->last.valid = false;
  return pid->last;
}
static inline MrPidStep MrPid_Update(MrPid *pid, float setpoint, float feedback,
                                     float feedback_dot, float dt_s) {
  if (pid == NULL || !pid->configured) return MrPid_FailedStep(pid);
  const float inputs[] = {setpoint, feedback, feedback_dot, dt_s};
  if (!mr_math_values_are_finite(inputs, 4) || dt_s < 1e-6f || dt_s > 1)
    return MrPid_FailedStep(pid);
  MrPid next = *pid; // Commit history only after the complete step succeeds.
  const MrPidConfig *config = &next.config;
  const float raw_error = setpoint - feedback;
  if (!mr_math_is_finite_scalar(raw_error)) return MrPid_FailedStep(pid);
  const float error = mr_math_wrap_error(raw_error, config->error_period);
  const float k_error = config->gains.k * error;
  const float k_feedback = config->gains.k * feedback;
  if (!mr_math_is_finite_scalar(k_error) || !mr_math_is_finite_scalar(k_feedback))
    return MrPid_FailedStep(pid);
  float derivative = 0;
  if (config->derivative_mode == MR_PID_D_MEASUREMENT) {
    const float sample_hz = 1.0f / dt_s;
    if (sample_hz != next.active_sample_freq_hz) {
      if (MrBiquad_LowPass(&next.coefficients, sample_hz,
                          config->derivative_cutoff_hz) == MR_FILTER_INVALID)
        return MrPid_FailedStep(pid);
      next.active_sample_freq_hz = sample_hz;
    }
    if (!next.derivative_initialized)
      MrBiquad_Reset(&next.filter, &next.coefficients, k_feedback);
    if (!MrBiquad_TryUpdate(&next.filter, &next.coefficients, k_feedback))
      return MrPid_FailedStep(pid);
    const float filtered = next.filter.output;
    if (next.derivative_initialized)
      derivative = (filtered - next.last_filtered_feedback) / dt_s;
    next.last_filtered_feedback = filtered;
    next.derivative_initialized = true;
  } else if (config->derivative_mode == MR_PID_D_EXTERNAL) {
    derivative = feedback_dot; // Already d(k * feedback)/dt; no second k scaling.
  }
  const float p = k_error * config->gains.p;
  const float d = -derivative * config->gains.d;
  const float pd = p + d;
  const float old_i = next.integral_state * config->gains.i;
  const float candidate_state = config->gains.i != 0
      ? next.integral_state + k_error * dt_s : 0;
  const float candidate_i = candidate_state * config->gains.i;
  const float terms[] = {derivative, p, d, pd, old_i, candidate_state, candidate_i};
  if (!mr_math_values_are_finite(terms, 7)) return MrPid_FailedStep(pid);
  float integral_output = MrPid_Limit(candidate_i, config->integral_output_limit);
  float lower = config->output_limit > 0 ? -config->output_limit : -MR_MATH_FINITE_LIMIT;
  float upper = config->output_limit > 0 ? config->output_limit : MR_MATH_FINITE_LIMIT;
  if (config->output_rate_limit > 0) {
    const float delta = config->output_rate_limit * dt_s;
    lower = fmaxf(lower, next.last.output - delta);
    upper = fminf(upper, next.last.output + delta);
  }
  const float requested = pd + integral_output;
  if (!mr_math_is_finite_scalar(requested)) return MrPid_FailedStep(pid);
  const bool saturated = requested > upper || requested < lower;
  if (config->hold_integral_on_saturation) {
    // Reach the bound without accumulating further. Always permit unwinding.
    if (requested > upper && integral_output > old_i)
      integral_output = fmaxf(old_i, upper - pd);
    else if (requested < lower && integral_output < old_i)
      integral_output = fminf(old_i, lower - pd);
  }
  const float state = config->gains.i != 0 ? integral_output / config->gains.i : 0;
  const float unsaturated = pd + integral_output;
  const float output = mr_math_clamp_scalar(unsaturated, lower, upper);
  const float outputs[] = {state, unsaturated, output};
  if (!mr_math_values_are_finite(outputs, 3)) return MrPid_FailedStep(pid);
  next.integral_state = state;
  const MrPidStep step = {output, setpoint, feedback, error, 0, p, integral_output,
                         d, true, saturated};
  next.last = step;
  *pid = next;
  return step;
}
