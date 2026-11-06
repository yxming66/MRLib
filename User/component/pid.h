#pragma once

/* Transitional entry points for the former PX4-inspired C PID.
 * New C code may use controller/pid_core.h directly. PID_Init copies parameters;
 * changing an old parameter table requires reinitialization.
 */
#include <stdint.h>
#include "component/controller/pid_core.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  KPID_MODE_NO_D = 0,
  KPID_MODE_CALC_D,
  KPID_MODE_SET_D
} KPID_Mode_t;
typedef struct {
  float k, p, i, d;
  float i_limit;       /* Legacy integral STATE limit; 0 is unlimited. */
  float out_limit;     /* Output limit; 0 is unlimited. */
  float d_cutoff_freq; /* Hz; <= 0 disables the derivative filter. */
  float range;         /* Full error period; 0 disables wrapping. */
} KPID_Params_t;
typedef struct {
  MrPid core;
  float i; /* Read-only diagnostic mirror of integral state. */
  struct { float err, k_fb, out; } last; /* Read-only diagnostics. */
} KPID_t;

/* The only legacy parameter conversion; shared with C++ device adapters. */
static inline bool PID_ConfigFromLegacy(MrPidConfig *out, KPID_Mode_t mode,
                                         float sample_hz, const KPID_Params_t *params) {
  if (out == NULL) return false;
  MrPidConfig config = MrPid_DefaultConfig();
  config.control_freq_hz = 0; // Failure leaves an explicitly invalid config.
  *out = config;
  if (params == NULL || !mr_math_is_finite_scalar(params->i_limit) || params->i_limit < 0)
    return false;
  const MrPidGains gains = {params->k, params->p, params->i, params->d};
  config.gains = gains;
  config.output_limit = params->out_limit;
  config.integral_output_limit = fabsf(params->i) * params->i_limit;
  config.error_period = params->range;
  config.derivative_cutoff_hz = params->d_cutoff_freq;
  config.control_freq_hz = sample_hz;
  switch (mode) {
    case KPID_MODE_NO_D: config.derivative_mode = MR_PID_D_NONE; break;
    case KPID_MODE_CALC_D: config.derivative_mode = MR_PID_D_MEASUREMENT; break;
    case KPID_MODE_SET_D: config.derivative_mode = MR_PID_D_EXTERNAL; break;
    default: return false;
  }
  if (!MrPid_ConfigValid(&config)) return false;
  *out = config;
  return true;
}
int8_t PID_Init(KPID_t *pid, KPID_Mode_t mode, float sample_freq,
                const KPID_Params_t *param);
/* dt is actual seconds in [1 us, 1 s]; invalid input holds output, valid=false.
 * External fb_dot is d(k * feedback)/dt, matching the old C/C++ convention.
 */
float PID_Calc(KPID_t *pid, float sp, float fb, float fb_dot, float dt);
int8_t PID_ResetIntegral(KPID_t *pid);
int8_t PID_Reset(KPID_t *pid);

#ifdef __cplusplus
}
#endif
