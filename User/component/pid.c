/* Thin C adapter. Control law: component/controller/pid_core.h.
 * Derived from the former PX4-inspired implementation:
 * https://github.com/PX4/Firmware/blob/master/src/lib/pid/pid.cpp
 */
#include "component/pid.h"

static void SyncDiagnostics(KPID_t *pid) {
  pid->i = pid->core.integral_state;
  pid->last.err = pid->core.last.error;
  pid->last.k_fb = pid->core.last_filtered_feedback;
  pid->last.out = pid->core.last.output;
}
int8_t PID_Init(KPID_t *pid, KPID_Mode_t mode, float sample_freq,
                const KPID_Params_t *param) {
  if (pid == NULL) return -1;
  MrPidConfig config;
  const bool valid = PID_ConfigFromLegacy(&config, mode, sample_freq, param);
  MrPid_Configure(&pid->core, valid ? &config : NULL);
  SyncDiagnostics(pid);
  return valid ? 0 : -1;
}
float PID_Calc(KPID_t *pid, float sp, float fb, float fb_dot, float dt) {
  if (pid == NULL) return 0;
  const MrPidStep result = MrPid_Update(&pid->core, sp, fb, fb_dot, dt);
  SyncDiagnostics(pid);
  return result.output;
}
int8_t PID_ResetIntegral(KPID_t *pid) {
  if (pid == NULL || !MrPid_ResetIntegral(&pid->core, 0)) return -1;
  SyncDiagnostics(pid);
  return 0;
}
int8_t PID_Reset(KPID_t *pid) {
  if (pid == NULL || !pid->core.configured) return -1;
  MrPid_Reset(&pid->core, 0);
  SyncDiagnostics(pid);
  return 0;
}
