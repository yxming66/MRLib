#include "component/controller/pid_core.h"
#include <stdio.h>

#define CHECK(x) do { if (!(x)) { fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); return false; } } while (0)
static bool near(float a, float b) { return isfinite(a) && fabsf(a-b) <= 1e-5f; }
static bool test_analytic_pi(void) {
  MrPidConfig config = MrPid_DefaultConfig();
  config.gains.k = 2; config.gains.p = 3; config.gains.i = 4;
  MrPid pid;
  CHECK(MrPid_Configure(&pid, &config));
  MrPidStep step = MrPid_Update(&pid, 2, 1, 0, 0.25f);
  CHECK(step.valid && near(step.p, 6) && near(step.i, 2) && near(step.output, 8));
  step = MrPid_Update(&pid, 2, 1, 0, 0.25f);
  CHECK(step.valid && near(step.output, 10) && near(pid.integral_state, 1));
  step = MrPid_Update(&pid, 0, 1, 0, 0.25f);
  CHECK(step.valid && near(step.output, -4) && near(pid.integral_state, 0.5f));
  CHECK(MrPid_ResetIntegral(&pid, 1));
  CHECK(near(pid.integral_state, 0.25f));
  CHECK(MrPid_Reset(&pid, 2));
  CHECK(near(pid.last.output, 2) && near(pid.integral_state, 0));
  return true;
}
static bool test_integral_limits(void) {
  MrPidConfig config = MrPid_DefaultConfig();
  config.gains.i = 2; config.integral_output_limit = 0.5f;
  MrPid pid;
  CHECK(MrPid_Configure(&pid, &config));
  for (unsigned i=0; i<50; ++i) {
    const MrPidStep step = MrPid_Update(&pid, 1, 0, 0, 0.25f);
    CHECK(step.valid && near(step.i, 0.5f) && near(pid.integral_state, 0.25f));
  }
  CHECK(near(MrPid_Update(&pid, -1, 0, 0, 0.25f).output, 0));
  CHECK(MrPid_ResetIntegral(&pid, 9) && near(pid.integral_state, 0.25f));
  config.gains.i = -2;
  CHECK(MrPid_Configure(&pid, &config));
  CHECK(near(MrPid_Update(&pid, 1, 0, 0, 0.25f).output, -0.5f));
  config.gains.i = 1e-8f; config.integral_output_limit = 0;
  CHECK(MrPid_Configure(&pid, &config));
  CHECK(fabsf(MrPid_Update(&pid, 1, 0, 0, 1).output - 1e-8f) < 1e-11f);
  return true;
}
static bool test_failure_paths(void) {
  MrPid pid;
  MrPidConfig config = MrPid_DefaultConfig();
  CHECK(!MrPid_Update(NULL, 1, 0, 0, 0.001f).valid);
  CHECK(!MrPid_Configure(NULL, &config));
  CHECK(!MrPid_ConfigValid(NULL));
  CHECK(!MrPid_Configure(&pid, NULL));
  CHECK(!MrPid_Update(&pid, 1, 0, 0, 0.001f).valid);
  config.gains.p = 1; config.error_period = 10;
  CHECK(MrPid_Configure(&pid, &config));
  CHECK(near(MrPid_Update(&pid, 5, 0, 0, 0.001f).output, 5));
  CHECK(near(MrPid_Update(&pid, -5, 0, 0, 0.001f).output, -5));
  CHECK(near(MrPid_Update(&pid, 31, 0, 0, 0.001f).output, 1));
  const float previous = pid.last.output;
  CHECK(!MrPid_Update(&pid, NAN, 0, 0, 0.001f).valid && pid.last.output == previous);
  CHECK(!MrPid_Update(&pid, 1, 0, 0, 0).valid && pid.last.output == previous);
  CHECK(!MrPid_Reset(&pid, NAN) && pid.last.output == previous);
  config.control_freq_hz = 0;
  CHECK(!MrPid_Configure(&pid, &config));
  CHECK(!pid.configured && near(pid.last.output, 0));
  return true;
}
int main(void) {
  if (!test_analytic_pi() || !test_integral_limits() || !test_failure_paths()) return 1;
  puts("C PID core tests passed (analytic PI, signed/small I, limits, errors)");
  return 0;
}
