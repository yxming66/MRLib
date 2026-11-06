#include "component/math/scalar.h"
#include "component/ahrs.h"
#include "component/crc16.h"
#include "component/filter/filter.h"
#include "component/pid.h"

#include <stdio.h>

#if defined(min) || defined(max) || defined(__packed)
#error Scalar math must not define unrelated global macros
#endif

#define CHECK(condition) do { \
  if (!(condition)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #condition); \
    return false; \
  } \
} while (0)

static bool near(float actual, float expected, float tolerance) {
  return isfinite(actual) && fabsf(actual - expected) <= tolerance;
}

static bool test_scalar_boundaries(void) {
  CHECK(mr_math_clamp_scalar(5.0f, -2.0f, 2.0f) == 2.0f);
  CHECK(mr_math_abs_clip_scalar(-5.0f, -2.0f) == -2.0f);
  CHECK(near(mr_math_wrap_to_range(31.0f, -2.0f, 2.0f,
                                 MR_MATH_DEFAULT_EPSILON), -1.0f, 1e-6f));
  CHECK(near(mr_math_wrap_to_range(-31.0f, -2.0f, 2.0f,
                                 MR_MATH_DEFAULT_EPSILON), 1.0f, 1e-6f));
  CHECK(mr_math_wrap_to_pi(MR_MATH_PI) == -MR_MATH_PI);
  CHECK(mr_math_wrap_error(25.0f, 10.0f) == 5.0f);
  CHECK(mr_math_wrap_error(-25.0f, 10.0f) == -5.0f);
  CHECK(mr_math_wrap_error(31.0f, 10.0f) == 1.0f);
  CHECK(mr_math_wrap_error(-31.0f, 10.0f) == -1.0f);
  CHECK(mr_math_wrap_error(3.0f, 0.0f) == 3.0f);
  CHECK(!mr_math_is_finite_scalar(NAN));
  CHECK(!mr_math_is_finite_scalar(INFINITY));
  CHECK(mr_math_normalization_factor(0.0f) == 0.0f);
  CHECK(mr_math_normalization_factor(-1.0f) == 0.0f);
  CHECK(mr_math_normalization_factor(INFINITY) == 0.0f);
  CHECK(near(mr_math_normalization_factor(4.0f), 0.5f, 1e-6f));
  CHECK(mr_math_uint_to_float(0, -2.0f, 2.0f, 16) == -2.0f);
  CHECK(mr_math_uint_to_float(UINT32_MAX, -2.0f, 2.0f, 16) == 2.0f);
  CHECK(mr_math_uint_to_float(1, -2.0f, 2.0f, 32) == -2.0f);
  return true;
}

static bool test_c_filters(void) {
  LowPassFilter2p_t lowpass;
  NotchFilter_t notch;
  LowPassFilter2p_Init(&lowpass, 1000.0f, 40.0f);
  float output = 0.0f;
  for (unsigned i = 0; i < 500; ++i) {
    output = LowPassFilter2p_Apply(&lowpass, 1.0f);
    CHECK(isfinite(output));
  }
  CHECK(near(output, 1.0f, 1e-4f));
  CHECK(near(LowPassFilter2p_Reset(&lowpass, 0.5f), 0.5f, 1e-4f));
  LowPassFilter2p_Init(&lowpass, 1000.0f, 0.0f);
  CHECK(LowPassFilter2p_Apply(&lowpass, -3.0f) == -3.0f);
  NotchFilter_Init(&notch, 1000.0f, 100.0f, 20.0f);
  CHECK(near(NotchFilter_Reset(&notch, 0.5f), 0.5f, 1e-4f));
  for (unsigned i = 0; i < 500; ++i) {
    output = NotchFilter_Apply(&notch, 0.5f);
  }
  CHECK(near(output, 0.5f, 1e-4f));
  return true;
}

static bool test_c_consumers(void) {
  KPID_Params_t params = {0};
  KPID_t pid;
  params.k = 1.0f;
  params.p = 1.0f;
  params.out_limit = 100.0f;
  params.range = 10.0f;
  CHECK(PID_Init(&pid, KPID_MODE_NO_D, 1000.0f, &params) == 0);
  CHECK(PID_Calc(&pid, 2.0f, 0.0f, 0.0f, 0.001f) == 2.0f);
  CHECK(PID_Calc(&pid, 31.0f, 0.0f, 0.0f, 0.001f) == 1.0f);
  CHECK(PID_Calc(&pid, -31.0f, 0.0f, 0.0f, 0.001f) == -1.0f);
  /* CRC-16/MCRF4XX check value, reflected 0x1021, init 0xffff. */
  const uint8_t bytes[] = "123456789";
  CHECK(CRC16_Calc(bytes, 9, CRC16_INIT) == 0x6f91);
  return true;
}

static bool test_stationary_ahrs(void) {
  AHRS_t ahrs;
  AHRS_Eulr_t angles;
  const AHRS_Accl_t acceleration = {0.0f, 0.0f, 1.0f};
  const AHRS_Gyro_t gyro = {0.0f, 0.0f, 0.0f};
  CHECK(AHRS_Init(&ahrs, NULL, 1000.0f) == 0);
  /* An exactly aligned IMU produces a zero gradient correction vector. */
  for (unsigned i = 0; i < 1000; ++i) {
    CHECK(AHRS_Update(&ahrs, &acceleration, &gyro, NULL) == 0);
    CHECK(near(ahrs.quat.q0, 1.0f, 1e-5f));
    CHECK(near(ahrs.quat.q1, 0.0f, 1e-5f));
    CHECK(near(ahrs.quat.q2, 0.0f, 1e-5f));
    CHECK(near(ahrs.quat.q3, 0.0f, 1e-5f));
  }
  CHECK(AHRS_GetEulr(&angles, &ahrs) == 0);
  CHECK(near(angles.yaw, 0.0f, 1e-5f));
  CHECK(near(angles.pit, 0.0f, 1e-5f));
  CHECK(near(angles.rol, 0.0f, 1e-5f));
  /* A vertical magnetic field has zero horizontal magnitude. It must not
   * divide by the normalization factor of a zero vector. */
  const AHRS_Magn_t fields[] = {{0.0f, 0.0f, 1.0f}, {1.0f, 0.0f, 0.0f}};
  for (unsigned field = 0; field < 2; ++field) {
    CHECK(AHRS_Init(&ahrs, NULL, 1000.0f) == 0);
    for (unsigned i = 0; i < 1000; ++i) {
      CHECK(AHRS_Update(&ahrs, &acceleration, &gyro, &fields[field]) == 0);
      CHECK(near(ahrs.quat.q0, 1.0f, 1e-5f));
      CHECK(near(ahrs.quat.q1, 0.0f, 1e-5f));
      CHECK(near(ahrs.quat.q2, 0.0f, 1e-5f));
      CHECK(near(ahrs.quat.q3, 0.0f, 1e-5f));
    }
  }
  return true;
}

int main(void) {
  if (!test_scalar_boundaries() || !test_c_filters() ||
      !test_c_consumers() || !test_stationary_ahrs()) return 1;
  puts("C component tests passed (4 groups)");
  return 0;
}
