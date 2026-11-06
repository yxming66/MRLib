#include "component/controller/pid.hpp"
#include "component/filter/filter.hpp"
#include "component/filter/filter.h"
#include "component/pid.h"

#include <cmath>
#include <cstdio>
#include <limits>

#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); return false; } } while (0)
namespace f = mr::comp::filter;
static bool Near(float a, float b, float eps = 1e-5f) {
  return std::isfinite(a) && std::fabs(a - b) <= eps;
}

static bool ApiAgreement() {
  LowPassFilter2p_t c_low{};
  NotchFilter_t c_notch{};
  CHECK(LowPassFilter2p_Init(&c_low, 1000, 40) == MR_FILTER_ENABLED);
  CHECK(NotchFilter_Init(&c_notch, 1000, 100, 20) == MR_FILTER_ENABLED);
  auto low = f::low_pass::Build(40, 1000);
  auto notch = f::notch::Build(100, 20, 1000);
  low.Reset(0.5f); notch.Reset(0.5f);
  LowPassFilter2p_Reset(&c_low, 0.5f); NotchFilter_Reset(&c_notch, 0.5f);
  for (unsigned i = 0; i < 3000; ++i) {
    const float input = std::sin(i * 0.37f) + (i > 1000 ? 2.0f : -1.0f);
    CHECK(Near(low.Update(input), LowPassFilter2p_Apply(&c_low, input)));
    CHECK(Near(notch.Update(input), NotchFilter_Apply(&c_notch, input)));
  }
  const float previous = low.output();
  CHECK(low.Update(std::numeric_limits<float>::quiet_NaN()) == previous);
  CHECK(low.Update(1, 0) == previous);
  CHECK(low.Update(1, -1) == previous);
  CHECK(low.Update(1, std::numeric_limits<float>::infinity()) == previous);
  for (unsigned i = 0; i < 2000; ++i) CHECK(std::isfinite(low.Update(3)));
  CHECK(Near(low.output(), 3, 1e-4f));
  return true;
}

template<class Filter> static float Gain(Filter filter, float frequency) {
  filter.Reset();
  double input_energy = 0, output_energy = 0;
  for (unsigned i = 0; i < 8000; ++i) {
    const float input = std::sin(2.0 * 3.141592653589793 * frequency * i / 1000.0);
    const float output = filter.Update(input);
    if (i >= 4000) { input_energy += input * input; output_energy += output * output; }
  }
  return static_cast<float>(std::sqrt(output_energy / input_energy));
}
static bool FrequencyResponse() {
  const auto low = f::low_pass::Build(40, 1000);
  CHECK(Near(Gain(low, 40), std::sqrt(0.5f), 5e-4f));
  CHECK(Gain(low, 5) > 0.99f);
  CHECK(Gain(low, 200) < 0.04f);
  const auto notch = f::notch::Build(100, 20, 1000);
  CHECK(Gain(notch, 100) < 1e-4f);
  CHECK(Gain(notch, 10) > 0.99f);
  return true;
}
static bool RetuningAndValidation() {
  auto low = f::low_pass::Build(40, 1000);
  auto nominal = low;
  low.Reset(); nominal.Reset();
  for (unsigned i = 0; i < 500; ++i)
    CHECK(Near(low.Update(std::sin(i * 0.1f), 0.001f), nominal.Update(std::sin(i * 0.1f)), 1e-4f));
  low.Update(1, 0.02f); // Nyquist is now 25 Hz: cutoff is invalid.
  CHECK(!low.valid() && low.bypassed());
  low.Update(1); // Fixed-period update restores the configured 1 kHz rate.
  CHECK(low.valid() && !low.bypassed());
  auto notch = f::notch::Build(100, 20, 1000);
  notch.Update(1, 0.01f);
  CHECK(!notch.valid());
  notch.Update(1);
  CHECK(notch.valid());
  CHECK(!low.SetCutoff(500));
  CHECK(low.Update(-2) == -2);
  CHECK(low.SetCutoff(0) && low.status() == MR_FILTER_DISABLED);
  CHECK(low.Update(7) == 7);
  CHECK(!notch.SetNotch(100, 500));
  CHECK(notch.SetNotch(0, 0) && notch.status() == MR_FILTER_DISABLED);
  LowPassFilter2p_t c{};
  CHECK(LowPassFilter2p_Init(nullptr, 1000, 40) == MR_FILTER_INVALID);
  CHECK(LowPassFilter2p_Init(&c, 0, 40) == MR_FILTER_INVALID);
  CHECK(LowPassFilter2p_Apply(&c, 2) == 2);
  CHECK(LowPassFilter2p_Init(&c, 1000, 0) == MR_FILTER_DISABLED);
  KPID_t pid{};
  KPID_Params_t params{};
  CHECK(PID_Init(nullptr, KPID_MODE_NO_D, 1000, &params) == -1);
  CHECK(PID_Init(&pid, KPID_MODE_NO_D, 1000, nullptr) == -1);
  CHECK(PID_Init(&pid, KPID_MODE_NO_D, 0, &params) == -1);
  params.d_cutoff_freq = 500;
  CHECK(PID_Init(&pid, KPID_MODE_CALC_D, 1000, &params) == -1);
  auto cpp_pid = mr::comp::cntlr::pid::Build(1, 1, 0, 0, 10, 1, 500, 1000);
  CHECK(!cpp_pid.Step(1, 0).valid);
  return true;
}
int main() {
  if (!ApiAgreement() || !FrequencyResponse() || !RetuningAndValidation()) return 1;
  std::puts("Filter tests passed (C/C++ agreement, frequency response, retuning, validation)");
}
