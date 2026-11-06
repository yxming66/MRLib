#include "component/controller/pid_legacy.hpp"
#include <cmath>
#include <cstdio>
#include <limits>

#define CHECK(x) do { if (!(x)) { std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #x); return false; } } while (0)
namespace c = mr::comp::cntlr;
static bool Near(float a, float b, float eps=1e-5f) { return std::isfinite(a) && std::fabs(a-b)<=eps; }
static bool Saturation() {
  c::pid_config config{};
  config.gains = {1, 10, 1, 0}; config.output_limit = c::SymmetricLimit::Abs(5);
  auto pid = c::pid::Build(config);
  for (unsigned i=0;i<100;++i) {
    auto step=pid.Step(1,0,0.1f);
    CHECK(step.valid && step.saturated && Near(step.output,5));
    CHECK(Near(step.i,0) && Near(pid.integral(),0));
  }
  CHECK(Near(pid.Step(0,0,0.1f).output,0)); // No rejected candidate I leaks into output.
  config.gains.p=0; config.gains.i=10;
  CHECK(pid.Configure(config));
  auto step=pid.Step(1,0,1);
  CHECK(step.valid && step.saturated && Near(step.output,5));
  CHECK(Near(pid.integral_output(),5)); // Pure I can reach a bound without freezing at zero.
  CHECK(Near(pid.Step(-1,0,0.1f).output,4));
  config.gains={1,0,1,-1}; config.derivative_mode=c::DerivativeMode::kExternal;
  CHECK(pid.Configure(config) && pid.ResetIntegral(4));
  step=pid.Step(-1,0,10,0.1f); // Positive D keeps actuator saturated while I unwinds.
  CHECK(step.saturated && Near(step.output,5) && Near(step.i,3.9f));
  config.gains={1,10,1,0}; config.hold_integral_on_saturation=false;
  CHECK(pid.Configure(config));
  pid.Step(1,0,0.1f); pid.Step(1,0,0.1f);
  CHECK(Near(pid.Step(0,0,0.1f).output,0.2f));
  config.hold_integral_on_saturation=true;
  config.gains={1,0,10,0}; config.output_rate_limit=c::SymmetricLimit::Abs(1);
  CHECK(pid.Configure(config));
  for(unsigned i=1;i<=10;++i) {
    step=pid.Step(1,0,0.1f);
    CHECK(step.valid && Near(step.output,i*0.1f) && Near(step.i,step.output));
  }
  CHECK(pid.Reset(9) && Near(pid.last().output,5)); // Reset seed respects output bound.
  CHECK(Near(pid.Step(0,0,0.1f).output,4.9f));
  return true;
}
static bool DerivativeAndAtomicFailure() {
  c::pid_config config{}; config.gains={2,0,0,0.5f};
  auto pid=c::pid::Build(config);
  CHECK(Near(pid.Step(0,3,0.1f).d,0));
  CHECK(Near(pid.Step(0,3.2f,0.1f).d,-2));
  CHECK(Near(pid.Step(100,3.2f,0.1f).d,0)); // No setpoint D kick.
  CHECK(pid.Reset());
  CHECK(Near(pid.Step(0,9,0.1f).d,0));
  config.derivative_mode=c::DerivativeMode::kExternal;
  CHECK(pid.Configure(config));
  CHECK(Near(pid.Step(0,3,4,0.1f).d,-2)); // External derivative isn't multiplied by k again.
  config.derivative_mode=c::DerivativeMode::kNone;
  CHECK(pid.Configure(config) && Near(pid.Step(0,3,4,0.1f).d,0));
  config.derivative_mode=c::DerivativeMode::kOnMeasurement;
  config.gains={1,2,0.5f,0.1f}; config.derivative_cutoff_hz=40;
  CHECK(pid.Configure(config));
  CHECK(pid.Step(1,0.2f,0.001f).valid);
  auto reference=pid;
  const float previous=pid.last().output;
  const float invalid_dt[]={0,-1,1e-7f,2,NAN,INFINITY,0.02f};
  for(float dt:invalid_dt) {
    CHECK(!pid.Step(9,8,dt).valid && pid.last().output==previous);
    CHECK(pid.integral()==reference.integral());
  }
  CHECK(!pid.Step(NAN,1,0.001f).valid);
  const auto a=pid.Step(1.1f,0.3f,0.001f), b=reference.Step(1.1f,0.3f,0.001f);
  CHECK(a.valid && b.valid && Near(a.output,b.output) && Near(a.d,b.d));
  // Arithmetic errors also roll back derivative/filter/integral histories.
  config.gains={1e30f,1e30f,1,1}; config.derivative_cutoff_hz=0;
  CHECK(pid.Configure(config)); reference=pid;
  CHECK(!pid.Step(1,0,0.001f).valid);
  CHECK(Near(pid.Step(0,0,0.001f).output,reference.Step(0,0,0.001f).output));
  config.gains={1,0,0,1e-30f}; config.derivative_cutoff_hz=40;
  CHECK(pid.Configure(config));
  CHECK(pid.Step(0,1e27f,0.001f).valid);
  reference=pid;
  CHECK(!pid.Step(0,1e30f,0.001f).valid); // Internal filter arithmetic must report failure.
  CHECK(Near(pid.Step(0,1e27f,0.001f).d,reference.Step(0,1e27f,0.001f).d));
  return true;
}
static bool LegacyAgreement() {
  const KPID_Mode_t modes[]={KPID_MODE_NO_D,KPID_MODE_CALC_D,KPID_MODE_SET_D};
  for(auto mode:modes) {
    KPID_Params_t params{1.3f,2.1f,0.4f,0.03f,3,5,40,10};
    KPID_t legacy{}; c::pid_config config{};
    CHECK(c::TryLegacyPidConfig(&params,mode,1000,&config));
    CHECK(Near(config.integral_output_limit.value,1.2f));
    CHECK(PID_Init(&legacy,mode,1000,&params)==0);
    auto pid=c::pid::Build(config);
    CHECK(pid.ready());
    params.k=99; // Both instances own a snapshot; no implicit live parameter pointer.
    for(unsigned i=0;i<3000;++i) {
      const float target=5*std::sin(i*0.013f), feedback=2*std::cos(i*0.017f);
      const float derivative=std::sin(i*0.11f), dt=i%2 ? 0.001f : 0.002f;
      const auto result=pid.Step(target,feedback,derivative,dt);
      CHECK(result.valid && Near(PID_Calc(&legacy,target,feedback,derivative,dt),result.output));
      CHECK(Near(legacy.i,pid.integral()) && Near(legacy.core.last.i,result.i));
      CHECK(legacy.core.last.saturated==result.saturated);
      if(i%400==0) { CHECK(PID_Reset(&legacy)==0 && pid.Reset()); }
    }
    CHECK(PID_ResetIntegral(&legacy)==0 && pid.ResetIntegral());
    CHECK(Near(legacy.i,0) && Near(pid.integral(),0));
  }
  CHECK(PID_Calc(nullptr,1,0,0,0.001f)==0);
  return true;
}
static bool Configuration() {
  c::pid_config base{}; base.gains.p=1;
  const float bad=std::numeric_limits<float>::quiet_NaN();
  for(unsigned i=0;i<11;++i) {
    auto config=base;
    switch(i) {
      case 0: config.gains.k=bad; break;
      case 1: config.gains.i=bad; break;
      case 2: config.output_limit.value=-1; break;
      case 3: config.integral_output_limit.value=bad; break;
      case 4: config.output_rate_limit.value=-1; break;
      case 5: config.error_wrap.range=-1; break;
      case 6: config.control_freq_hz=0; break;
      case 7: config.control_freq_hz=1000001; break;
      case 8: config.derivative_cutoff_hz=500; break;
      case 9: config.derivative_cutoff_hz=bad; break;
      case 10: config.derivative_mode=static_cast<c::DerivativeMode>(127); break;
    }
    auto pid=c::pid::Build(config);
    CHECK(!pid.ready() && !pid.Step(1,0).valid);
    CHECK(pid.Configure(base) && Near(pid.Step(1,0).output,1));
    CHECK(!pid.Configure(config) && !pid.ready() && pid.last().output==0);
  }
  KPID_Params_t params{1,1,2,0,3,5,0,0};
  c::pid_config config{};
  CHECK(c::TryLegacyPidConfig(&params,KPID_MODE_NO_D,1000,&config));
  CHECK(Near(config.integral_output_limit.value,6));
  CHECK(!c::TryLegacyPidConfig(&params,static_cast<KPID_Mode_t>(127),1000,&config));
  CHECK(!c::pid::IsConfigValid(config));
  auto core=c::pid::ToCoreConfig(base);
  CHECK(c::pid::IsConfigValid(c::pid::FromCoreConfig(core)));
  core.derivative_mode=static_cast<MrPidDerivativeMode>(256);
  CHECK(!c::pid::IsConfigValid(c::pid::FromCoreConfig(core)));
  return true;
}
int main() {
  if(!Saturation() || !DerivativeAndAtomicFailure() || !LegacyAgreement() || !Configuration()) return 1;
  std::puts("C++ PID tests passed (windup, slew, D reset, transaction, legacy equivalence)");
}
