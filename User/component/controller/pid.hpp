#pragma once

/* C++17 interface over pid_core.h; no second control law.
 * Configure returns false on invalid parameters. Check ready()/Step.valid.
 * Explicit dt is seconds in [1 us, 1 s]. Reset seeds output, not feedback;
 * the first measurement after Reset establishes history without D kick.
 */
#include "component/controller/controller_types.hpp"
#include "component/controller/pid_core.h"

namespace mr::comp::cntlr {

struct pid_gains {
  Scalar k = 1.0f;
  Scalar p = 0.0f;
  Scalar i = 0.0f;
  Scalar d = 0.0f;
};

struct pid_config {
  pid_gains gains{};
  SymmetricLimit output_limit{};
  SymmetricLimit integral_output_limit{};
  SymmetricLimit output_rate_limit{};
  RangeWrap error_wrap{};
  DerivativeMode derivative_mode = DerivativeMode::kOnMeasurement;
  Scalar derivative_cutoff_hz = 0.0f;
  Scalar control_freq_hz = kDefaultSampleFreqHz;
  bool hold_integral_on_saturation = true;
};

class pid {
 public:
  using Gains = pid_gains;
  using Config = pid_config;

  static pid Build(const Config& config) { return pid(config); }

  static pid Build(Scalar k,
                   Scalar p,
                   Scalar i,
                   Scalar d,
                   Scalar output_limit = kUnlimited,
                   Scalar integral_output_limit = kUnlimited,
                   Scalar derivative_cutoff_hz = 0.0f,
                   Scalar control_freq_hz = kDefaultSampleFreqHz) {
    return Build({k, p, i, d}, SymmetricLimit::Abs(output_limit),
                 SymmetricLimit::Abs(integral_output_limit),
                 SymmetricLimit::Unlimited(),
                 RangeWrap::Disabled(), DerivativeMode::kOnMeasurement,
                 derivative_cutoff_hz, true, control_freq_hz);
  }

  static pid Build(const Gains& gains,
                   SymmetricLimit output_limit = SymmetricLimit::Unlimited(),
                   SymmetricLimit integral_output_limit =
                       SymmetricLimit::Unlimited(),
                   SymmetricLimit output_rate_limit =
                       SymmetricLimit::Unlimited(),
                   RangeWrap error_wrap = RangeWrap::Disabled(),
                   DerivativeMode derivative_mode =
                       DerivativeMode::kOnMeasurement,
                   Scalar derivative_cutoff_hz = 0.0f,
                   bool hold_integral_on_saturation = true,
                   Scalar control_freq_hz = kDefaultSampleFreqHz) {
    Config config{};
    config.gains = gains;
    config.output_limit = output_limit;
    config.integral_output_limit = integral_output_limit;
    config.output_rate_limit = output_rate_limit;
    config.error_wrap = error_wrap;
    config.derivative_mode = derivative_mode;
    config.derivative_cutoff_hz = derivative_cutoff_hz;
    config.control_freq_hz = control_freq_hz;
    config.hold_integral_on_saturation = hold_integral_on_saturation;
    return pid(config);
  }

  pid() { Configure(Config{}); }
  explicit pid(const Config& config) { Configure(config); }

  bool Configure(const Config& config) {
    config_ = config;
    const auto core_config = ToCoreConfig(config);
    const bool result = MrPid_Configure(&core_, &core_config);
    SyncLast();
    return result;
  }
  bool Reset(Scalar output = 0.0f) {
    const bool result = MrPid_Reset(&core_, output);
    SyncLast();
    return result;
  }
  bool ResetIntegral(Scalar integral_output = 0.0f) {
    return MrPid_ResetIntegral(&core_, integral_output);
  }
  Scalar Update(Scalar setpoint, Scalar feedback) { return Step(setpoint, feedback).output; }
  Scalar Update(Scalar setpoint, Scalar feedback, Scalar dt_s) {
    return Step(setpoint, feedback, dt_s).output;
  }
  Scalar Update(Scalar setpoint, Scalar feedback, Scalar feedback_dot, Scalar dt_s) {
    return Step(setpoint, feedback, feedback_dot, dt_s).output;
  }
  Scalar Apply(Scalar setpoint, Scalar feedback) { return Update(setpoint, feedback); }
  Scalar Apply(Scalar setpoint, Scalar feedback, Scalar dt_s) {
    return Update(setpoint, feedback, dt_s);
  }
  Scalar Apply(Scalar setpoint, Scalar feedback, Scalar feedback_dot, Scalar dt_s) {
    return Update(setpoint, feedback, feedback_dot, dt_s);
  }
  ::mr::comp::cntlr::Step Step(Scalar setpoint, Scalar feedback) {
    return Step(setpoint, feedback, 0, 1.0f / core_.config.control_freq_hz);
  }
  ::mr::comp::cntlr::Step Step(Scalar setpoint, Scalar feedback, Scalar dt_s) {
    return Step(setpoint, feedback, 0, dt_s);
  }
  ::mr::comp::cntlr::Step Step(Scalar setpoint, Scalar feedback,
                              Scalar feedback_dot, Scalar dt_s) {
    MrPid_Update(&core_, setpoint, feedback, feedback_dot, dt_s);
    SyncLast();
    return last_;
  }
  bool ready() const { return core_.configured; }
  const ::mr::comp::cntlr::Step& last() const { return last_; }
  const Config& config() const { return config_; }
  Scalar integral() const { return core_.integral_state; }
  Scalar integral_output() const { return core_.integral_state * core_.config.gains.i; }

  // Representation conversion only; validation and behavior live in C core.
  static MrPidConfig ToCoreConfig(const Config& config) {
    const MrPidConfig result = {{config.gains.k, config.gains.p, config.gains.i, config.gains.d},
        config.output_limit.value, config.integral_output_limit.value,
        config.output_rate_limit.value, config.error_wrap.range,
        static_cast<MrPidDerivativeMode>(config.derivative_mode),
        config.derivative_cutoff_hz, config.control_freq_hz,
        config.hold_integral_on_saturation};
    return result;
  }
  static Config FromCoreConfig(const MrPidConfig& config) {
    Config result{};
    result.gains = {config.gains.k, config.gains.p, config.gains.i, config.gains.d};
    result.output_limit = SymmetricLimit::Abs(config.output_limit);
    result.integral_output_limit = SymmetricLimit::Abs(config.integral_output_limit);
    result.output_rate_limit = SymmetricLimit::Abs(config.output_rate_limit);
    result.error_wrap = RangeWrap::Periodic(config.error_period);
    result.derivative_mode = static_cast<DerivativeMode>(config.derivative_mode);
    result.derivative_cutoff_hz = config.derivative_cutoff_hz;
    result.control_freq_hz = config.control_freq_hz;
    result.hold_integral_on_saturation = config.hold_integral_on_saturation;
    if (!MrPid_ConfigValid(&config)) result.control_freq_hz = 0;
    return result;
  }
  static bool IsConfigValid(const Config& config) {
    const auto core = ToCoreConfig(config);
    return MrPid_ConfigValid(&core);
  }
 private:
  void SyncLast() {
    const auto& value = core_.last;
    last_ = {value.output, value.setpoint, value.feedback, value.error,
             value.feedforward, value.p, value.i, value.d, value.valid, value.saturated};
  }
  Config config_{};
  MrPid core_{};
  ::mr::comp::cntlr::Step last_{};
};
}  // namespace mr::comp::cntlr
