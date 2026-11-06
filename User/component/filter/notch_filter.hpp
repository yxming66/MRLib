#pragma once

#include "component/filter/filter_types.hpp"

namespace mr::comp::filter {

struct notch_config {
  Scalar sample_freq_hz = kDefaultSampleFreqHz;
  Scalar notch_freq_hz = 0.0f;
  Scalar bandwidth_hz = 0.0f;
};

/* Fixed Update uses the configured frequency. Update(sample, dt) retunes only
 * when the effective frequency changes; fixed/variable calls may be mixed.
 * Configure resets the state; setters retain it. Call Reset to seed the signal.
 * Invalid configuration bypasses and reports MR_FILTER_INVALID; intentional
 * disabled frequencies report MR_FILTER_DISABLED. Check status after retuning.
 */
class notch {
 public:
  using Config = notch_config;
  static notch Build(const Config& config) { return notch(config); }
  static notch Build(Scalar notch_freq_hz, Scalar bandwidth_hz,
                          Scalar sample_freq_hz = kDefaultSampleFreqHz) {
    Config config{};
    config.notch_freq_hz = notch_freq_hz;
    config.bandwidth_hz = bandwidth_hz;
    config.sample_freq_hz = sample_freq_hz;
    return notch(config);
  }
  notch() { Configure(Config{}); }
  explicit notch(const Config& config) { Configure(config); }
  bool Configure(const Config& config) {
    config_ = config;
    state_ = {};
    return Reconfigure();
  }
  bool SetNotch(Scalar notch_freq_hz, Scalar bandwidth_hz) {
    config_.notch_freq_hz = notch_freq_hz;
    config_.bandwidth_hz = bandwidth_hz;
    return Reconfigure();
  }
  bool SetSampleFreq(Scalar sample_freq_hz) {
    config_.sample_freq_hz = sample_freq_hz;
    return Reconfigure();
  }
  void Reset(Scalar output = 0.0f) { state_.Reset(coeff_, output); }
  Scalar Update(Scalar sample) {
    UseFrequency(config_.sample_freq_hz);
    return state_.Update(coeff_, sample);
  }
  Scalar Update(Scalar sample, Scalar dt_s) {
    if (!is_usable_scalar(dt_s) || dt_s <= 0.0f) return state_.output();
    UseFrequency(1.0f / dt_s);
    return state_.Update(coeff_, sample);
  }
  Scalar Apply(Scalar sample) { return Update(sample); }
  Scalar Apply(Scalar sample, Scalar dt_s) { return Update(sample, dt_s); }
  const Config& config() const { return config_; }
  Scalar output() const { return state_.output(); }
  bool bypassed() const { return coeff_.bypass; }
  bool valid() const { return status_ != MR_FILTER_INVALID; }
  MrFilterStatus status() const { return status_; }

 private:
  bool Reconfigure() {
    Retune(config_.sample_freq_hz);
    return valid();
  }
  void UseFrequency(Scalar sample_freq_hz) {
    if (sample_freq_hz != active_sample_freq_hz_) Retune(sample_freq_hz);
  }
  void Retune(Scalar sample_freq_hz) {
    active_sample_freq_hz_ = sample_freq_hz;
    status_ = MrBiquad_Notch(&coeff_, sample_freq_hz, config_.notch_freq_hz, config_.bandwidth_hz);
  }
  Config config_{};
  Scalar active_sample_freq_hz_ = 0.0f;
  MrFilterStatus status_ = MR_FILTER_INVALID;
  detail::biquad_coefficients coeff_ = MrBiquad_Bypass();
  detail::biquad_state state_{};
};

}  // namespace mr::comp::filter
