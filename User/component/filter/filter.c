#include "component/filter/filter.h"

MrFilterStatus LowPassFilter2p_Init(LowPassFilter2p_t *f, float sample_hz,
                                    float cutoff_hz) {
  if (f == NULL) return MR_FILTER_INVALID;
  f->cutoff_freq = cutoff_hz;
  f->status = MrBiquad_LowPass(&f->coefficients, sample_hz, cutoff_hz);
  MrBiquad_Clear(&f->state);
  MrBiquad_Reset(&f->state, &f->coefficients, 0.0f);
  return f->status;
}
float LowPassFilter2p_Apply(LowPassFilter2p_t *f, float sample) {
  return f ? MrBiquad_Update(&f->state, &f->coefficients, sample) : 0.0f;
}
float LowPassFilter2p_Reset(LowPassFilter2p_t *f, float sample) {
  if (f == NULL) return 0.0f;
  MrBiquad_Reset(&f->state, &f->coefficients, sample);
  return f->state.output;
}
MrFilterStatus NotchFilter_Init(NotchFilter_t *f, float sample_hz,
                                float notch_hz, float bandwidth_hz) {
  if (f == NULL) return MR_FILTER_INVALID;
  f->notch_freq = notch_hz;
  f->bandwidth = bandwidth_hz;
  f->status = MrBiquad_Notch(&f->coefficients, sample_hz, notch_hz, bandwidth_hz);
  MrBiquad_Clear(&f->state);
  MrBiquad_Reset(&f->state, &f->coefficients, 0.0f);
  return f->status;
}
float NotchFilter_Apply(NotchFilter_t *f, float sample) {
  return f ? MrBiquad_Update(&f->state, &f->coefficients, sample) : 0.0f;
}
float NotchFilter_Reset(NotchFilter_t *f, float sample) {
  if (f == NULL) return 0.0f;
  MrBiquad_Reset(&f->state, &f->coefficients, sample);
  return f->state.output;
}
