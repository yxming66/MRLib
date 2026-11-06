#pragma once
#include "component/filter/biquad.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float cutoff_freq;
  MrBiquadCoefficients coefficients;
  MrBiquadState state;
  MrFilterStatus status;
} LowPassFilter2p_t;

typedef struct {
  float notch_freq;
  float bandwidth;
  MrBiquadCoefficients coefficients;
  MrBiquadState state;
  MrFilterStatus status;
} NotchFilter_t;

/* Returns INVALID for bad configuration, DISABLED for an explicit off setting.
 * Invalid configurations bypass safely; callers must check the returned status.
 * Init starts at zero. Reset can seed a known initial signal value.
 */
MrFilterStatus LowPassFilter2p_Init(LowPassFilter2p_t *filter, float sample_hz,
                                    float cutoff_hz);
float LowPassFilter2p_Apply(LowPassFilter2p_t *filter, float sample);
float LowPassFilter2p_Reset(LowPassFilter2p_t *filter, float sample);
MrFilterStatus NotchFilter_Init(NotchFilter_t *filter, float sample_hz,
                                float notch_hz, float bandwidth_hz);
float NotchFilter_Apply(NotchFilter_t *filter, float sample);
float NotchFilter_Reset(NotchFilter_t *filter, float sample);

#ifdef __cplusplus
}
#endif
