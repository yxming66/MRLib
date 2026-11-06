#pragma once
#include "device/buzzer.h"
// Optional project wiring; generic playback never includes the PWM driver.
static inline int8_t MusicBuzzerSink(void* context, NOTE_t note, uint8_t octave) {
  return BUZZER_ApplyTone((BUZZER_t*)context, note, octave);
}
