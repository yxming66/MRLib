#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "bsp/pwm.h"
#include "device/device.h"
#include "component/audio/score.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct {
  DEVICE_Header_t header;
  BSP_PWM_Channel_t channel;
} BUZZER_t;
// Hardware output only; scheduling and playlists live in module/audio.
int8_t BUZZER_Init(BUZZER_t* buzzer, BSP_PWM_Channel_t channel);
int8_t BUZZER_Start(BUZZER_t* buzzer);
int8_t BUZZER_Stop(BUZZER_t* buzzer);
int8_t BUZZER_Set(BUZZER_t* buzzer, float freq, float duty_cycle);
float BUZZER_CalcFreq(NOTE_t note, uint8_t octave);
int8_t BUZZER_ApplyTone(BUZZER_t* buzzer, NOTE_t note, uint8_t octave);
#ifdef __cplusplus
}
#endif
