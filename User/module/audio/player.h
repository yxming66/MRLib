#pragma once
#include <stdbool.h>
#include "component/audio/score.h"
#ifdef __cplusplus
extern "C" {
#endif
#define AUDIO_OK 0
#define AUDIO_ERR (-1)
// NOTE_REST silences output. Callback is synchronous; no score pointer escapes.
typedef struct {
  void* context;
  int8_t (*set_tone)(void* context, NOTE_t note, uint8_t octave);
} Audio_Sink_t;
typedef struct {
  Audio_Sink_t sink;
  Audio_Score_t score;
  size_t index;
  uint32_t deadline_ms;
  uint32_t remaining_ms;
  bool active, paused, gap, loop;
  int8_t last_error;
} Audio_Player_t;
bool Audio_ScoreIsValid(const Audio_Score_t* score);
int8_t Audio_PlayerInit(Audio_Player_t* player, Audio_Sink_t sink);
int8_t Audio_PlayerStart(Audio_Player_t* player, const Audio_Score_t* score,
                         bool loop, bool paused, uint32_t now_ms);
int8_t Audio_PlayerUpdate(Audio_Player_t* player, uint32_t now_ms);
int8_t Audio_PlayerStop(Audio_Player_t* player);
int8_t Audio_PlayerSetPaused(Audio_Player_t* player, bool paused, uint32_t now_ms);
#ifdef __cplusplus
}
#endif
