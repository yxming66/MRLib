#pragma once
#include "module/audio/player.h"
#ifdef __cplusplus
extern "C" {
#endif
// Application owns the playlist array and immutable melodies for the lifetime
// of this instance. Empty configuration is valid and stays silent.
typedef struct {
  const Audio_Score_t* const* playlist;
  size_t playlist_length;
  const Audio_Score_t* startup;
  bool loop_playlist;
  bool start_paused;
} CloudMusic_Config_t;
typedef struct {
  bool next_event, previous_event, toggle_pause_event, play_event, stop_event;
} CloudMusic_Control_t;
typedef struct {
  Audio_Player_t player;
  CloudMusic_Config_t config;
  size_t playlist_index;
  bool startup_active, stopped, paused, initialized;
  int8_t last_error;
} CloudMusic_t;
int8_t CloudMusic_Init(CloudMusic_t* music, Audio_Sink_t sink,
                       const CloudMusic_Config_t* config);
int8_t CloudMusic_Start(CloudMusic_t* music, uint32_t now_ms);
int8_t CloudMusic_Update(CloudMusic_t* music, uint32_t now_ms);
int8_t CloudMusic_ApplyControl(CloudMusic_t* music,
                               const CloudMusic_Control_t* control, uint32_t now_ms);
int8_t CloudMusic_Next(CloudMusic_t* music, uint32_t now_ms);
int8_t CloudMusic_Previous(CloudMusic_t* music, uint32_t now_ms);
int8_t CloudMusic_SetPaused(CloudMusic_t* music, bool paused, uint32_t now_ms);
int8_t CloudMusic_Stop(CloudMusic_t* music);
#ifdef __cplusplus
}
#endif
