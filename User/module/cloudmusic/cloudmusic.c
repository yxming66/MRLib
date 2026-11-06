#include "module/cloudmusic/cloudmusic.h"

static bool Ready(const CloudMusic_t* music) {
  return music != NULL && music->initialized;
}
static int8_t Finish(CloudMusic_t* music, int8_t result) {
  music->last_error = result;
  if (result != AUDIO_OK) {
    music->stopped = true;
    music->startup_active = false;
    (void)Audio_PlayerStop(&music->player);
  }
  return result;
}
static int8_t Play(CloudMusic_t* music, const Audio_Score_t* score, uint32_t now_ms) {
  const int8_t result = Audio_PlayerStart(&music->player, score, false, music->paused, now_ms);
  if (result != AUDIO_OK) return Finish(music, result);
  music->stopped = false;
  return Finish(music, AUDIO_OK);
}
static int8_t PlayIndex(CloudMusic_t* music, uint32_t now_ms) {
  if (music->config.playlist_length == 0) return AUDIO_ERR;
  music->startup_active = false;
  return Play(music, music->config.playlist[music->playlist_index], now_ms);
}
int8_t CloudMusic_Init(CloudMusic_t* music, Audio_Sink_t sink,
                       const CloudMusic_Config_t* config) {
  if (music == NULL || config == NULL || sink.set_tone == NULL ||
      (config->playlist_length > 0 && config->playlist == NULL) ||
      (config->startup != NULL && !Audio_ScoreIsValid(config->startup))) return AUDIO_ERR;
  for (size_t i = 0; i < config->playlist_length; ++i)
    if (!Audio_ScoreIsValid(config->playlist[i])) return AUDIO_ERR;
  *music = (CloudMusic_t){0};
  music->config = *config;
  (void)Audio_PlayerInit(&music->player, sink);
  music->initialized = true;
  music->stopped = true;
  return AUDIO_OK;
}
int8_t CloudMusic_Start(CloudMusic_t* music, uint32_t now_ms) {
  if (!Ready(music)) return AUDIO_ERR;
  music->playlist_index = 0;
  music->paused = music->config.start_paused;
  music->startup_active = music->config.startup != NULL;
  if (music->startup_active) return Play(music, music->config.startup, now_ms);
  if (music->config.playlist_length > 0) return PlayIndex(music, now_ms);
  return CloudMusic_Stop(music);
}
int8_t CloudMusic_Update(CloudMusic_t* music, uint32_t now_ms) {
  if (!Ready(music)) return AUDIO_ERR;
  if (music->stopped) return music->last_error;
  const int8_t result = Audio_PlayerUpdate(&music->player, now_ms);
  if (result != AUDIO_OK) return Finish(music, result);
  if (music->player.active) return AUDIO_OK;
  if (music->startup_active) {
    music->startup_active = false;
    if (music->config.playlist_length > 0) return PlayIndex(music, now_ms);
  } else if (music->playlist_index + 1 < music->config.playlist_length) {
    ++music->playlist_index;
    return PlayIndex(music, now_ms);
  } else if (music->config.loop_playlist && music->config.playlist_length > 0) {
    music->playlist_index = 0;
    return PlayIndex(music, now_ms);
  }
  music->stopped = true;
  return AUDIO_OK;
}
int8_t CloudMusic_Next(CloudMusic_t* music, uint32_t now_ms) {
  if (!Ready(music) || music->config.playlist_length == 0) return AUDIO_ERR;
  music->playlist_index = (music->playlist_index + 1) % music->config.playlist_length;
  return PlayIndex(music, now_ms);
}
int8_t CloudMusic_Previous(CloudMusic_t* music, uint32_t now_ms) {
  if (!Ready(music) || music->config.playlist_length == 0) return AUDIO_ERR;
  music->playlist_index = music->playlist_index == 0
      ? music->config.playlist_length - 1 : music->playlist_index - 1;
  return PlayIndex(music, now_ms);
}
int8_t CloudMusic_SetPaused(CloudMusic_t* music, bool paused, uint32_t now_ms) {
  if (!Ready(music)) return AUDIO_ERR;
  music->paused = paused;
  if (!paused && music->stopped) {
    if (music->config.playlist_length == 0) return AUDIO_ERR;
    return PlayIndex(music, now_ms);
  }
  return Finish(music, Audio_PlayerSetPaused(&music->player, paused, now_ms));
}
int8_t CloudMusic_Stop(CloudMusic_t* music) {
  if (!Ready(music)) return AUDIO_ERR;
  music->stopped = true;
  music->startup_active = false;
  music->paused = false;
  return Finish(music, Audio_PlayerStop(&music->player));
}
int8_t CloudMusic_ApplyControl(CloudMusic_t* music,
                               const CloudMusic_Control_t* control, uint32_t now_ms) {
  if (!Ready(music) || control == NULL) return AUDIO_ERR;
  // Stop dominates simultaneous events; otherwise accept one event per call.
  if (control->stop_event) return CloudMusic_Stop(music);
  if (control->previous_event) return CloudMusic_Previous(music, now_ms);
  if (control->next_event) return CloudMusic_Next(music, now_ms);
  if (control->toggle_pause_event) return CloudMusic_SetPaused(music, !music->paused, now_ms);
  if (control->play_event) return CloudMusic_SetPaused(music, false, now_ms);
  return AUDIO_OK;
}
