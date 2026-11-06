#include "module/audio/player.h"

static bool Ready(const Audio_Player_t* player) {
  return player != NULL && player->sink.set_tone != NULL;
}
static bool Reached(uint32_t now, uint32_t target) {
  return now - target <= INT32_MAX;
}
static int8_t Write(Audio_Player_t* player, NOTE_t note, uint8_t octave) {
  const int8_t result = player->sink.set_tone(player->sink.context, note, octave);
  if (result != AUDIO_OK) {
    player->active = false;
    player->paused = false;
    player->last_error = result;
    // Best effort silence; retain the original output error.
    (void)player->sink.set_tone(player->sink.context, NOTE_REST, 0);
  }
  return result;
}
bool Audio_ScoreIsValid(const Audio_Score_t* score) {
  if (score == NULL || score->melody == NULL || score->melody_length == 0) return false;
  for (size_t i = 0; i < score->melody_length; ++i) {
    const Tone_t* tone = &score->melody[i];
    if (tone->duration_ms == 0 || (tone->note != NOTE_REST &&
        ((uint32_t)tone->note > NOTE_B || tone->octave > 9))) return false;
  }
  return true;
}
int8_t Audio_PlayerInit(Audio_Player_t* player, Audio_Sink_t sink) {
  if (player == NULL || sink.set_tone == NULL) return AUDIO_ERR;
  *player = (Audio_Player_t){0};
  player->sink = sink;
  return AUDIO_OK;
}
int8_t Audio_PlayerStart(Audio_Player_t* player, const Audio_Score_t* score,
                         bool loop, bool paused, uint32_t now_ms) {
  if (!Ready(player) || !Audio_ScoreIsValid(score)) return AUDIO_ERR;
  player->score = *score;
  player->index = 0;
  player->active = true;
  player->paused = paused;
  player->gap = false;
  player->loop = loop;
  player->last_error = AUDIO_OK;
  player->deadline_ms = now_ms + score->melody[0].duration_ms;
  player->remaining_ms = score->melody[0].duration_ms;
  return Write(player, paused ? NOTE_REST : score->melody[0].note, score->melody[0].octave);
}
int8_t Audio_PlayerStop(Audio_Player_t* player) {
  if (!Ready(player)) return AUDIO_ERR;
  player->active = false;
  player->paused = false;
  player->gap = false;
  player->index = 0;
  return Write(player, NOTE_REST, 0);
}
int8_t Audio_PlayerUpdate(Audio_Player_t* player, uint32_t now_ms) {
  if (!Ready(player)) return AUDIO_ERR;
  if (!player->active || player->paused || !Reached(now_ms, player->deadline_ms))
    return player->last_error;
  if (!player->gap && player->score.tone_gap_ms > 0) {
    const int8_t result = Write(player, NOTE_REST, 0);
    if (result != AUDIO_OK) return result;
    player->gap = true;
    player->deadline_ms = now_ms + player->score.tone_gap_ms;
    return AUDIO_OK;
  }
  player->gap = false;
  if (++player->index == player->score.melody_length) {
    if (!player->loop) return Audio_PlayerStop(player);
    player->index = 0;
  }
  const Tone_t* tone = &player->score.melody[player->index];
  player->deadline_ms = now_ms + tone->duration_ms;
  return Write(player, tone->note, tone->octave);
}
int8_t Audio_PlayerSetPaused(Audio_Player_t* player, bool paused, uint32_t now_ms) {
  if (!Ready(player)) return AUDIO_ERR;
  if (!player->active || player->paused == paused) return player->last_error;
  if (paused) {
    player->remaining_ms = Reached(now_ms, player->deadline_ms)
        ? 0 : player->deadline_ms - now_ms;
    const int8_t result = Write(player, NOTE_REST, 0);
    if (result != AUDIO_OK) return result;
    player->paused = true;
    return AUDIO_OK;
  }
  player->paused = false;
  player->deadline_ms = now_ms + player->remaining_ms;
  const Tone_t* tone = &player->score.melody[player->index];
  const int8_t result = Write(player, player->gap ? NOTE_REST : tone->note, tone->octave);
  return result == AUDIO_OK ? Audio_PlayerUpdate(player, now_ms) : result;
}
