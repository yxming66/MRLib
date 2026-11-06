#include <assert.h>
#include "module/cloudmusic/cloudmusic.h"
#include "remote_menu.h"
#include "music.h"
typedef struct { NOTE_t note; int count; int8_t error; } Sink;
static int8_t SetTone(void* context, NOTE_t note, uint8_t octave) {
  Sink* sink = context;
  (void)octave;
  sink->note = note;
  ++sink->count;
  return sink->error;
}
int main(void) {
  const Tone_t tones[] = {{NOTE_C, 4, 10}, {NOTE_D, 4, 20}};
  const Audio_Score_t score = {tones, 2, 3};
  const Tone_t one[] = {{NOTE_E, 4, 5}};
  const Audio_Score_t short_score = {one, 1, 0};
  const Audio_Score_t* playlist[] = {&short_score, &score};
  Sink sink = {0};
  Audio_Sink_t output = {&sink, SetTone};
  Audio_Player_t player = {0};
  assert(Audio_PlayerInit(&player, output) == 0);
  assert(Audio_PlayerStart(&player, &score, false, false, UINT32_MAX - 4) == 0);
  assert(sink.note == NOTE_C);
  assert(Audio_PlayerUpdate(&player, 4) == 0 && sink.note == NOTE_C);
  assert(Audio_PlayerUpdate(&player, 5) == 0 && sink.note == NOTE_REST);
  assert(Audio_PlayerUpdate(&player, 8) == 0 && sink.note == NOTE_D);
  assert(Audio_PlayerSetPaused(&player, true, 13) == 0 && sink.note == NOTE_REST);
  assert(Audio_PlayerUpdate(&player, 100) == 0 && player.paused);
  assert(Audio_PlayerSetPaused(&player, false, 100) == 0 && sink.note == NOTE_D);
  assert(Audio_PlayerUpdate(&player, 114) == 0 && sink.note == NOTE_D);
  assert(Audio_PlayerUpdate(&player, 115) == 0 && sink.note == NOTE_REST);
  assert(Audio_PlayerUpdate(&player, 118) == 0 && !player.active);
  assert(Audio_PlayerStart(&player, &score, false, false, 120) == 0);
  assert(Audio_PlayerUpdate(&player, 130) == 0 && player.gap);
  assert(Audio_PlayerSetPaused(&player, true, 131) == 0);
  assert(Audio_PlayerSetPaused(&player, false, 500) == 0 && sink.note == NOTE_REST);
  assert(Audio_PlayerUpdate(&player, 501) == 0 && player.gap);
  assert(Audio_PlayerUpdate(&player, 502) == 0 && sink.note == NOTE_D);
  assert(Audio_PlayerStart(&player, &short_score, true, false, 200) == 0);
  assert(Audio_PlayerUpdate(&player, 205) == 0 && player.active && sink.note == NOTE_E);
  assert(Audio_PlayerStop(&player) == 0 && !player.active);
  const Tone_t invalid_tone[] = {{NOTE_C, 4, 0}};
  const Audio_Score_t invalid = {invalid_tone, 1, 0};
  assert(!Audio_ScoreIsValid(&invalid));
  assert(Audio_PlayerStart(&player, &invalid, false, false, 0) != 0);
  CloudMusic_Config_t config = {playlist, 2, &short_score, false, false};
  CloudMusic_t music = {0};
  assert(CloudMusic_Init(&music, output, &config) == 0);
  assert(CloudMusic_Start(&music, 0) == 0 && music.startup_active);
  assert(CloudMusic_Update(&music, 5) == 0 && !music.startup_active);
  assert(music.playlist_index == 0);
  assert(CloudMusic_Update(&music, 10) == 0 && music.playlist_index == 1);
  CloudMusic_Control_t stop = {0};
  stop.stop_event = stop.next_event = stop.play_event = true;
  assert(CloudMusic_ApplyControl(&music, &stop, 11) == 0 && music.stopped);
  int count = sink.count;
  assert(CloudMusic_Update(&music, 1000) == 0 && sink.count == count);
  assert(CloudMusic_SetPaused(&music, false, 1001) == 0 && !music.stopped);
  assert(CloudMusic_Previous(&music, 1002) == 0 && music.playlist_index == 0);
  assert(CloudMusic_Previous(&music, 1003) == 0 && music.playlist_index == 1);
  sink.error = -7;
  assert(CloudMusic_Next(&music, 1004) == -7 && music.stopped);
  assert(CloudMusic_Update(&music, 1005) == -7);
  sink.error = 0;
  assert(CloudMusic_Start(&music, 1006) == 0);
  config.startup = NULL;
  config.start_paused = true;
  assert(CloudMusic_Stop(&music) == 0);
  assert(CloudMusic_Init(&music, output, &config) == 0);
  assert(CloudMusic_Start(&music, 0) == 0 && music.paused && music.player.paused);
  assert(sink.note == NOTE_REST);
  assert(CloudMusic_SetPaused(&music, false, 100) == 0);
  config.playlist = NULL;
  config.playlist_length = 0;
  config.start_paused = false;
  assert(CloudMusic_Stop(&music) == 0);
  assert(CloudMusic_Init(&music, output, &config) == 0);
  assert(CloudMusic_Start(&music, 0) == 0 && music.stopped);
  const Audio_Score_t* short_playlist[] = {&short_score, &short_score};
  config.playlist = short_playlist;
  config.playlist_length = 2;
  assert(CloudMusic_Init(&music, output, &config) == 0);
  assert(CloudMusic_Start(&music, 0) == 0);
  assert(CloudMusic_Update(&music, 5) == 0 && music.playlist_index == 1);
  assert(CloudMusic_Update(&music, 10) == 0 && music.stopped);
  config.loop_playlist = true;
  assert(CloudMusic_Init(&music, output, &config) == 0);
  assert(CloudMusic_Start(&music, 0) == 0);
  assert(CloudMusic_Update(&music, 5) == 0 && music.playlist_index == 1);
  assert(CloudMusic_Update(&music, 10) == 0 && music.playlist_index == 0 && !music.stopped);
  sink.error = -8;
  assert(CloudMusic_Update(&music, 15) == -8 && music.stopped);
  assert(CloudMusic_Update(&music, 100) == -8); // Never skip failed hardware output.
  sink.error = 0;
  MusicGesture_t gesture = {0};
  assert(MusicGestureUpdate(&gesture, true, 1, 0).next_event);
  assert(!MusicGestureUpdate(&gesture, true, 1, 0).next_event);
  (void)MusicGestureUpdate(&gesture, true, 0, 0);
  assert(MusicGestureUpdate(&gesture, true, -1, 0).previous_event);
  // Linking these two files must not require any other score or a HAL stub.
  assert(Audio_ScoreIsValid(&mr_music_nokia) && Audio_ScoreIsValid(&mr_music_rm));
}
