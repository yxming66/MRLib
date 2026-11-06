#pragma once
#include <stdbool.h>
#include "module/cloudmusic/cloudmusic.h"
// Normalized app input: board/DR16 parsing and switch selection stay in app.
// Call once per scheduler cycle and feed the result to ApplyControl.
typedef struct { bool waiting_release; } MusicGesture_t;
static inline CloudMusic_Control_t MusicGestureUpdate(MusicGesture_t* gesture,
    bool menu_active, float x, float y) {
  CloudMusic_Control_t control = {0};
  if (!menu_active) { gesture->waiting_release = false; return control; }
  if (gesture->waiting_release) {
    if (x > -0.30f && x < 0.30f && y > -0.30f && y < 0.30f)
      gesture->waiting_release = false;
    return control;
  }
  if (x >= 0.75f) control.next_event = true;
  else if (x <= -0.75f) control.previous_event = true;
  else if (y >= 0.75f) control.play_event = true;
  else if (y <= -0.75f) control.toggle_pause_event = true;
  gesture->waiting_release = control.next_event || control.previous_event ||
                             control.play_event || control.toggle_pause_event;
  return control;
}
