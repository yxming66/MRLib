#pragma once
#include <stddef.h>
#include <stdint.h>

typedef enum {
  NOTE_C = 0,
  NOTE_CS = 1,
  NOTE_D = 2,
  NOTE_DS = 3,
  NOTE_E = 4,
  NOTE_F = 5,
  NOTE_FS = 6,
  NOTE_G = 7,
  NOTE_GS = 8,
  NOTE_A = 9,
  NOTE_AS = 10,
  NOTE_B = 11,
  NOTE_REST = 255
} NOTE_t;

typedef struct {
  NOTE_t note;
  uint8_t octave;
  uint16_t duration_ms;
} Tone_t;

typedef struct {
  const Tone_t* melody;
  size_t melody_length;
  uint16_t tone_gap_ms;
} Audio_Score_t;
