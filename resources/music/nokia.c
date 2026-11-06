#include "music.h"
#include "score_macros.h"
static const Tone_t NOKIA[] = {
    {NOTE_E, 5, 125}, {NOTE_D, 5, 125}, {NOTE_FS, 4, 250}, {NOTE_GS, 4, 250},
    {NOTE_CS, 5, 125}, {NOTE_B, 4, 125}, {NOTE_D, 4, 250}, {NOTE_E, 4, 250},
    {NOTE_B, 4, 125}, {NOTE_A, 4, 125}, {NOTE_CS, 4, 250}, {NOTE_E, 4, 250},
    {NOTE_A, 4, 500}
};
const Audio_Score_t mr_music_nokia = {NOKIA, sizeof(NOKIA)/sizeof(Tone_t), MUSIC_TONE_GAP_MS};
