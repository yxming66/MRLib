#include "music.h"
#include "score_macros.h"
static const Tone_t RM[] = {
    {NOTE_B, 5, 200},
    {NOTE_G, 4, 200},
    {NOTE_B, 5, 400},
    {NOTE_G, 4, 200},
    {NOTE_B, 5, 400},
    {NOTE_G, 4, 200},
    {NOTE_D, 5, 400},
    {NOTE_G, 4, 200},
    {NOTE_C, 5, 200},
    {NOTE_C, 5, 200},
    {NOTE_G, 4, 200},
    {NOTE_B, 5, 200},
    {NOTE_C, 5, 200}
};
const Audio_Score_t mr_music_rm = {RM, sizeof(RM)/sizeof(Tone_t), MUSIC_TONE_GAP_MS};
