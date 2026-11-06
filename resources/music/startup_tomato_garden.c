#include "music.h"
#include "score_macros.h"
static const Tone_t STARTUP_TOMATO_GARDEN[] = {
    {NOTE_DS, 4, 300}, {NOTE_F, 4, 300}, {NOTE_G, 4, 450},
    {NOTE_AS, 4, 650}, {NOTE_REST, 0, 100},
    {NOTE_G, 4, 300}, {NOTE_F, 4, 300}, {NOTE_DS, 4, 450},
    {NOTE_AS, 3, 650}, {NOTE_REST, 0, 120},
    {NOTE_DS, 4, 300}, {NOTE_F, 4, 300}, {NOTE_G, 4, 450},
    {NOTE_F, 4, 300}, {NOTE_DS, 4, 900}
};
const Audio_Score_t mr_music_startup_tomato_garden = {STARTUP_TOMATO_GARDEN, sizeof(STARTUP_TOMATO_GARDEN)/sizeof(Tone_t), MUSIC_TONE_GAP_MS};
