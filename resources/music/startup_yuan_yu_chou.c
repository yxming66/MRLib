#include "music.h"
#include "score_macros.h"
static const Tone_t STARTUP_YUAN_YU_CHOU[] = {
    {NOTE_B, 5, 198}, {NOTE_C, 6, 198}, {NOTE_B, 5, 198},
    {NOTE_G, 5, 198}, {NOTE_E, 5, 198}, {NOTE_G, 5, 198},
    {NOTE_B, 5, 198}, {NOTE_C, 6, 198}, {NOTE_FS, 5, 198},
    {NOTE_FS, 4, 198}, {NOTE_A, 4, 198}, {NOTE_FS, 4, 198},
    {NOTE_FS, 5, 198}, {NOTE_G, 5, 198}, {NOTE_FS, 5, 198},
    {NOTE_D, 5, 198}, {NOTE_E, 5, 594}, {NOTE_REST, 0, 80},
    {NOTE_B, 4, 198}, {NOTE_D, 5, 198}, {NOTE_E, 5, 594}
};
const Audio_Score_t mr_music_startup_yuan_yu_chou = {STARTUP_YUAN_YU_CHOU, sizeof(STARTUP_YUAN_YU_CHOU)/sizeof(Tone_t), YUAN_YU_CHOU_TONE_GAP_MS};
