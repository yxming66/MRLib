#include <assert.h>
#include <float.h>
#include <math.h>

#include "component/mixer.h"
int main(void) {
  Mixer_t mixer;
  MoveVector_t input = {.vx = 2, .vy = 0, .wz = 0};
  float output[4] = {7, 7, 7, 7};
  assert(Mixer_Init(&mixer, MIXER_PARLFIX4) == 0);
  assert(Mixer_Apply(&mixer, &input, output, 4, 3) == 0);
  assert(output[0] == -3 && output[1] == 3 && output[2] == 3 && output[3] == -3);
  assert(Mixer_Init(&mixer, MIXER_PARLFIX2) == 0);
  assert(Mixer_Apply(&mixer, &input, output, 2, 1) == 0);
  assert(output[0] == -1 && output[1] == 1 && output[2] == 3);
  assert(Mixer_Init(&mixer, MIXER_SINGLE) == 0);
  assert(Mixer_Apply(&mixer, &input, output, 1, .5f) == 0 && output[0] == .5f);
  assert(Mixer_Apply(&mixer, &input, output, 4, 1) != 0 && output[0] == 0 && output[3] == 0);
  assert(Mixer_Apply(&mixer, NULL, output, 1, 1) != 0);
  assert(Mixer_Apply(&mixer, &input, NULL, 1, 1) != 0);
  assert(Mixer_Apply(&mixer, &input, output, -1, 1) != 0);
  input.vx = NAN;
  assert(Mixer_Apply(&mixer, &input, output, 1, 1) != 0 && output[0] == 0);
  input.vx = 2;
  mixer.mode = (Mixer_Mode_t)99;
  assert(Mixer_Apply(&mixer, &input, output, 1, 1) != 0);
  assert(Mixer_Init(&mixer, MIXER_MECANUM) == 0);
  input.vx = input.vy = input.wz = FLT_MAX;
  assert(Mixer_Apply(&mixer, &input, output, 4, 1) != 0 && output[0] == 0 && output[3] == 0);
  return 0;
}
