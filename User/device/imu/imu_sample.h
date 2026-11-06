#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Board adapters convert sensor axes into the chosen body/NED convention
 * before publishing. The estimator assumes acceleration at rest is +Z.
 * A sample is a complete coherent frame, not independent latest axis values.
 */
typedef struct { float x, y, z; } MrVector3;
typedef struct {
  MrVector3 acceleration_mps2;
  MrVector3 angular_velocity_rad_s;
  uint64_t timestamp_us;  /* Same monotonic clock as the caller. */
  uint32_t sequence;      /* Changes for each frame, wrapping is allowed. */
  bool valid;
} MrImuSample;
