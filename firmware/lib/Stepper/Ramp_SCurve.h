#pragma once

#include <algorithm>
#include <cstdint>
#include <math.h>

static inline float computePulseRate_scurve(int32_t x, int32_t x_final, float v, float v_max,
                                            float a, float d, float jerk, long dt_us) {
  // Convert dt to seconds
  float dt = dt_us * 1e-6f;

  // Distance remaining
  uint32_t dx = abs(x_final - x);

  // Distance to start decelerating
  float x_decel = (v * v) / (2.0f * d);

  // Update acceleration smoothly with jerk (jerk)
  if (dx <= x_decel) {
    // Deceleration phase
    a -= jerk * dt;
  } else if (v < v_max) {
    // Acceleration phase
    a += jerk * dt;
  } else {
    // Cruising (zero acceleration)
    if (a > 0) {
      a -= jerk * dt;
    } else if (a < 0) {
      a += jerk * dt;
    }
  }

  // Integrate velocity
  float v_now = v + a * dt;

  // Clamp velocity
  if (v_now > v_max) v_now = v_max;
  if (v_now < 0.0f) v_now = 0.0f;

  return v_now;
}
