#pragma once

#include <cstdint>
#include <math.h>

static inline float computePulseRate_trapezoidal(int32_t x, int32_t x_final, float v, float v_max,
                                                 float a, float d, long dt_us) {

  // Convert dt to seconds
  float dt = dt_us * 1e-6f;

  // Distance remaining
  uint32_t dx = abs(x_final - x);

  // Distance to start decelerating
  // (1) v^2 = u^2 + 2as, where v = 0
  // (2) s = v_max / 2d
  float x_decel = (v * v) / (2.0f * d);

  float v_now = v;

  if (dx <= x_decel) {
    // Decelerate
    v_now -= d * dt;
  } else if (v < v_max) {
    // Accelerate
    v_now += a * dt;
  } else {
    // Cruise
    v_now = v_max;
  }

  return v_now;
}
