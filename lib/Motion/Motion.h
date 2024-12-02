#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>

inline float curveP1(double n, long x, float min) { return (float)(n * x * x + min); }

inline float curveP2(double n, long x, float max) { return (float)(-n * x * x + max); }

inline double decelStepCorrection(uint32_t s, float rpm) {
  /* ============================= Logarithmic correction ============================= */
  float c1 = (0.2108 * log(rpm) - 1.2824);

  /* ================================ Correction by RPM =============================== */
  float c2;

  if (rpm < 500)
    c2 = 2e-9 * pow(rpm, 3) - 3e-6 * pow(rpm, 2) + 0.0011 * rpm - 0.1728;
  else if (rpm > 550)
    c2 = -6e-5 * rpm + 0.0213;
  else
    c2 = 0;

  return s - (uint32_t)((c1 + c2) * s);
}

#endif
