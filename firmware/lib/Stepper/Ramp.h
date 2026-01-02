#pragma once

#include <Arduino.h>
#include <algorithm>
#include <cstdint>
#include <math.h>

static inline float computePulseRate_trapezoidal(int32_t x, int32_t x_final, float v, float v_max,
                                                 float a, float d, long dt_ms) {

  // Convert dt to seconds
  float dt = dt_ms * 1e-3f;

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

struct SCurveResults {
  float v_now;
  float a_now;
};

static inline SCurveResults computePulseRate_scurve(int32_t x, int32_t x_final, float v,
                                                    float v_max, float a_max, float d_max, float ja,
                                                    float jd, float a_now, long dt_ms) {

  SCurveResults res{0, 0};

  // Convert dt to seconds
  float dt = dt_ms * 1e-3f;

  Serial.print(x);
  Serial.print(" | ");
  Serial.print(x_final);
  Serial.print(" | ");
  Serial.print(v);
  Serial.print(" | ");
  Serial.print(v_max);
  Serial.print(" | ");
  Serial.print(a_max);
  Serial.print(" | ");
  Serial.print(d_max);
  Serial.print(" | ");
  Serial.print(ja);
  Serial.print(" | ");
  Serial.print(jd);
  Serial.print(" | ");
  Serial.print(a_now);
  Serial.print(" | ");
  // Distance remaining
  uint32_t dx = abs(x_final - x);

  // Distance to start decelerating
  float t_j          = d_max / jd;
  float x_decel      = 1.9 * (v_max * v_max) / (2.0f * d_max) + (d_max * d_max) / (6.0f * jd * jd);
  float x_decel_half = x_decel / 2.0;

  // Update acceleration with jerk
  // | RampUp | RampDown | Constant | -RampUp | -RampDown |
  if (dx <= x_decel) {
    // if (dx <= x_decel_half) {
    if (v <= (v_max / 2)) {
      // 5th
      Serial.println("5");
      a_now += jd * dt;
    } else {
      // 4rd
      Serial.println("4");
      a_now -= jd * dt;
    }

    if (a_now > 0.0) a_now = 0.0;

  } else if (v < v_max) {
    if (v < v_max / 2.0) {
      // 1st
      Serial.println("1");
      a_now += ja * dt;
    } else {
      // 2nd
      Serial.println("2");
      a_now -= ja * dt;
    }
  } else {
    // 3rd
    Serial.println("3");
    a_now = 0;
  }

  // Bound
  a_now = std::min(a_max, a_now);
  a_now = std::max(-d_max, a_now);

  // Integrate velocity
  float v_now = v + a_now * dt;

  // Clamp velocity
  if (v_now > v_max) v_now = v_max;
  if (v_now < 0.0f) v_now = 0.0f;

  res.v_now = v_now;
  res.a_now = a_now;

  return res;
}
