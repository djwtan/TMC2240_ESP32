#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>

inline float curveP1(double n, long x, float min) { return (float)(n * x * x + min); }

inline float curveP2(double n, long x, float max) { return (float)(-n * x * x + max); }

#endif
