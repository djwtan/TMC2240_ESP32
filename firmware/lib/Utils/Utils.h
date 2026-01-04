#ifndef UTILS_H
#define UTILS_H

#include <cstdint>

static inline uint32_t floatTo32Bit(float value) {
  union {
    float    f;
    uint32_t i;
  } u;
  u.f = value;

  return u.i;
}

#endif