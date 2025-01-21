#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

template <typename T> uint32_t convertTo32BitBinaryValue(T value) {
  uint32_t result = 0;

  if constexpr (std::is_same<T, uint8_t>::value || std::is_same<T, int8_t>::value) {
    result = static_cast<uint32_t>(value & 0xFF); // Mask to 8 bits and cast to uint32_t
  } else if constexpr (std::is_same<T, uint16_t>::value || std::is_same<T, int16_t>::value) {
    result = static_cast<uint32_t>(value & 0xFFFF); // Mask to 16 bits and cast to uint32_t
  } else if constexpr (std::is_same<T, uint32_t>::value || std::is_same<T, int32_t>::value) {
    result = static_cast<uint32_t>(value); // Already 32 bits, cast directly
  } else if constexpr (std::is_same<T, float>::value) {
    union {
      float f;
      uint32_t i;
    } u;
    u.f = value;
    result = u.i; // Interpret the float as uint32_t
  }

  return result;
}
#endif