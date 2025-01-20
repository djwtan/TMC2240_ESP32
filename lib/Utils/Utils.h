#ifndef UTILS_H
#define UTILS_H

#include <Arduino.h>

template <typename T> String convertTo32BitBinaryString(T value) {
  String binaryString = "";

  // Specialize for each type
  if constexpr (std::is_same<T, uint8_t>::value) {
    for (int i = 7; i >= 0; i--) {
      binaryString += String((value >> i) & 1);
    }
    // Pad with leading zeros to make it 32 bits
    while (binaryString.length() < 32) {
      binaryString = "0" + binaryString;
    }
  } else if constexpr (std::is_same<T, uint16_t>::value) {
    for (int i = 15; i >= 0; i--) {
      binaryString += String((value >> i) & 1);
    }
    // Pad with leading zeros to make it 32 bits
    while (binaryString.length() < 32) {
      binaryString = "0" + binaryString;
    }
  } else if constexpr (std::is_same<T, uint32_t>::value) {
    for (int i = 31; i >= 0; i--) {
      binaryString += String((value >> i) & 1);
    }
  } else if constexpr (std::is_same<T, float>::value) {
    union {
      float f;
      uint32_t i;
    } u;
    u.f = value;
    for (int i = 31; i >= 0; i--) {
      binaryString += String((u.i >> i) & 1);
    }
  } else if constexpr (std::is_same<T, int8_t>::value) {
    for (int i = 7; i >= 0; i--) {
      binaryString += String((value >> i) & 1);
    }
    // Pad with leading zeros to make it 32 bits
    while (binaryString.length() < 32) {
      binaryString = "0" + binaryString;
    }
  } else if constexpr (std::is_same<T, int16_t>::value) {
    for (int i = 15; i >= 0; i--) {
      binaryString += String((value >> i) & 1);
    }
    // Pad with leading zeros to make it 32 bits
    while (binaryString.length() < 32) {
      binaryString = "0" + binaryString;
    }
  } else if constexpr (std::is_same<T, int32_t>::value) {
    for (int i = 31; i >= 0; i--) {
      binaryString += String((value >> i) & 1);
    }
  }

  return binaryString;
}
#endif