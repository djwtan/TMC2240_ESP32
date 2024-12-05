#ifndef TMC2240_SPI_H
#define TMC2240_SPI_H

#include "Define.h"
#include <Arduino.h>
#include <SPI.h>

#define NULL_PIN 0

class TMC2240_SPI {
public:
  TMC2240_SPI();

  uint8_t cspins[MAX_STEPPER] = {};

  void RegisterCSPin(uint8_t num, uint8_t pin);
  void SPIExchange(uint8_t *data, const int size, uint8_t num);

private:
  SPISettings spiSettings;
};

#endif // TMC2240_SPI