#ifndef TMC2240_SPI_H
#define TMC2240_SPI_H

#include <Arduino.h>
#include <SPI.h>

class TMC2240_SPI {
public:
  TMC2240_SPI();

  void SPIExchange(uint8_t *data, const int size, uint8_t pin_num);

private:
  SPISettings spiSettings;
};

#endif // TMC2240_SPI