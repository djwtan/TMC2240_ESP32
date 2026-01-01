#include "TMC2240_SPI.h"

TMC2240_SPI::TMC2240_SPI() : spiSettings(10000000, MSBFIRST, SPI_MODE3) {}

void TMC2240_SPI::SPIExchange(uint8_t *data, const int size, uint8_t pin_num) {

  digitalWrite(pin_num, LOW);

  delayMicroseconds(1);
  SPI.beginTransaction(spiSettings);
  delayMicroseconds(1);

  SPI.transfer(data, size);
  delayMicroseconds(1);

  SPI.endTransaction();
  delayMicroseconds(1);

  digitalWrite(pin_num, HIGH);

  delayMicroseconds(1);
}
