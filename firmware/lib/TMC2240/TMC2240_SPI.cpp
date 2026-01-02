#include "TMC2240_SPI.h"

TMC2240_SPI::TMC2240_SPI() : spiSettings(10000000, MSBFIRST, SPI_MODE3) {}

void TMC2240_SPI::SPIExchange(uint8_t *data, size_t size, uint8_t csPin) {
  configASSERT(mutex);

  xSemaphoreTake(mutex, portMAX_DELAY);

  SPI.beginTransaction(spiSettings);
  digitalWrite(csPin, LOW);

  SPI.transfer(data, size);

  digitalWrite(csPin, HIGH);
  SPI.endTransaction();

  xSemaphoreGive(mutex);
}
