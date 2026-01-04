#ifndef TMC2240_SPI_H
#define TMC2240_SPI_H

#include <Arduino.h>
#include <SPI.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

class TMC2240_SPI {
public:
  TMC2240_SPI();

  void SPIExchange(uint8_t *data, size_t size, uint8_t pin_num);

private:
  SPISettings spiSettings;

  SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
};

#endif // TMC2240_SPI