#include "TMC2240_SPI.h"

TMC2240_SPI::TMC2240_SPI() : spiSettings(10000000, MSBFIRST, SPI_MODE3) {}

void TMC2240_SPI::RegisterCSPin(uint8_t num, uint8_t pin) {
  if (num >= 0 && num < MAX_STEPPER) {
    if (cspins[num] == NULL_PIN)
      cspins[num] = pin;
  }
}

void TMC2240_SPI::SPIExchange(uint8_t *data, const int size, uint8_t num) {

  for (int n = 0; n < MAX_STEPPER; n++) {
    if (n == num) {
      if (cspins[n] != NULL_PIN) {
        digitalWrite(cspins[n], LOW);
      }
    } else {
      if (cspins[n] != NULL_PIN) { // Correctly reference the loop variable 'n'
        digitalWrite(cspins[n], HIGH);
      }
    }
  };

  delayMicroseconds(1);
  SPI.beginTransaction(spiSettings);
  delayMicroseconds(1);

  SPI.transfer(data, size);
  delayMicroseconds(1);

  SPI.endTransaction();
  delayMicroseconds(1);

  for (int n = 0; n < MAX_STEPPER; n++) {
    if (cspins[n] != NULL_PIN)
      digitalWrite(cspins[num], HIGH);
  };

  delayMicroseconds(1);
}
