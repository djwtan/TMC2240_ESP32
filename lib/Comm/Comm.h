#ifndef COMM_H
#define COMM_H

#include "Stepper.h"
#include <Arduino.h>
#include <CRC32.h>

// Instruction type
#define INSTRUCTION_STEPPER_READ 0x00
#define INSTRUCTION_STEPPER_WRITE 0x01
#define INSTRUCTION_SYSTEM_READ 0x02
#define INSTRUCTION_SYSTEM_WRITE 0x03

#define BUFFER_SIZE 32 // arduino -8, esp32 -32

class Comm {
public:
  void init(Stream *serial);
  void readSerial(); // !loop
  void changeId();   // TODO
  void initStepper(uint8_t num, Stepper *stepper);

private:
  Stream *m_serial{nullptr};
  Stepper *steppers[MAX_STEPPER] = {nullptr};

  const uint8_t DEVICE_ID{0x01}; // TODO: write EEPROM
  const uint8_t START_BYTE{0xAA};
  const uint8_t END_BYTE{0x55};

  const unsigned long TIMEOUT = 1 * 1000; // 1 second

  // Read Bit
  void pri_read8(uint8_t *w);
  void pri_read32(uint32_t *w);

  // Checks
  bool pri_isStartByte(uint8_t sB);
  bool pri_isEndByte(uint8_t id);
  bool pri_isCorrectId(uint8_t id);
  bool pri_isCorrectCRC(uint32_t recvCRC, uint32_t bufData);

  // Map command
  void pri_readCmd(uint8_t *reg);
  void pri_writeCmd(uint8_t *reg, uint32_t *w);

  // Response
  void pri_execCmd(uint8_t instruction, uint8_t stepperId, uint8_t reg, uint32_t data);
};

#endif