#include "Comm.h"

void Comm::init(Stream *serial) { m_serial = serial; }
void Comm::readSerial() {
  if (m_serial == nullptr)
    return;

  if (m_serial->available() > 0) {
    /*
    |-------------|-------------|-------------|-------------|-------------|-------------|-------------|------------|
    |     SB      |  DEVICE ID  | INSTRUCTION | STEPPER ID  |  REGISTER   | 32BIT DATA  |  32BIT CRC  |     EB     |
    |-------------|-------------|-------------|-------------|-------------|-------------|-------------|------------|
    */

    // Start Byte
    uint8_t startByte;
    this->pri_read8(&startByte);

    if (this->pri_isStartByte(startByte)) {
      uint8_t id;
      uint8_t instruction;
      uint8_t stepperId;
      uint8_t reg;
      uint32_t data;
      uint32_t receivedCrc;

      // Device ID
      this->pri_read8(&id);
      if (!this->pri_isCorrectId(id))
        return;

      // Instruction
      this->pri_read8(&instruction);

      // Stepper ID
      this->pri_read8(&stepperId);

      // Register
      this->pri_read8(&reg);

      // Read Data
      this->pri_read32(&data);

      // Read the CRC32 checksum
      this->pri_read32(&receivedCrc);

      // CRC Check
      if (!this->pri_isCorrectCRC(receivedCrc, data))
        return;

      // Read the end byte
      uint8_t endByte;
      this->pri_read8(&endByte);
      if (!this->pri_isEndByte(endByte))
        return;

      this->pri_execCmd(instruction, stepperId, reg, data);
    }
  }
}

void Comm::initStepper(uint8_t num, Stepper *stepper) {
  if (num >= 0 && num < MAX_STEPPER) {
    if (steppers[num] == nullptr)
      steppers[num] = stepper;
  }
}

/* ================================================================================== */
/*                                      Read Bit                                      */
/* ================================================================================== */
void Comm::pri_read8(uint8_t *w) {
  if (m_serial->available() > 0) {
    *w = m_serial->read();
  }
}
void Comm::pri_read32(uint32_t *w) {
  *w = 0;
  int bytesRead = 0;
  unsigned long startTime = millis();

  while (bytesRead < 4) {
    if (millis() - startTime > TIMEOUT) {
      break;
    }

    if (m_serial->available() > 0) {
      byte dataByte = m_serial->read();
      *w = (*w << 8) | dataByte;
      bytesRead++;
    }
  }
}

/* ================================================================================== */
/*                                       Checks                                       */
/* ================================================================================== */
bool Comm::pri_isStartByte(uint8_t sB) { return (sB == START_BYTE) ? true : false; }
bool Comm::pri_isEndByte(uint8_t eB) { return (eB == END_BYTE) ? true : false; }
bool Comm::pri_isCorrectId(uint8_t id) { return (id == DEVICE_ID) ? true : false; }
bool Comm::pri_isCorrectCRC(uint32_t recvCRC, uint32_t bufData) {
  byte data[4];
  data[0] = (bufData >> 24) & 0xFF;
  data[1] = (bufData >> 16) & 0xFF;
  data[2] = (bufData >> 8) & 0xFF;
  data[3] = bufData & 0xFF;

  // Calculate CRC32 checksum for received data
  uint32_t calculatedCrc = CRC32::calculate(data, sizeof(data));

  return (calculatedCrc == recvCRC) ? true : false;
}

/* ================================================================================== */
/*                                      Response                                      */
/* ================================================================================== */
void Comm::pri_execCmd(uint8_t instruction, uint8_t stepperId, uint8_t reg, uint32_t data) {
  if (stepperId < MAX_STEPPER && steppers[stepperId] != nullptr) {
    switch (instruction) {
    case INSTRUCTION_STEPPER_READ:
      m_serial->print(steppers[stepperId]->HandleRead(reg));
      break;
    case INSTRUCTION_STEPPER_WRITE:
      m_serial->print(steppers[stepperId]->HandleWrite(reg, data));
      break;
    case INSTRUCTION_SYSTEM_READ:
      // m_serial->print(steppers[stepperId]->HandleWrite(reg, data));
      break;
    case INSTRUCTION_SYSTEM_WRITE:
      // m_serial->print(steppers[stepperId]->HandleWrite(reg, data));
      break;
    default:
      m_serial->print("UNKNOWN INSTRUCTION: ");
      break;
    }
  }
}