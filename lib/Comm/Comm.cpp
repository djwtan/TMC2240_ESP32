#include "Comm.h"

void Comm::init(Stream *serial) { m_serial = serial; }
void Comm::readSerial() {
  /*   ┌─────────────────────────────────────────────────────────────────────────────────────────────────────────┐
   *   │   Format of the message transmitted over serial:                                                        │
   *   │                                                                                                         │
   *   │   ┌────────┬───────────┬─────────────┬────────────┬─────────────┬───────────────┬────────────┬────────┐ │
   *   │   │   SB   │ DEVICE ID │ INSTRUCTION │ STEPPER ID │   REGISTER  │   32-BIT DATA │   32-BIT   │   EB   │ │
   *   │   │        │           │             │            │             │               │    CRC     │        │ │
   *   │   ├────────┼───────────┼─────────────┼────────────┼─────────────┼───────────────┼────────────┼────────┤ │
   *   │   │ Start  │ Device ID │ Instruction │ Stepper ID │ Register    │ Data to send  │ Error      │  End   │ │
   *   │   │  Byte  │ 1 Byte    │   1 Byte    │   1 Byte   │ Address     │ or receive    │ Detection  │  Byte  │ │
   *   │   │        │           │             │            │             │               │  (CRC32)   │        │ │
   *   │   └────────┴───────────┴─────────────┴────────────┴─────────────┴───────────────┴────────────┴────────┘ │
   *   └─────────────────────────────────────────────────────────────────────────────────────────────────────────┘
   */
  if (m_serial == nullptr)
    return;

  if (m_serial->available() == 0)
    return;

  /* ================================ Start Byte Check ================================ */
  uint8_t startByte;
  this->pri_read8(&startByte);

  if (!this->pri_isStartByte(startByte))
    return;

  /* ================================= Device ID check ================================ */
  uint8_t id;
  this->pri_read8(&id);
  if (!this->pri_isCorrectId(id))
    return;

  /* ================================ Construct Message =============================== */
  message msg;

  this->pri_read8(&msg.instruction);
  this->pri_read8(&msg.stepperId);
  this->pri_read8(&msg.reg);
  this->pri_read32(&msg.data);

  /* ==================================== CRC Check =================================== */
  uint32_t receivedCrc;
  // Read the CRC32 checksum
  this->pri_read32(&receivedCrc);

  if (!this->pri_isCorrectCRC(receivedCrc, msg.data))
    return;

  /* ================================= End Byte Check ================================= */
  uint8_t endByte;
  this->pri_read8(&endByte);
  if (!this->pri_isEndByte(endByte))
    return;

  this->pri_execCommand(msg);
}

void Comm::initStepper(uint8_t num, Stepper *stepper) {
  // max SPI = 128
  if (!(num >= 0 && num < MAX_STEPPER))
    return;

  // sanity check
  if (steppers[num] != nullptr)
    return;

  steppers[num] = stepper;
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
bool Comm::pri_isStartByte(uint8_t sB) { return sB == START_BYTE; }
bool Comm::pri_isEndByte(uint8_t eB) { return eB == END_BYTE; }
bool Comm::pri_isCorrectId(uint8_t id) { return id == DEVICE_ID; }
bool Comm::pri_isCorrectCRC(uint32_t recvCRC, uint32_t bufData) { return recvCRC == this->pri_computeCRC32(bufData); }
uint32_t Comm::pri_computeCRC32(uint32_t bufData) {
  byte data[4];
  data[0] = (bufData >> 24) & 0xFF; // MSB
  data[1] = (bufData >> 16) & 0xFF;
  data[2] = (bufData >> 8) & 0xFF;
  data[3] = bufData & 0xFF; // LSB

  // Calculate CRC32 checksum for received data
  uint32_t crc32 = CRC32::calculate(data, sizeof(data));

  return crc32;
}

/* ================================================================================== */
/*                                      Response                                      */
/* ================================================================================== */
void Comm::pri_execCommand(message msg) {
  /*   ┌───────────────────────────────────────────────────────────────────────────────┐
   *   │   Format of the message transmitted over serial:                              │
   *   │                                                                               │
   *   │   ┌────────┬───────────┬─────────────┬───────────────┬────────────┬────────┐  │
   *   │   │   SB   │ DEVICE ID │ INSTRUCTION │   32-BIT DATA │   32-BIT   │   EB   │  │
   *   │   │        │           │             │               │    CRC     │        │  │
   *   │   ├────────┼───────────┼─────────────┼───────────────┼────────────┼────────┤  │
   *   │   │ Start  │ Device ID │ Good /      │ Data to send  │ Error      │  End   │  │
   *   │   │  Byte  │ 1 Byte    │ Bad         │ or receive    │ Detection  │  Byte  │  │
   *   │   │        │           │ 1 Byte      │               │  (CRC32)   │        │  │
   *   │   └────────┴───────────┴─────────────┴───────────────┴────────────┴────────┘  │
   *   └───────────────────────────────────────────────────────────────────────────────┘
   */
  /* ================================= Create response ================================ */
  response response;
  response.start_byte = START_BYTE;
  response.device_id = DEVICE_ID;
  response.end_byte = END_BYTE;

  switch (msg.instruction) {
  case INSTRUCTION_STEPPER_READ:
    if (steppers[msg.stepperId] == nullptr)
      response.validity = BAD_INSTRUCTION;
    else {
      response.validity = GOOD_INSTRUCTION;
      response.result = steppers[msg.stepperId]->HandleRead(msg.reg);
    }
    break;
  case INSTRUCTION_STEPPER_WRITE:
    if (steppers[msg.stepperId] == nullptr)
      response.validity = BAD_INSTRUCTION;
    else {
      response.validity = GOOD_INSTRUCTION;
      response.result = steppers[msg.stepperId]->HandleWrite(msg.reg, msg.data);
    }
    break;
  case INSTRUCTION_SYSTEM_READ:
    response.validity = GOOD_INSTRUCTION;
    response.result = 0;
    break;
  case INSTRUCTION_SYSTEM_WRITE:
    response.validity = GOOD_INSTRUCTION;
    response.result = 0;
    break;
  default:
    response.validity = BAD_INSTRUCTION;
    response.result = 0;
    break;
  }

  /* =================================== Compute CRC ================================== */
  response.crc = this->pri_computeCRC32(response.result);

  /* ====================================== Send ====================================== */
  m_serial->write(response.start_byte);            // Start byte
  m_serial->write(response.device_id);             // Device ID
  m_serial->write(response.validity);              // Validity
  m_serial->write((response.result >> 24) & 0xFF); // Result (4 bytes)
  m_serial->write((response.result >> 16) & 0xFF);
  m_serial->write((response.result >> 8) & 0xFF);
  m_serial->write(response.result & 0xFF);
  m_serial->write((response.crc >> 24) & 0xFF); // CRC (4 bytes)
  m_serial->write((response.crc >> 16) & 0xFF);
  m_serial->write((response.crc >> 8) & 0xFF);
  m_serial->write(response.crc & 0xFF);
  m_serial->write(response.end_byte); // End byte
}