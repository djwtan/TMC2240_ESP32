#ifndef DEFINE_H
#define DEFINE_H

#include <Arduino.h>

#define MAX_STEPPER 4
#define ADDR_CONTROLLER_ID 0

// Response
#define GOOD_INSTRUCTION 0x01
#define BAD_INSTRUCTION 0x00
#define WRITE_SUCCESS 0x00000001
#define WRITE_FAIL 0x00000000
#define INVALID_REGISTER 0xFFFFFFFF

// COMM
enum class ReadBack_Key {
  TEMPERATURE,
  STATUS,
  MOTOR_STATE,
};

enum class MotorState {
  STALLED,
  OVERSPEED,
  IDLE,
  RUNNING,
  POWER_ERR,
  NOT_INIT,
};

enum class HomingMethod {
  IMMEDIATE,
  SENSOR,
  TORQUE,
};

static inline uint8_t get_MotorState(MotorState mState) {
  switch (mState) {
  case MotorState::STALLED:
    return 0;
  case MotorState::OVERSPEED:
    return 1;
  case MotorState::IDLE:
    return 2;
  case MotorState::RUNNING:
    return 3;
  case MotorState::POWER_ERR:
    return 4;
  case MotorState::NOT_INIT:
    return 5;
  default:
    return 6;
  }
};

static inline uint8_t get_HomingMethod(HomingMethod hMethod) {
  switch (hMethod) {
  case HomingMethod::IMMEDIATE:
    return 0;
  case HomingMethod::SENSOR:
    return 1;
  case HomingMethod::TORQUE:
    return 2;
  default:
    return 3;
  }
};

// DRIVER
enum class OpMode {
  POSITION,
  VELOCITY,
  INVERSE_TIME,
};

static inline uint8_t get_OperationMode(OpMode opMode) {
  switch (opMode) {
  case OpMode::POSITION:
    return 0;
  case OpMode::VELOCITY:
    return 1;
  case OpMode::INVERSE_TIME:
    return 2;
  default:
    return 3;
  }
}

enum class PositioningMode {
  RELATIVE,
  ABSOLUTE,
};

static inline uint8_t get_PositioningMode(PositioningMode posMode) {
  switch (posMode) {
  case PositioningMode::RELATIVE:
    return 0;
  case PositioningMode::ABSOLUTE:
    return 1;
  default:
    return 3;
  }
}

struct message {
  uint8_t instruction;
  uint8_t stepperId;
  uint8_t reg;
  uint32_t data;
};

struct response {
  uint8_t start_byte;
  uint8_t device_id;
  uint8_t validity;
  uint32_t result;
  uint32_t crc;
  uint8_t end_byte;
};

#endif