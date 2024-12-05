#ifndef DEFINE_H
#define DEFINE_H

#include <Arduino.h>

#define MAX_STEPPER 2
#define ADDR_CONTROLLER_ID 0

/* ---------------------------------------------------------------------------------- */
#define NO_READ_REGISTER "write only register"
#define NO_WRITE_REGISTER "read only register"
#define WRITE_SUCCESS "write success"
#define INVALID_REGISTER "invalid register"

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

inline String get_MotorState(MotorState mState) {
  switch (mState) {
  case MotorState::STALLED:
    return "stalled";
  case MotorState::OVERSPEED:
    return "overspeed";
  case MotorState::IDLE:
    return "idle";
  case MotorState::RUNNING:
    return "running";
  case MotorState::POWER_ERR:
    return "power_err";
  case MotorState::NOT_INIT:
    return "not_init";
  default:
    return "err";
  }
};

inline String get_HomingMethod(HomingMethod hMethod) {
  switch (hMethod) {
  case HomingMethod::IMMEDIATE:
    return "immediate";
  case HomingMethod::SENSOR:
    return "sensor";
  case HomingMethod::TORQUE:
    return "torque";
  default:
    return "err";
  }
};

// DRIVER
enum class OpMode {
  POSITION,
  VELOCITY,
  INVERSE_TIME,
};

inline String get_OperationMode(OpMode opMode) {
  switch (opMode) {
  case OpMode::POSITION:
    return "position mode";
  case OpMode::VELOCITY:
    return "velocity mode";
  case OpMode::INVERSE_TIME:
    return "inverse time mode";
  default:
    return "err";
  }
}

enum class PosCmdMode {
  RELATIVE,
  ABSOLUTE,
};

#endif