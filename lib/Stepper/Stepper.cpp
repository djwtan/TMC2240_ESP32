#include "Stepper.h"

Stepper::Stepper(uint8_t id) : pri_id(id), spiSettings(10000000, MSBFIRST, SPI_MODE3) {}

/* ================================================================================== */
/*                                         SET                                        */
/* ================================================================================== */
void Stepper::ConfigurePin(PinConfig pin) {
  m_pinConfig = pin;
  pinMode(m_pinConfig.EN_PIN, OUTPUT);
  pinMode(m_pinConfig.STEP_PIN, OUTPUT);
  pinMode(m_pinConfig.DIR_PIN, OUTPUT);
  pinMode(m_pinConfig.CS_PIN, OUTPUT);

  digitalWrite(m_pinConfig.CS_PIN, HIGH);
}

void Stepper::ConfigureMotor(MotorConfig mconfig) { m_motorConfig = mconfig; }

void Stepper::Initialize(bool *result) {
  uint32_t ms;
  const uint8_t MRES_BIT = 24;

  this->_RegWrite(0x6C, 0x00000000 | (ms << MRES_BIT) | 0x00); // disable driver

  switch (m_motorConfig.MICROSTEP) {
  case 128:
    ms = 0x1;
    break;
  case 64:
    ms = 0x2;
    break;
  case 32:
    ms = 0x3;
    break;
  case 16:
    ms = 0x4;
    break;
  case 8:
    ms = 0x5;
    break;
  case 4:
    ms = 0x6;
    break;
  case 2:
    ms = 0x7;
    break;
  case 1:
    ms = 0x8;
    break;
  default:
    ms = 0x0;
  }

  // Setting TOFF flag & microsteps...
  // this->_RegWrite(0x6C, 0x10410150 | (ms << MRES_BIT) | Toff);
  this->_RegWrite(0x6C, 0x30410150 | (ms << MRES_BIT) | Toff);

  // Setting running & ing current...
  this->_RegWrite(0x10, 0x00060000 | ((uint32_t)(m_motorConfig.RUNNING_CURRENT & 0x1F) << 8) |
                            ((uint32_t)m_motorConfig.HOLDING_CURRENT & 0x1F));

  uint32_t data;
  uint8_t status;
  this->_RegRead(0x6C, &data, &status);

  if ((data & 0x0000000F) != Toff) {
    if (result != nullptr)
      *result = false;
  } else {
    if (result != nullptr)
      *result = true;
    // this->_UpdateMotorState(MotorState::IDLE);
    swEN = true;
  };
}

/* ================================================================================== */
/*                                        READ                                        */
/* ================================================================================== */
String Stepper::HandleRead(uint8_t reg) {
  String result;

  switch (reg) {
  case REG_TARGET_POSITION:
    result = String(targetPOS);
    break;
  case REG_TARGET_RPM:
    result = String(targetRPM);
    break;
  case REG_MOVE:
    result = NO_READ_REGISTER;
    break;
  case REG_TEMPERATURE:
    result = String(this->ReadTemperature());
    break;
  case REG_DRV_STATUS:
    result = String(this->ReadStatus());
    break;
  case REG_MOTOR_STATUS:
    result = String(this->ReadMotorState());
    break;
  case REG_EMERGENCY_STOP:
    result = NO_READ_REGISTER;
    break;
  case REG_STOP_VELOCITY:
    result = NO_READ_REGISTER;
    break;
  case REG_ENABLE_STEPPER:
    result = NO_READ_REGISTER;
    break;
  case REG_OPERATION_MODE:
    result = get_OperationMode(opMode);
    break;
  case REG_ACEL_TIME:
    result = String(timeAcel_ms);
    break;
  case REG_DECEL_TIME:
    result = String(timeDecel_ms);
    break;
  case REG_CURRENT_RPM:
    result = String(currentRPM);
    break;
  case REG_CURRENT_POS:
    result = String(currentPOS);
    break;
  case REG_ACTUAL_ACCELERATION_TIME:
    result = String(actualAcelTime);
    break;
  case REG_ACTUAL_DECCELERATION_TIME:
    result = String(actualDecelTime);
    break;
  default:
    result = INVALID_REGISTER;
  }

  return result;
  // return _GenerateMessage() + result;
}

float Stepper::ReadTemperature() {
  uint8_t status;
  uint32_t data;
  this->_RegRead(0x51, &data, &status);

  return (float)((uint16_t)(data & 0x00001FFF) - 2038) / 7.7;
}

uint8_t Stepper::ReadStatus() {
  uint8_t status;
  uint32_t data;
  this->_RegRead(0x00, &data, &status);

  return status;
}

String Stepper::ReadMotorState() { return get_MotorState(motorState); }

/* ================================================================================== */
/*                                        WRITE                                       */
/* ================================================================================== */
String Stepper::HandleWrite(uint8_t reg, uint32_t data) {
  String result;

  switch (reg) {
  case REG_TARGET_POSITION:
    result = this->WriteTargetPosition((int32_t)data);
    break;
  case REG_TARGET_RPM:
    result = this->WriteTargetRPM(data);
    break;
  case REG_MOVE:
    result = this->Move();
    break;
  case REG_TEMPERATURE:
    result = NO_WRITE_REGISTER;
    break;
  case REG_DRV_STATUS:
    result = NO_WRITE_REGISTER;
    break;
  case REG_MOTOR_STATUS:
    result = NO_WRITE_REGISTER;
    break;
  case REG_EMERGENCY_STOP:
    result = this->EmergencyStop();
    break;
  case REG_STOP_VELOCITY:
    result = this->StopVelocity();
    break;
  case REG_ENABLE_STEPPER:
    result = this->EnableStepper();
    break;
  case REG_OPERATION_MODE:
    result = this->SetOperationMode(data);
    break;
  case REG_ACEL_TIME:
    result = this->SetAccelerationTime(data);
    break;
  case REG_DECEL_TIME:
    result = this->SetDeccelerationTime(data);
    break;
  case REG_CURRENT_RPM:
    result = NO_WRITE_REGISTER;
    break;
  case REG_CURRENT_POS:
    result = this->WriteCurrentPosition(data);
    break;
  case REG_ACTUAL_ACCELERATION_TIME:
    result = NO_WRITE_REGISTER;
    break;
  case REG_ACTUAL_DECCELERATION_TIME:
    result = NO_WRITE_REGISTER;
    break;
  default:
    result = INVALID_REGISTER;
  }
  return result;
  // return _GenerateMessage() + result;
}

String Stepper::WriteTargetPosition(int32_t pos) {
  uint32_t previousValue = targetPOS;

  switch (opMode) {
  case OperationMode::POSITION:
    switch (posMode) {
    case Mode_PositionCommand::ABSOLUTE:
      targetPOS = pos;
      break;
    case Mode_PositionCommand::RELATIVE:
      targetPOS = currentPOS + pos;
      break;
    }
    minRPM = (targetRPM > RPMThresh) ? minRPMFast : minRPMSlow; // min RPM corrector
    break;

  case OperationMode::VELOCITY:
    targetPOS = pos >= 0 ? DUMMY_POSITIVE : DUMMY_NEGATIVE; // dummy value
    minRPM = minRPMSlow;
  }

  return "target position " + String(previousValue) + " -> " + String(targetPOS);

  // targetPOS = pos;
  // return "pos done";
}

String Stepper::WriteCurrentPosition(int32_t pos) {
  // uint32_t previousValue = targetPOS;

  // switch (opMode) {
  // case OperationMode::POSITION:
  //   switch (posMode) {
  //   case Mode_PositionCommand::ABSOLUTE:
  //     targetPOS = pos;
  //     break;
  //   case Mode_PositionCommand::RELATIVE:
  //     targetPOS = currentPOS + pos;
  //     break;
  //   }
  //   minRPM = (targetRPM > RPMThresh) ? minRPMFast : minRPMSlow; // min RPM corrector
  //   break;

  // case OperationMode::VELOCITY:
  //   targetPOS = pos >= 0 ? DUMMY_POSITIVE : DUMMY_NEGATIVE; // dummy value
  //   minRPM = minRPMSlow;
  // }

  // return "target position " + String(previousValue) + " -> " + String(targetPOS);

  // targetPOS = pos;
  return "ignore first";
}

String Stepper::WriteTargetRPM(uint32_t rpm) {
  uint32_t previousValue = targetRPM;
  targetRPM = rpm;
  return "target rpm " + String(previousValue) + " -> " + String(targetRPM);

  // targetRPM = rpm;
  // return "rpm done";
}

String Stepper::Move() {
  //  Parameters
  s_0 = currentPOS;
  sTotal = _abs(targetPOS - s_0);

  this->_ComputeAccelerationParameters(currentRPM);
  this->_ComputeDeccelerationParameters(targetRPM);

  t_0 = micros();
  swEN = true;

  return "Move executed";
}

String Stepper::EmergencyStop() {
  digitalWrite(m_pinConfig.EN_PIN, HIGH); // releases axis
  swEN = false;
  targetRPM = 0;
  currentPOS = 0;
  return "emergency stop";
}

String Stepper::StopVelocity() {
  if ((opMode == OperationMode::VELOCITY)) {
    if (currentRPM != 0) {
      this->WriteTargetPosition(targetPOS);
      this->WriteTargetRPM(minRPMSlow + 1.0);
      this->Move();
    }
    return "stop received";
  }
  return "not in velocity mode";
}

String Stepper::EnableStepper() {
  targetRPM = 0;
  targetPOS = 0;
  currentPOS = targetPOS;
  this->Initialize();
  // this->_UpdateMotorState(MotorState::IDLE);
  digitalWrite(m_pinConfig.EN_PIN, LOW); // enable axis
  swEN = true;

  return "stepper enabled";
}

String Stepper::SetOperationMode(uint32_t mode) {
  String previousValue = get_OperationMode(opMode);

  if (motorState == MotorState::IDLE) {
    switch (mode) {
    case 0:
      opMode = OperationMode::POSITION;
      break;
    case 1:
      opMode = OperationMode::VELOCITY;
      break;
    case 2:
      opMode = OperationMode::INVERSE_TIME;
      break;
    default:
      return "invalid mode";
    }
    return previousValue + " -> " + get_OperationMode(opMode);
  }
  return "operation mode can only be toggled in idle state";
}

String Stepper::SetAccelerationTime(uint32_t millis) {
  long previousValue = timeAcel_ms;

  timeAcel_ms = (double)millis * 1000;

  return "time acel " + String(previousValue) + " -> " + String(timeAcel_ms);
}

String Stepper::SetDeccelerationTime(uint32_t millis) {
  long previousValue = timeDecel_ms;

  timeDecel_ms = (double)millis * 1000;

  return "time decel " + String(previousValue) + " -> " + String(timeDecel_ms);
}
/* ================================================================================== */
/*                                       ACTION                                       */
/* ================================================================================== */
void Stepper::ReleaseAxis() {
  if (motorState == MotorState::IDLE)
    (m_pinConfig.EN_PIN, HIGH);
}

void Stepper::Run() {
  /* ================================== Execute Step ================================== */
  digitalWrite(m_pinConfig.STEP_PIN, _step);

  _step = !_step;

  /* ================================= Update Position ================================ */
  switch (opMode) {
  case OperationMode::POSITION:
    currentPOS += (direction ? 1 : -1);
    break;
  case OperationMode::VELOCITY:
    currentPOS += sAbs >= sAcel ? 0 : (direction ? 1 : -1);
    break;
  }
}

void Stepper::MoveInverseTime() {
  /*
  Args: Number of steps, time taken
  Output: RPM, timeAcel, timeDecel
  */
}

/* ---------------------------------------------------------------------------------- */
String Stepper::_GenerateMessage() { return "s" + String(pri_id) + ": "; }

void Stepper::_UpdateMotorState(MotorState mState) { motorState = mState; }

bool Stepper::_IsStalled() {
  uint8_t status;
  uint32_t data;

  if (currentRPM < threshLow) {
    return false;
  } else if (currentRPM < threshHigh) {
    this->_RegRead(0x75, &data, &status);
    return data == 0;
  } else {
    this->_RegRead(0x00, &data, &status);
    return bitRead(status, 2);
  }
}

void Stepper::_ComputeAccelerationParameters(float v0) {
  float highV = _max(targetRPM, v0);
  float lowV = v0 < targetRPM ? _max(minRPM, v0) : targetRPM;

  /* ================================= Estimated step ================================= */
  sAcel = (targetRPM > minRPM && timeAcel_ms > 0)
              ? ((highV + lowV) * timeAcel_ms / 2) / (60.0 * 1e6) * (200 * m_motorConfig.MICROSTEP)
              : 0;

  /* ================================ Parameter (Time) ================================ */
  nAcel = 2 * (highV - lowV) / (double)(pow(timeAcel_ms, 2));
}

void Stepper::_ComputeDeccelerationParameters(float vmax) {

  /* ================================= Estimated Step ================================= */
  // FIXME: No idea why timeDecel_ms needs to be divided by 4 instead of 2
  sDecel = timeDecel_ms > 0 ? ((vmax + minRPM) * timeDecel_ms / 4) / (60.0 * 1e6) * (200 * m_motorConfig.MICROSTEP) : 0;

  sDecel = decelStepCorrection(sDecel, targetRPM);

  /* ============================ Parameter (Displacement) ============================ */
  // nDecel = 2 * (vmax - minRPM) / (double)(pow(sDecel, 2));

  /* ================================ Parameter (Time) ================================ */
  // nDecel = 2 * (vmax - minRPM) / (double)(pow(timeDecel_ms, 2));

  /* =============================== Parameter (Linear) =============================== */
  mDecel = (float)(targetRPM - minRPM) / sDecel;
}

void Stepper::_ComputeShortDeccelerationParameters(float vmax, uint32_t sRemaining) {
  /* ============================ Parameter (Displacement) ============================ */
  // nDecel = 2 * (vmax - minRPM) / (double)(pow(sRemaining, 2));
}

unsigned long Stepper::ComputeTimePeriod() {

  /* ====================================== Move? ===================================== */
  if (currentPOS == targetPOS) {
    this->_UpdateMotorState(MotorState::IDLE);
    currentRPM = 0;
    return 0;

    uint8_t status;
    status = this->ReadStatus();

    if (status == 0 || status == 255) {
      this->EmergencyStop();
      this->_UpdateMotorState(MotorState::POWER_ERR);
    }

  } else {
    /* =============================== Update Motor State =============================== */
    if (this->_IsStalled()) {
      if (stopOnStall)
        this->EmergencyStop();
      this->_UpdateMotorState(MotorState::STALLED);
    } else {
      this->_UpdateMotorState(MotorState::RUNNING);
    }

    /* ==================================== direction =================================== */
    direction = targetPOS > currentPOS;
    digitalWrite(m_pinConfig.DIR_PIN, direction);

    acelerating = targetRPM > currentRPM;
    sAbs = _abs(currentPOS - s_0);

    unsigned long timeNow = micros();
    long _dt = timeNow - t_0;

    /* ================================================================================== */
    /*                                       S-Curve                                      */
    /* ================================================================================== */
    if (sTotal > sDecel + sAcel) {
      /* ================================= Complete Motion ================================ */
      if (sAbs < (sTotal - sDecel)) {
        /* =============================== Acceleration Phase =============================== */
        if (sAbs < sAcel) {
          // if (sAbs <= sAcel / 2) {
          if (_dt <= timeDecel_ms / 2) {
            currentRPM = acelerating ? curveP1(nAcel, _dt, _max(minRPM, v_0)) : curveP2(nAcel, _dt, v_0);
          } else {
            long dS = _abs(_dt - timeAcel_ms);
            currentRPM = acelerating ? curveP2(nAcel, dS, targetRPM) : curveP1(nAcel, dS, _max(minRPM, targetRPM));
          }

          /* ===================== TEST: Compute actual acceleration time ===================== */
          actualAcelTime = _dt;
        } else {
          /* ================================= Constant Speed ================================= */
          currentRPM = targetRPM;
          tDecel_0 = timeNow;
        }
      } else {
        /* =============================== Decceleration Phase ============================== */
        long _dt_decel = timeNow - tDecel_0;
        long sAbs_decel = sAbs - (sTotal - sDecel);

        /* ===================================== S-Curve ==================================== */
        // if (sAbs_decel <= sDecel / 2) {
        //   currentRPM = curveP2(nDecel, sAbs_decel, targetRPM);
        // } else {
        //   long dS = _abs(sAbs_decel - sDecel);
        //   currentRPM = curveP1(nDecel, dS, minRPM);
        // }

        /* =================================== Y = -mx + c =================================== */
        currentRPM = targetRPM - mDecel * sAbs_decel;

        /* ===================== TEST: Compute actual decceleration time ===================== */
        actualDecelTime = _dt_decel;
      }

    } else {
      /* ================================ Incomplete motion =============================== */
      if (sAbs < (sTotal / 2)) {
        /* ================================== Acceleration ================================== */
        if (sAbs <= sAcel / 2) {
          currentRPM = acelerating ? curveP1(nAcel, _dt, _max(minRPM, v_0)) : curveP2(nAcel, _dt, v_0);
        } else {
          long dS = _abs(_dt - timeAcel_ms);
          currentRPM = acelerating ? curveP2(nAcel, dS, targetRPM) : curveP1(nAcel, dS, _max(minRPM, targetRPM));
        }

        RPMPeak = currentRPM;
        recomputeParam = true;

        actualAcelTime = _dt;
        tDecel_0 = timeNow;

      } else {
        long _dt_decel = timeNow - tDecel_0;

        /* ================================== Decceleration ================================= */
        if (recomputeParam) {
          sDecelRecomputed = sTotal - sAbs;
          // this->_ComputeShortDeccelerationParameters(RPMPeak, sDecelRecomputed);
          mDecel = (float)(RPMPeak - minRPM) / sDecelRecomputed;
          recomputeParam = false;
        }
        long sAbs_decel = sDecelRecomputed - sTotal + sAbs;

        /* ===================================== S-Curve ==================================== */

        // if (sAbs_decel <= sDecelRecomputed / 2) {
        //   currentRPM = curveP2(nDecel, sAbs_decel, RPMPeak);
        // } else {
        //   long dS = sAbs_decel - sDecelRecomputed;
        //   currentRPM = curveP1(nDecel, dS, minRPM);
        // }

        // Serial.println(currentRPM);
        /* =================================== Y = -mx + c =================================== */
        currentRPM = RPMPeak - mDecel * sAbs_decel;

        /* ===================== TEST: Compute actual decceleration time ===================== */
        actualDecelTime = _dt_decel;
      }
    }

    /* =================================== step delay =================================== */
    long pulseRateHz = (currentRPM * 200 * m_motorConfig.MICROSTEP) / 60;
    stepDelay = pulseRateHz == 0 ? 0 : 1000000 / pulseRateHz;

    return stepDelay;
  }
}

void Stepper::_RegWrite(const uint8_t address, const uint32_t data) {
  uint8_t buff[5] = {address | 0x80, (data >> 24) & 0xFF, (data >> 16) & 0xFF, (data >> 8) & 0xFF, data & 0xFF};
  this->_SPIExchange(buff, 5);
}

void Stepper::_RegRead(const uint8_t address, uint32_t *data, uint8_t *status) {
  uint8_t buff[5];

  for (int i = 0; i < 2; i++) {
    buff[0] = address;
    buff[1] = 0x00;
    buff[2] = 0x00;
    buff[3] = 0x00;
    buff[4] = 0x00;
    this->_SPIExchange(buff, 5);
  }

  /*
  -------- STATUS --------
  7 - last direction
  6 - don't care
  5 - don't care
  4 - don't care
  3 - standstill
  2 - stallguard active
  1 - driver_error
  0 - reset occurred

  all 1 or 0 -> Driver not powered
  ------------------------
  */
  *status = buff[0];
  *data = (buff[1] << 24) | (buff[2] << 16) | (buff[3] << 8) | buff[4];
}

void Stepper::_SPIExchange(uint8_t *data, const int size) {
  digitalWrite(m_pinConfig.CS_PIN, LOW);
  delayMicroseconds(1);
  SPI.beginTransaction(spiSettings);
  delayMicroseconds(1);

  SPI.transfer(data, size);
  delayMicroseconds(1);

  SPI.endTransaction();
  delayMicroseconds(1);
  digitalWrite(m_pinConfig.CS_PIN, HIGH);
  delayMicroseconds(1);
}
