#include "Stepper.h"

Stepper::Stepper(uint8_t id) : pri_id(id) {}

/* ================================================================================== */
/*                                         SET                                        */
/* ================================================================================== */
void Stepper::ConfigurePin(PinConfig pin) {
  m_pinConfig = pin;
  pinMode(m_pinConfig.EN_PIN, OUTPUT);
  pinMode(m_pinConfig.STEP_PIN, OUTPUT);
  pinMode(m_pinConfig.DIR_PIN, OUTPUT);
  pinMode(m_pinConfig.CS_PIN, OUTPUT);
  pinMode(m_pinConfig.HOME_SENSOR_PIN, INPUT);
}

void Stepper::InitSPI(TMC2240_SPI *tmc2240spi) { m_spi = tmc2240spi; }

void Stepper::Initialize(bool *result) {
  uint32_t ms;
  const uint8_t MRES_BIT = 24;

  this->_RegWrite(0x6C, 0x00000000 | (ms << MRES_BIT) | 0x00); // disable driver

  switch (microstep) {
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
  this->_RegWrite(0x10, 0x00060000 | ((uint32_t)(runningCurrent & 0x1F) << 8) | ((uint32_t)holdingCurrent & 0x1F));

  uint32_t data;
  uint8_t status;
  this->_RegRead(0x6C, &data, &status);

  if ((data & 0x0000000F) != Toff) {
    if (result != nullptr)
      *result = false;
    enabled = false;
  } else {
    if (result != nullptr)
      *result = true;
    enabled = true;
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
  case REG_STOP_ON_STALL:
    result = stopOnStall ? "stopOnStall is true" : "stopOnStall is false";
    break;
  case REG_MICROSTEPPING:
    result = String(microstep);
    break;
  case REG_RUNNING_CURRENT:
    result = String(runningCurrent);
    break;
  case REG_HOLDING_CURRENT_PERCENTAGE:
    result = String(holdingCurrentPercentage);
    break;
  case REG_DISABLE_STEPPER:
    result = NO_READ_REGISTER;
    break;
  case REG_STALL_VALUE:
    result = String(this->ReadStallValue());
    break;
  case REG_HOMING_METHOD:
    result = String(get_HomingMethod(homingMethod));
    break;
  case REG_HOMING_SENSOR_TRIGGER_VALUE:
    result = sensorHomeValue ? "high" : "low";
    break;
  case REG_REQUEST_HOMING:
    result = runHoming ? "yes" : "no";
    break;
  case REG_HOMED:
    result = homed ? "yes" : "no";
    break;
  default:
    result = INVALID_REGISTER;
  }

  // return result;
  return _GenerateMessage() + result;
}

float Stepper::ReadTemperature() {
  uint8_t status;
  uint32_t data;
  this->_RegRead(0x51, &data, &status);

  return (float)((uint16_t)(data & 0x00001FFF) - 2038) / 7.7;
}

uint16_t Stepper::ReadStallValue() {
  uint8_t status;
  uint32_t data;
  this->_RegRead(0x6F, &data, &status);

  return (uint16_t)(data & 0x000003FF);
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
  case REG_STOP_ON_STALL:
    result = this->SetStopOnStall(data);
    break;
  case REG_MICROSTEPPING:
    result = this->SetMicrostepping(data);
    break;
  case REG_RUNNING_CURRENT:
    result = this->SetRunningCurrent(data);
    break;
  case REG_HOLDING_CURRENT_PERCENTAGE:
    result = this->SetHoldingCurrentPercentage(data);
    break;
  case REG_DISABLE_STEPPER:
    result = this->DisableStepper();
    break;
  case REG_STALL_VALUE:
    result = NO_WRITE_REGISTER;
    break;
  case REG_HOMING_METHOD:
    result = this->SetHomingMethod(data);
    break;
  case REG_HOMING_SENSOR_TRIGGER_VALUE:
    result = this->SetHomingSensorTriggerValue(data);
    break;
  case REG_REQUEST_HOMING:
    result = this->RequestHoming(data);
    break;
  case REG_HOMED:
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
  case OpMode::POSITION:
    switch (posCmdMode) {
    case PosCmdMode::ABSOLUTE:
      targetPOSHold = pos;
      break;
    case PosCmdMode::RELATIVE:
      targetPOSHold = currentPOS + pos;
      break;
    }
    break;

  case OpMode::VELOCITY:
    targetPOSHold += pos >= 0 ? DUMMY_POSITIVE : DUMMY_NEGATIVE; // dummy value
  }

  return "target position " + String(previousValue) + " -> " + String(targetPOSHold);
}

String Stepper::WriteCurrentPosition(int32_t pos) {
  uint32_t previousValue = currentPOS;

  // switch (opMode) {
  // case OpMode::POSITION:
  //   switch (posCmdMode) {
  //   case PosCmdMode::ABSOLUTE:
  //     targetPOS = pos;
  //     break;
  //   case PosCmdMode::RELATIVE:
  //     targetPOS = currentPOS + pos;
  //     break;
  //   }
  //   minRPM = (targetRPM > RPMThresh) ? minRPMFast : minRPMSlow; // min RPM corrector
  //   break;

  // case OpMode::VELOCITY:
  //   targetPOS = pos >= 0 ? DUMMY_POSITIVE : DUMMY_NEGATIVE; // dummy value
  //   minRPM = minRPMSlow;
  // }

  // return "target position " + String(previousValue) + " -> " + String(targetPOS);

  // targetPOS = pos;
  return "ignore first";
}

String Stepper::WriteTargetRPM(uint32_t rpm) {
  uint32_t previousValue = targetRPM;
  targetRPM_Hold = rpm;
  return "target rpm " + String(previousValue) + " -> " + String(targetRPM_Hold);
}

String Stepper::Move() {
  if (enabled) {
    // Assign Settings
    targetRPM = targetRPM_Hold;
    targetPOS = targetPOSHold;
    s_0 = currentPOS;
    v_0 = currentRPM;
    sTotal = _abs(targetPOS - s_0);

    // Compute Parameters
    this->_ComputeAccelerationParameters();
    this->_ComputeDeccelerationParameters(targetRPM);

    // Set time
    t_0 = micros();

    return "Move executed";
  }
  return "enable stepper first";
}

String Stepper::EmergencyStop() {
  digitalWrite(m_pinConfig.EN_PIN, HIGH); // releases axis
  enabled = false;
  return "stopped";
}

String Stepper::StopVelocity() {
  if ((opMode == OpMode::VELOCITY)) {
    if (currentRPM == targetRPM)
      currentPOS = targetPOS > 0 ? targetPOS - sDecel : targetPOS + sDecel;
    else if (currentRPM != 0) {
      this->_ComputeDeccelerationParameters(currentRPM);
      currentPOS = targetPOS > 0 ? targetPOS - sDecel : targetPOS + sDecel;
    }
    return "stop received";
  }
  return "not in velocity mode";
}

String Stepper::EnableStepper() {
  if (!enabled) {
    // Reset Motion Commands
    targetRPM = 0;
    targetPOS = 0;
    currentPOS = 0;
    currentRPM = 0;

    // Enable Driver
    digitalWrite(m_pinConfig.EN_PIN, LOW); // enable axis

    bool res;

    // Reinitialize
    this->Initialize(&res);

    if (res) { // Set flag to True
      return "stepper enabled";
    }
    return "failed to enable stepper";
  }
  return "stepper already enabled";
}

String Stepper::DisableStepper() {
  if (motorState == MotorState::IDLE) {
    (m_pinConfig.EN_PIN, HIGH);
    enabled = false;
    return "stepper disabled";
  }
  return "stepper can only be manually disabled in idle state";
}

String Stepper::SetOperationMode(uint32_t mode) {
  String previousValue = get_OperationMode(opMode);

  if (motorState == MotorState::IDLE) {
    switch (mode) {
    case 0:
      opMode = OpMode::POSITION;
      break;
    case 1:
      opMode = OpMode::VELOCITY;
      break;
    case 2:
      opMode = OpMode::INVERSE_TIME;
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

String Stepper::SetStopOnStall(uint32_t userInput) {
  switch (userInput) {
  case 0:
    stopOnStall = false;
    return "stopOnStall flag set to false";
  case 1:
    stopOnStall = true;
    return "stopOnStall flag set to true";
  default:
    return "invalid input";
  }
}

String Stepper::SetMicrostepping(uint32_t userInput) {
  if (motorState == MotorState::IDLE) {
    // if (0 < userInput <= 31) {
    //   runningCurrent = userInput;
    //   holdingCurrent = runningCurrent * holdingCurrentPercentage / 100;

    switch (userInput) {
    case 1:
      microstep = 1;
      break;
    case 2:
      microstep = 2;
      break;
    case 4:
      microstep = 4;
      break;
    case 8:
      microstep = 8;
      break;
    case 16:
      microstep = 16;
      break;
    case 32:
      microstep = 32;
      break;
    case 64:
      microstep = 64;
      break;
    case 128:
      microstep = 128;
      break;
    default:
      return "Microstep must be 2^n (max 128)";
    }

    bool res;

    // Reinitialize
    this->Initialize(&res);

    if (res) { // Set flag to True
      return "Microstep: " + String(microstep);
    }

    return "Failed to write microstep to driver";
  }
  return "Microstep can only be modified in idle mode";
}

String Stepper::SetRunningCurrent(uint32_t userInput) {
  if (motorState == MotorState::IDLE) {
    if (0 < userInput <= 31) {
      runningCurrent = userInput;
      holdingCurrent = runningCurrent * holdingCurrentPercentage / 100;

      bool res;

      // Reinitialize
      this->Initialize(&res);

      if (res) { // Set flag to True
        return "Running current: " + String(runningCurrent);
      }

      return "Failed to write running current to driver";
    }
    return "Value out of bounds";
  }
  return "Running current can only be modified in idle mode";
}

String Stepper::SetHoldingCurrentPercentage(uint32_t userInput) {
  if (motorState == MotorState::IDLE) {
    if (0 < userInput <= 50) {
      holdingCurrentPercentage = userInput;
      holdingCurrent = runningCurrent * holdingCurrentPercentage / 100;

      bool res;

      // Reinitialize
      this->Initialize(&res);

      if (res) { // Set flag to True
        return "Holding current percentage: " + String(holdingCurrentPercentage) + "%";
      }

      return "Failed to write holding current percentage to driver";
    }
    return "Value out of bounds";
  }
  return "Holding current percentage can only be modified in idle mode";
}

String Stepper::SetHomingMethod(uint32_t userInput) {
  switch (userInput) {
  case 0:
    homingMethod = HomingMethod::IMMEDIATE;
    return "homing method set to immediate";
  case 1:
    homingMethod = HomingMethod::TORQUE;
    return "homing method set to torque";
  case 2:
    homingMethod = HomingMethod::SENSOR;
    return "homing method set to sensor";
  default:
    return "invalid input";
  }
}

String Stepper::SetHomingSensorTriggerValue(uint32_t userInput) {
  switch (userInput) {
  case 0:
    sensorHomeValue = false;
    return "sensor home set to LOW";
  case 1:
    sensorHomeValue = true;
    return "sensor home set to HIGH";
  default:
    return "invalid input";
  }
}

String Stepper::RequestHoming(uint32_t userInput) {
  switch (userInput) {
  case 0:
    runHoming = false;
    return "Homing request cancelled";
  case 1:
    homed = false; // reset home flag
    runHoming = true;
    return "Homing requested. Method: " + get_HomingMethod(homingMethod) +
           (homingMethod == HomingMethod::SENSOR ? (sensorHomeValue ? "high" : "low") : "");
  default:
    return "invalid input";
  }
}

void Stepper::Run() {
  if (currentPOS != targetPOS) {
    /* ================================== Execute Step ================================== */
    digitalWrite(m_pinConfig.STEP_PIN, _step);

    _step = !_step;

    /* ================================= Update Position ================================ */
    switch (opMode) {
    case OpMode::POSITION:
      currentPOS += (direction ? 1 : -1);
      break;
    case OpMode::VELOCITY:
      if (sAbs >= sAcel && sAbs < sTotal - sDecel)
        currentPOS += 0;
      else
        currentPOS += (direction ? 1 : -1);
      break;
    }
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

void Stepper::_ComputeAccelerationParameters() {
  float highV = _max(targetRPM, v_0);
  float lowV = v_0 < targetRPM ? _max(minRPM, v_0) : targetRPM;

  /* ================================= Estimated step ================================= */
  sAcel = (targetRPM > minRPM && timeAcel_ms > 0)
              ? ((highV + lowV) * timeAcel_ms / 2) / (60.0 * 1e6) * (200 * microstep)
              : 0;

  /* ================================ Parameter (Time) ================================ */
  nAcel = 2 * (highV - lowV) / (double)(pow(timeAcel_ms, 2));
}

void Stepper::_ComputeDeccelerationParameters(float vmax) {

  /* ================================= Estimated Step ================================= */
  // FIXME: No idea why timeDecel_ms needs to be divided by 4 instead of 2
  sDecel = timeDecel_ms > 0 ? ((vmax + minRPM) * timeDecel_ms / 4) / (60.0 * 1e6) * (200 * microstep) : 0;

  sDecel = decelStepCorrection(sDecel, targetRPM);

  /* =============================== Parameter (Linear) =============================== */
  mDecel = (float)(targetRPM - minRPM) / sDecel;
}

unsigned long Stepper::ComputeTimePeriod() {

  /* ====================================== Move? ===================================== */
  if (currentPOS == targetPOS) {
    currentRPM = 0;

    // Reset to 0 on velocity mode
    if (opMode == OpMode::VELOCITY) {
      currentPOS = 0;
      targetPOS = 0;
    }

    uint8_t status;
    status = this->ReadStatus();

    if (status == 0 || status == 255) {
      this->EmergencyStop();
      this->_UpdateMotorState(MotorState::POWER_ERR);
    } else
      this->_UpdateMotorState(MotorState::IDLE);
    return 0;

  } else {

    /* ===================================== Homing? ==================================== */
    if (runHoming) {
      switch (homingMethod) {
      case HomingMethod::IMMEDIATE:
        currentPOS = 0;
        targetPOS = 0;
        homed = true;
        runHoming = false;
        break;

      case HomingMethod::TORQUE: {
        if (this->_IsStalled()) {
          currentPOS = 0;
          targetPOS = 0;
          homed = true;
          runHoming = false;
        }
        break;
      }

      case HomingMethod::SENSOR:
        if (digitalRead(m_pinConfig.HOME_SENSOR_PIN) == sensorHomeValue) {
          currentPOS = 0;
          targetPOS = 0;
          homed = true;
          runHoming = false;
        }
        break;
      }
    }

    /* =============================== Update Motor State =============================== */
    uint8_t status;
    status = this->ReadStatus();

    if (status == 0 || status == 255) {
      this->EmergencyStop();
      this->_UpdateMotorState(MotorState::POWER_ERR);
    } else if (this->_IsStalled() || (stopOnStall && motorState == MotorState::STALLED)) {
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

        peakRPM = currentRPM;
        recomputeParam = true;

        actualAcelTime = _dt;
        tDecel_0 = timeNow;

      } else {
        long _dt_decel = timeNow - tDecel_0;

        /* ================================== Decceleration ================================= */
        if (recomputeParam) {
          sDecelRecomputed = sTotal - sAbs;
          mDecel = (float)(peakRPM - minRPM) / sDecelRecomputed;
          recomputeParam = false;
        }
        long sAbs_decel = sDecelRecomputed - sTotal + sAbs;

        /* =================================== Y = -mx + c =================================== */
        currentRPM = peakRPM - mDecel * sAbs_decel;

        /* ===================== TEST: Compute actual decceleration time ===================== */
        actualDecelTime = _dt_decel;
      }
    }

    /* =================================== step delay =================================== */
    long pulseRateHz = (currentRPM * 200 * microstep) / 60;
    stepDelay = pulseRateHz == 0 ? 0 : 1000000 / pulseRateHz;

    return stepDelay;
  }
}

void Stepper::_RegWrite(const uint8_t address, const uint32_t data) {
  uint8_t buff[5] = {address | 0x80, (data >> 24) & 0xFF, (data >> 16) & 0xFF, (data >> 8) & 0xFF, data & 0xFF};
  m_spi->SPIExchange(buff, 5, pri_id);
}

void Stepper::_RegRead(const uint8_t address, uint32_t *data, uint8_t *status) {
  uint8_t buff[5];

  for (int i = 0; i < 2; i++) {
    buff[0] = address;
    buff[1] = 0x00;
    buff[2] = 0x00;
    buff[3] = 0x00;
    buff[4] = 0x00;
    m_spi->SPIExchange(buff, 5, pri_id);
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