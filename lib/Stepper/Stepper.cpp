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
    result = convertTo32BitBinaryString(targetPOS);
    break;
  case REG_TARGET_RPM:
    result = convertTo32BitBinaryString(targetRPM);
    break;
  case REG_TEMPERATURE:
    result = convertTo32BitBinaryString(this->ReadTemperature());
    break;
  case REG_DRV_STATUS:
    result = convertTo32BitBinaryString(this->ReadStatus());
    break;
  case REG_MOTOR_STATUS:
    result = convertTo32BitBinaryString(get_MotorState(motorState));
    break;
  case REG_OPERATION_MODE:
    result = convertTo32BitBinaryString(get_OperationMode(opMode));
    break;
  case REG_ACEL_TIME:
    result = convertTo32BitBinaryString(timeAcel_ms);
    break;
  case REG_DECEL_TIME:
    result = convertTo32BitBinaryString(timeDecel_ms);
    break;
  case REG_CURRENT_RPM:
    result = convertTo32BitBinaryString(currentRPM);
    break;
  case REG_CURRENT_POS:
    result = convertTo32BitBinaryString(currentPOS);
    break;
  case REG_ACTUAL_ACCELERATION_TIME:
    result = convertTo32BitBinaryString(actualAcelTime);
    break;
  case REG_ACTUAL_DECCELERATION_TIME:
    result = convertTo32BitBinaryString(actualDecelTime);
    break;
  case REG_STOP_ON_STALL:
    result = convertTo32BitBinaryString(stopOnStall ? 1 : 0);
    break;
  case REG_MICROSTEPPING:
    result = convertTo32BitBinaryString(microstep);
    break;
  case REG_RUNNING_CURRENT:
    result = convertTo32BitBinaryString(runningCurrent);
    break;
  case REG_HOLDING_CURRENT_PERCENTAGE:
    result = convertTo32BitBinaryString(holdingCurrentPercentage);
    break;
  case REG_STALL_VALUE:
    result = convertTo32BitBinaryString(this->ReadStallValue());
    break;
  case REG_HOMING_METHOD:
    result = convertTo32BitBinaryString(get_HomingMethod(homingMethod));
    break;
  case REG_HOMING_SENSOR_TRIGGER_VALUE:
    result = convertTo32BitBinaryString(sensorHomeValue ? 1 : 0);
    break;
  case REG_REQUEST_HOMING:
    result = convertTo32BitBinaryString(runHoming ? 1 : 0);
    break;
  case REG_HOMED:
    result = convertTo32BitBinaryString(homed ? 1 : 0);
    break;
  case REG_POSITIONING_MODE:
    result = convertTo32BitBinaryString(get_PositioningMode(posMode));
    break;
  default:
    result = convertTo32BitBinaryString(INVALID_REGISTER);
    break;
  }

  return result;
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

/* ================================================================================== */
/*                                        WRITE                                       */
/* ================================================================================== */
String Stepper::HandleWrite(uint8_t reg, uint32_t data) {
  uint32_t result;

  switch (reg) {
  case REG_TARGET_POSITION:
    result = this->SetTargetPosition((int32_t)data);
    break;
  case REG_TARGET_RPM:
    result = this->SetTargetRPM(data);
    break;
  case REG_MOVE:
    result = this->Move();
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
  case REG_CURRENT_POS:
    result = this->SetCurrentPosition(data);
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
  case REG_HOMING_METHOD:
    result = this->SetHomingMethod(data);
    break;
  case REG_HOMING_SENSOR_TRIGGER_VALUE:
    result = this->SetHomingSensorTriggerValue(data);
    break;
  case REG_REQUEST_HOMING:
    result = this->RequestHoming(data);
    break;
  case REG_POSITIONING_MODE:
    result = this->SetPositioningMode(data);
    break;
  default:
    result = INVALID_REGISTER;
  }
  return convertTo32BitBinaryString(result);
}

uint32_t Stepper::SetTargetPosition(int32_t pos) {
  uint32_t previousValue = targetPOS;

  switch (opMode) {
  case OpMode::POSITION:
    switch (posMode) {
    case PositioningMode::ABSOLUTE:
      targetPOSHold = pos;
      break;
    case PositioningMode::RELATIVE:
      targetPOSHold = currentPOS + pos;
      break;
    }
    break;

  case OpMode::VELOCITY:
    targetPOSHold += pos >= 0 ? DUMMY_POSITIVE : DUMMY_NEGATIVE; // dummy value
  }

  return WRITE_SUCCESS;
}

uint32_t Stepper::SetCurrentPosition(int32_t pos) {
  uint32_t previousValue = currentPOS;

  // switch (opMode) {
  // case OpMode::POSITION:
  //   switch (posMode) {
  //   case PositioningMode::ABSOLUTE:
  //     targetPOS = pos;
  //     break;
  //   case PositioningMode::RELATIVE:
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
  return WRITE_SUCCESS;
}

uint32_t Stepper::SetTargetRPM(uint32_t rpm) {
  targetRPM_Hold = rpm;
  return WRITE_SUCCESS;
}

uint32_t Stepper::Move() {
  if (!enabled)
    return WRITE_FAIL;

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

  return WRITE_SUCCESS;
}

uint32_t Stepper::EmergencyStop() {
  digitalWrite(m_pinConfig.EN_PIN, HIGH); // releases axis
  enabled = false;
  return WRITE_SUCCESS;
}

uint32_t Stepper::StopVelocity() {
  if (opMode != OpMode::VELOCITY)
    return WRITE_FAIL;

  if (currentRPM == targetRPM)
    currentPOS = targetPOS > 0 ? targetPOS - sDecel : targetPOS + sDecel;
  else if (currentRPM != 0) {
    this->_ComputeDeccelerationParameters(currentRPM);
    currentPOS = targetPOS > 0 ? targetPOS - sDecel : targetPOS + sDecel;
  }
  return WRITE_SUCCESS;
}

uint32_t Stepper::EnableStepper() {
  // Already enabled
  if (enabled)
    return WRITE_SUCCESS;

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
    return WRITE_SUCCESS;
  }
  return WRITE_FAIL;
}

uint32_t Stepper::DisableStepper() {
  // Can only disable in idle state
  if (motorState != MotorState::IDLE)
    return WRITE_FAIL;

  (m_pinConfig.EN_PIN, HIGH);
  enabled = false;
  return WRITE_SUCCESS;
}

uint32_t Stepper::SetOperationMode(uint32_t mode) {
  // Can only set in idle state
  if (motorState != MotorState::IDLE)
    return WRITE_FAIL;

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
    return WRITE_FAIL;
  }
  return WRITE_SUCCESS;
}

uint32_t Stepper::SetPositioningMode(uint32_t mode) {
  // Can only set in idle state
  if (motorState != MotorState::IDLE)
    return WRITE_FAIL;

  switch (mode) {
  case 0:
    posMode = PositioningMode::ABSOLUTE;
    break;
  case 1:
    posMode = PositioningMode::RELATIVE;
    break;
  default:
    return WRITE_FAIL;
  }
  return WRITE_SUCCESS;
}

uint32_t Stepper::SetAccelerationTime(uint32_t millis) {
  timeAcel_ms = (double)millis * 1000;

  return WRITE_SUCCESS;
}

uint32_t Stepper::SetDeccelerationTime(uint32_t millis) {
  timeDecel_ms = (double)millis * 1000;

  return WRITE_SUCCESS;
}

uint32_t Stepper::SetStopOnStall(uint32_t userInput) {
  switch (userInput) {
  case 0:
    stopOnStall = false;
    break;
  case 1:
    stopOnStall = true;
    break;
  default:
    return WRITE_FAIL;
  }
  return WRITE_SUCCESS;
}

uint32_t Stepper::SetMicrostepping(uint32_t userInput) {
  if (motorState != MotorState::IDLE)
    return WRITE_FAIL;

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
    return WRITE_FAIL;
  }

  bool res;

  // Reinitialize
  this->Initialize(&res);

  if (res) { // Set flag to True
    return WRITE_SUCCESS;
  }

  return WRITE_FAIL;
}

uint32_t Stepper::SetRunningCurrent(uint32_t userInput) {
  // Require idle
  if (motorState != MotorState::IDLE)
    return WRITE_FAIL;

  if (userInput <= 0 || userInput > 31)
    return WRITE_FAIL;

  runningCurrent = userInput;
  holdingCurrent = runningCurrent * holdingCurrentPercentage / 100;

  bool res;

  // Reinitialize
  this->Initialize(&res);

  if (res) { // Set flag to True
    return WRITE_SUCCESS;
  }

  return WRITE_FAIL;
}

uint32_t Stepper::SetHoldingCurrentPercentage(uint32_t userInput) {
  // Require idle
  if (motorState != MotorState::IDLE)
    return WRITE_FAIL;

  if (userInput <= 0 || userInput > 50)
    return WRITE_FAIL;

  holdingCurrentPercentage = userInput;
  holdingCurrent = runningCurrent * holdingCurrentPercentage / 100;

  bool res;

  // Reinitialize
  this->Initialize(&res);

  if (res) { // Set flag to True
    return WRITE_SUCCESS;
  }

  return WRITE_FAIL;
}

uint32_t Stepper::SetHomingMethod(uint32_t userInput) {
  switch (userInput) {
  case 0:
    homingMethod = HomingMethod::IMMEDIATE;
    break;
  case 1:
    homingMethod = HomingMethod::TORQUE;
    break;
  case 2:
    homingMethod = HomingMethod::SENSOR;
    break;
  default:
    return WRITE_FAIL;
  }
  return WRITE_SUCCESS;
}

uint32_t Stepper::SetHomingSensorTriggerValue(uint32_t userInput) {
  switch (userInput) {
  case 0:
    sensorHomeValue = false;
    break;
  case 1:
    sensorHomeValue = true;
    break;
  default:
    return WRITE_FAIL;
  }
  return WRITE_SUCCESS;
}

uint32_t Stepper::RequestHoming(uint32_t userInput) {
  switch (userInput) {
  case 0:
    runHoming = false;
    break;
  case 1:
    homed = false; // reset home flag
    runHoming = true;
    break;
  default:
    return WRITE_FAIL;
  }
  return WRITE_SUCCESS;
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