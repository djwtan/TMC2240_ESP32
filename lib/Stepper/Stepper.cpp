#include "Stepper.h"

Stepper::Stepper(uint8_t id, TMC2240_SPI *tmc2240spi, volatile bool *run, hw_timer_t *hwtimer,
                 portMUX_TYPE *timerMux)
    : m_id(id), m_spi(tmc2240spi), m_run(run), m_hwtimer(hwtimer), m_timerMux(timerMux) {}

/* ---------------------------------------------------------------------------------- */
void Stepper::ConfigurePin(PinConfig pin) {
  m_pinConfig = pin;
  pinMode(m_pinConfig.EN_PIN, OUTPUT);
  pinMode(m_pinConfig.STEP_PIN, OUTPUT);
  pinMode(m_pinConfig.DIR_PIN, OUTPUT);
  pinMode(m_pinConfig.CS_PIN, OUTPUT);
  pinMode(m_pinConfig.HOME_SENSOR_PIN, INPUT);

  // Default Modes
  digitalWrite(m_pinConfig.EN_PIN, LOW);
  digitalWrite(m_pinConfig.CS_PIN, HIGH);
}

/* ---------------------------------------------------------------------------------- */
void Stepper::Initialize(bool *result) {
  uint32_t      ms;
  const uint8_t MRES_BIT = 24;

  this->WriteRegister(0x6C, 0x00000000 | (ms << MRES_BIT) | 0x00); // disable driver

  switch (microstep) {
  case 128: ms = 0x1; break;
  case 64:  ms = 0x2; break;
  case 32:  ms = 0x3; break;
  case 16:  ms = 0x4; break;
  case 8:   ms = 0x5; break;
  case 4:   ms = 0x6; break;
  case 2:   ms = 0x7; break;
  case 1:   ms = 0x8; break;
  default:  ms = 0x0;
  }

  // CHOPCONF: configure microsteps + TOFF + enable interpolation
  // this->WriteRegister(TMC2240_Registers::CHOPCONF, 0x10410150 | (ms << MRES_BIT) | Toff);
  this->WriteRegister(TMC2240_Registers::CHOPCONF, 0x30410150 | (ms << MRES_BIT) | Toff);

  // IHOLD_IRUN: run current, hold current, hold delay.
  this->WriteRegister(TMC2240_Registers::IHOLD_IRUN, 0x00060000 |
                                                         ((uint32_t)(runningCurrent & 0x1F) << 8) |
                                                         ((uint32_t)(holdingCurrent & 0x1F)));

  // TPOWERDOWN: delay before switching to hold current (in ~1.6s units)
  this->WriteRegister(TMC2240_Registers::TPOWERDOWN, 10); // e.g. 10 = ~16s

  // TPWMTHRS: threshold for switching from stealthChop to spreadCycle
  // todo: set threshold
  this->WriteRegister(TMC2240_Registers::TPWMTHRS, 0xFFFFF); // Use stealthChop for all speeds

  // GCONF: enable StealthChop and diagnostic output config if needed
  this->WriteRegister(TMC2240_Registers::GCONF, 0x00000004); // en_pwm_mode

  // PWMCONF: configure StealthChop
  this->WriteRegister(TMC2240_Registers::PWMCONF, 0x00050480); // conservative default

  uint32_t data;
  uint8_t  status;
  this->ReadRegister(0x6C, &data, &status);

  if ((data & 0x0000000F) != Toff) {
    if (result != nullptr) *result = false;
    enabled = false;
  } else {
    if (result != nullptr) *result = true;
    if (!m_tasksStarted) {
      xTaskCreate(Stepper::task_ComputeRampParam, // function name
                  "Compute Ramp",                 // task name
                  200,                            // stack size
                  this,                           // task parameters
                  1,                              // task priority
                  NULL                            // task handle
      );
      xTaskCreate(Stepper::task_UpdateStatus, // function name
                  "Update Status",            // task name
                  200,                        // stack size
                  this,                       // task parameters
                  1,                          // task priority
                  NULL                        // task handle
      );
      m_tasksStarted = true;
    }

    enabled = true;
  };
}

/* ================================================================================== */
/*                                        READ                                        */
/* ================================================================================== */
uint32_t Stepper::HandleRead(uint8_t reg) {
  uint32_t result;

  switch (reg) {
  case S_Reg::TARGET_POSITION:             result = to32Bit(targetPOS); break;
  case S_Reg::TARGET_RPM:                  result = to32Bit(targetRPM); break;
  case S_Reg::TEMPERATURE:                 result = to32Bit(this->ReadTemperature()); break;
  case S_Reg::DRV_STATUS:                  result = to32Bit(this->ReadStatus()); break;
  case S_Reg::MOTOR_STATUS:                result = static_cast<uint32_t>(m_status); break;
  case S_Reg::OPERATION_MODE:              result = static_cast<uint32_t>(opMode); break;
  case S_Reg::ACEL_TIME:                   result = to32Bit(timeAcel_ms); break;
  case S_Reg::DECEL_TIME:                  result = to32Bit(timeDecel_ms); break;
  case S_Reg::CURRENT_RPM:                 result = to32Bit(currentRPM); break;
  case S_Reg::CURRENT_POS:                 result = to32Bit(currentPOS); break;
  case S_Reg::ACTUAL_ACCELERATION_TIME:    result = to32Bit(actualAcelTime); break;
  case S_Reg::ACTUAL_DECCELERATION_TIME:   result = to32Bit(actualDecelTime); break;
  case S_Reg::STOP_ON_STALL:               result = to32Bit(stopOnStall ? 1 : 0); break;
  case S_Reg::MICROSTEPPING:               result = to32Bit(microstep); break;
  case S_Reg::RUNNING_CURRENT:             result = to32Bit(runningCurrent); break;
  case S_Reg::HOLDING_CURRENT_PERCENTAGE:  result = to32Bit(holdingCurrentPercentage); break;
  case S_Reg::STALL_VALUE:                 result = to32Bit(this->ReadStallValue()); break;
  case S_Reg::HOMING_METHOD:               result = static_cast<uint32_t>(homingMethod); break;
  case S_Reg::HOMING_SENSOR_TRIGGER_VALUE: result = to32Bit(sensorHomeValue ? 1 : 0); break;
  case S_Reg::REQUEST_HOMING:              result = to32Bit(runHoming ? 1 : 0); break;
  case S_Reg::HOMED:                       result = to32Bit(homed ? 1 : 0); break;
  case S_Reg::POSITIONING_MODE:            result = static_cast<uint32_t>(posMode); break;
  default:                                 result = to32Bit(INVALID_REGISTER); break;
  }

  return result;
}

/* ---------------------------------------------------------------------------------- */
float Stepper::ReadTemperature() {
  uint8_t  status;
  uint32_t data;
  this->ReadRegister(TMC2240_Registers::TEMPERATURE, &data, &status);

  return (float)((uint16_t)(data & 0x00001FFF) - 2038) / 7.7;
}

/* ---------------------------------------------------------------------------------- */
uint16_t Stepper::ReadStallValue() {
  uint8_t  status;
  uint32_t data;
  this->ReadRegister(TMC2240_Registers::SG_RESULT_IND, &data, &status);

  return data;
}

/* ---------------------------------------------------------------------------------- */
uint8_t Stepper::ReadStatus() {
  uint8_t  status;
  uint32_t data;
  this->ReadRegister(TMC2240_Registers::GCONF, &data, &status);

  return status;
}

/* ================================================================================== */
/*                                        WRITE                                       */
/* ================================================================================== */
uint32_t Stepper::HandleWrite(uint8_t reg, uint32_t data) {
  uint32_t result;

  switch (reg) {
  case S_Reg::TARGET_POSITION:             result = this->SetTargetPosition((int32_t)data); break;
  case S_Reg::TARGET_RPM:                  result = this->SetTargetRPM(data); break;
  case S_Reg::MOVE:                        result = this->Move(); break;
  case S_Reg::EMERGENCY_STOP:              result = this->EmergencyStop(); break;
  case S_Reg::STOP_VELOCITY:               result = this->StopVelocity(); break;
  case S_Reg::ENABLE_STEPPER:              result = this->EnableStepper(); break;
  case S_Reg::OPERATION_MODE:              result = this->SetOperationMode(data); break;
  case S_Reg::ACEL_TIME:                   result = this->SetAccelerationTime(data); break;
  case S_Reg::DECEL_TIME:                  result = this->SetDeccelerationTime(data); break;
  case S_Reg::CURRENT_POS:                 result = this->SetCurrentPosition(data); break;
  case S_Reg::STOP_ON_STALL:               result = this->SetStopOnStall(data); break;
  case S_Reg::MICROSTEPPING:               result = this->SetMicrostepping(data); break;
  case S_Reg::RUNNING_CURRENT:             result = this->SetRunningCurrent(data); break;
  case S_Reg::HOLDING_CURRENT_PERCENTAGE:  result = this->SetHoldingCurrentPercentage(data); break;
  case S_Reg::DISABLE_STEPPER:             result = this->DisableStepper(); break;
  case S_Reg::HOMING_METHOD:               result = this->SetHomingMethod(data); break;
  case S_Reg::HOMING_SENSOR_TRIGGER_VALUE: result = this->SetHomingSensorTriggerValue(data); break;
  case S_Reg::REQUEST_HOMING:              result = this->RequestHoming(data); break;
  case S_Reg::POSITIONING_MODE:            result = this->SetPositioningMode(data); break;
  default:                                 result = INVALID_REGISTER;
  }
  return result;
}

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::SetTargetPosition(int32_t pos) {
  uint32_t previousValue = targetPOS;

  switch (opMode) {
  case OpMode::POSITION:
    switch (posMode) {
    case PositioningMode::ABSOLUTE: targetPOSHold = pos; break;
    case PositioningMode::RELATIVE: targetPOSHold = currentPOS + pos; break;
    }
    break;

  case OpMode::VELOCITY: targetPOSHold += pos >= 0 ? DUMMY_POSITIVE : DUMMY_NEGATIVE; // dummy value
  }

  return WRITE_SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
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

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::SetTargetRPM(uint32_t rpm) {
  targetRPM_Hold = rpm;
  return WRITE_SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::Move() {
  if (!enabled) return WRITE_FAIL;

  // Assign Settings
  targetRPM = targetRPM_Hold;
  targetPOS = targetPOSHold;
  s_0       = currentPOS;
  v_0       = currentRPM;
  sTotal    = _abs(targetPOS - s_0);

  // Compute Parameters
  this->_ComputeAccelerationParameters();
  this->_ComputeDeccelerationParameters(targetRPM);

  // Set time
  t_0 = micros();

  return WRITE_SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::EmergencyStop() {
  digitalWrite(m_pinConfig.EN_PIN, HIGH); // releases axis
  enabled = false;
  return WRITE_SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::StopVelocity() {
  if (opMode != OpMode::VELOCITY) return WRITE_FAIL;

  if (currentRPM == targetRPM)
    currentPOS = targetPOS > 0 ? targetPOS - sDecel : targetPOS + sDecel;
  else if (currentRPM != 0) {
    this->_ComputeDeccelerationParameters(currentRPM);
    currentPOS = targetPOS > 0 ? targetPOS - sDecel : targetPOS + sDecel;
  }
  return WRITE_SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::EnableStepper() {
  // Already enabled
  if (enabled) return WRITE_SUCCESS;

  // Reset Motion Commands
  targetRPM  = 0;
  targetPOS  = 0;
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

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::DisableStepper() {
  // Can only disable in idle state
  if (m_status != Status::IDLE) return WRITE_FAIL;

  digitalWrite(m_pinConfig.EN_PIN, HIGH);
  enabled = false;
  return WRITE_SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::SetOperationMode(uint32_t mode) {
  // Can only set in idle state
  if (m_status != Status::IDLE) return WRITE_FAIL;

  switch (mode) {
  case 0:  opMode = OpMode::POSITION; break;
  case 1:  opMode = OpMode::VELOCITY; break;
  case 2:  opMode = OpMode::INVERSE_TIME; break;
  default: return WRITE_FAIL;
  }
  return WRITE_SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::SetPositioningMode(uint32_t mode) {
  // Can only set in idle state
  if (m_status != Status::IDLE) return WRITE_FAIL;

  switch (mode) {
  case 0:  posMode = PositioningMode::ABSOLUTE; break;
  case 1:  posMode = PositioningMode::RELATIVE; break;
  default: return WRITE_FAIL;
  }
  return WRITE_SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::SetAccelerationTime(uint32_t millis) {
  timeAcel_ms = (double)millis * 1000;

  return WRITE_SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::SetDeccelerationTime(uint32_t millis) {
  timeDecel_ms = (double)millis * 1000;

  return WRITE_SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::SetStopOnStall(uint32_t userInput) {
  switch (userInput) {
  case 0:  stopOnStall = false; break;
  case 1:  stopOnStall = true; break;
  default: return WRITE_FAIL;
  }
  return WRITE_SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::SetMicrostepping(uint32_t userInput) {
  if (m_status != Status::IDLE) return WRITE_FAIL;

  switch (userInput) {
  case 1:   microstep = 1; break;
  case 2:   microstep = 2; break;
  case 4:   microstep = 4; break;
  case 8:   microstep = 8; break;
  case 16:  microstep = 16; break;
  case 32:  microstep = 32; break;
  case 64:  microstep = 64; break;
  case 128: microstep = 128; break;
  default:  return WRITE_FAIL;
  }

  bool res;

  // Reinitialize
  this->Initialize(&res);

  if (res) { // Set flag to True
    return WRITE_SUCCESS;
  }

  return WRITE_FAIL;
}

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::SetRunningCurrent(uint32_t userInput) {
  // Require idle
  if (m_status != Status::IDLE) return WRITE_FAIL;

  if (userInput <= 0 || userInput > 31) return WRITE_FAIL;

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

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::SetHoldingCurrentPercentage(uint32_t userInput) {
  // Require idle
  if (m_status != Status::IDLE) return WRITE_FAIL;

  if (userInput <= 0 || userInput > 100) return WRITE_FAIL;

  holdingCurrentPercentage = userInput;
  holdingCurrent           = runningCurrent * holdingCurrentPercentage / 100;

  bool res;

  // Reinitialize
  this->Initialize(&res);

  if (res) { // Set flag to True
    return WRITE_SUCCESS;
  }

  return WRITE_FAIL;
}

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::SetHomingMethod(uint32_t userInput) {
  switch (userInput) {
  case 0:  homingMethod = HomingMethod::IMMEDIATE; break;
  case 1:  homingMethod = HomingMethod::TORQUE; break;
  case 2:  homingMethod = HomingMethod::SENSOR; break;
  default: return WRITE_FAIL;
  }
  return WRITE_SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::SetHomingSensorTriggerValue(uint32_t userInput) {
  switch (userInput) {
  case 0:  sensorHomeValue = false; break;
  case 1:  sensorHomeValue = true; break;
  default: return WRITE_FAIL;
  }
  return WRITE_SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
uint32_t Stepper::RequestHoming(uint32_t userInput) {
  switch (userInput) {
  case 0: runHoming = false; break;
  case 1:
    homed     = false; // reset home flag
    runHoming = true;
    break;
  default: return WRITE_FAIL;
  }
  return WRITE_SUCCESS;
}

/* ---------------------------------------------------------------------------------- */
void Stepper::Run() {
  if (currentPOS != targetPOS) {
    digitalWrite(m_pinConfig.STEP_PIN, _step);
    _step = !_step;
    currentPOS += (direction ? 1 : -1);
  }
}

/* ---------------------------------------------------------------------------------- */
void Stepper::MoveInverseTime() {
  /*
  Args: Number of steps, time taken
  Output: RPM, timeAcel, timeDecel
  */
}

/* ---------------------------------------------------------------------------------- */
void Stepper::UpdateStatus(Status status) { m_status = status; }

/* ---------------------------------------------------------------------------------- */
bool Stepper::IsStalled(uint32_t sg_data, uint8_t status) {
  if (!stopOnStall) return false;

  // Lower than threshold
  if (currentRPM < threshLow) return false;

  // TODO: at higher rpm, sudden dip in value can be used
  if (currentRPM < threshHigh) return sg_data == 0;

  // Transition RPM
  if (currentRPM >= 700 && currentRPM <= 800) return false;

  // SG Flag from status at high RPM
  return bitRead(status, TMC2240_StatusFlag::STALLGUARD);
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::IsRunning() { return (currentPOS != targetPOS && !m_is_stalled); }

/* ---------------------------------------------------------------------------------- */
unsigned long Stepper::UpdateTickPeriod(TickType_t dt_ticks) {
  // In Position
  if (currentPOS == targetPOS) {
    m_in_position = true;
    return 0;
  } else
    m_in_position = false;

  // Homing
  // todo: separate
  if (runHoming) {
    switch (homingMethod) {
    case HomingMethod::IMMEDIATE:
      currentPOS = 0;
      targetPOS  = 0;
      homed      = true;
      runHoming  = false;
      break;

    case HomingMethod::TORQUE: {
      if (m_is_stalled) {
        currentPOS = 0;
        targetPOS  = 0;
        homed      = true;
        runHoming  = false;
      }
      break;
    }

    case HomingMethod::SENSOR:
      if (digitalRead(m_pinConfig.HOME_SENSOR_PIN) == sensorHomeValue) {
        currentPOS = 0;
        targetPOS  = 0;
        homed      = true;
        runHoming  = false;
      }
      break;
    }
  }

  // Stalled
  if (m_is_stalled) { return 0; }

  /* ==================================== direction =================================== */
  direction = target_pulse > current_pulse;
  digitalWrite(m_pinConfig.DIR_PIN, direction);

  long dt_us      = pdTICKS_TO_MS(dt_ticks);
  long pulse_rate = 0;

  if (use_s_curve) {
    pulse_rate = computePulseRate_scurve(current_pulse, target_pulse, current_speed, target_speed,
                                         acceleration, deceleration, jerk, dt_us);
  } else {
    pulse_rate = computePulseRate_trapezoidal(current_pulse, target_pulse, current_speed,
                                              target_speed, acceleration, deceleration, dt_us);
  }

  /* =================================== step delay =================================== */
  return 1000000 / pulse_rate;
}

/* ================================================================================== */
/*                                        Tasks                                       */
/* ================================================================================== */
void Stepper::task_ComputeRampParam(void *parameters) {
  auto *self = static_cast<Stepper *>(parameters);

  TickType_t    last_tick = xTaskGetTickCount();
  unsigned long tick_freq;
  for (;;) {
    // Sanity check
    if (self->m_timerMux == nullptr) continue;
    if (self->m_hwtimer == nullptr) continue;

    // Update time stamp
    TickType_t now      = xTaskGetTickCount();
    TickType_t dt_ticks = now - last_tick;
    last_tick           = now;

    // Compute
    tick_freq = self->UpdateTickPeriod(dt_ticks);

    // Update SW interrupt frequency
    portENTER_CRITICAL(self->m_timerMux);
    if (tick_freq > 0) {
      *(self->m_run) = true;
      timerAlarmWrite(self->m_hwtimer, tick_freq, true);
    } else {
      *(self->m_run) = false;
      timerAlarmWrite(self->m_hwtimer, 5000000, true);
    }
    portEXIT_CRITICAL(self->m_timerMux);

    vTaskDelay(1);
  }
}
/* ---------------------------------------------------------------------------------- */
void Stepper::task_UpdateStatus(void *parameters) {
  auto    *self = static_cast<Stepper *>(parameters);
  uint8_t  status;
  uint32_t sg_data;

  for (;;) {
    self->ReadRegister(TMC2240_Registers::SG_RESULT_IND, &sg_data, &status);

    // Power Error
    if (status == 0 || status == 255) {
      self->EmergencyStop();
      self->UpdateStatus(Status::POWER_ERR);
      continue;
    }

    // Stall Detection
    if (self->IsStalled(sg_data, status)) {
      self->EmergencyStop();
      self->UpdateStatus(Status::STALLED);
      self->m_is_stalled = true;
      continue;
    }

    // Running State
    if (self->IsRunning()) {
      self->UpdateStatus(Status::RUNNING);
      continue;
    }

    self->UpdateStatus(Status::IDLE);

    vTaskDelay(1);
  }
}

/* ================================================================================== */
/*                                      SPI Comm                                      */
/* ================================================================================== */
void Stepper::WriteRegister(const uint8_t address, const uint32_t data) {
  uint8_t buff[5];
  buff[0] = static_cast<uint8_t>(address | 0x80);
  buff[1] = static_cast<uint8_t>((data >> 24) & 0xFF);
  buff[2] = static_cast<uint8_t>((data >> 16) & 0xFF);
  buff[3] = static_cast<uint8_t>((data >> 8) & 0xFF);
  buff[4] = static_cast<uint8_t>(data & 0xFF);

  m_spi->SPIExchange(buff, 5, m_pinConfig.CS_PIN);
}

/* ---------------------------------------------------------------------------------- */
void Stepper::ReadRegister(const uint8_t address, uint32_t *data, uint8_t *status) {
  uint8_t buff[5];

  for (int i = 0; i < 2; i++) {
    buff[0] = address;
    buff[1] = 0x00;
    buff[2] = 0x00;
    buff[3] = 0x00;
    buff[4] = 0x00;
    m_spi->SPIExchange(buff, 5, m_pinConfig.CS_PIN);
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
  *data   = (buff[1] << 24) | (buff[2] << 16) | (buff[3] << 8) | buff[4];
}
/* ---------------------------------------------------------------------------------- */