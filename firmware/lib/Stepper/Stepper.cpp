#include "Stepper.h"

Stepper::Stepper(uint8_t id, PinConfig pinConfig, TMC2240_SPI *tmc2240spi, hw_timer_t *hwtimer,
                 portMUX_TYPE *timerMux)
    : m_id(id), m_spi(tmc2240spi), m_pinConfig(pinConfig), m_hwtimer(hwtimer),
      m_timerMux(timerMux) {

  // Initialize Pins
  pinMode(m_pinConfig.EN_PIN, OUTPUT);
  pinMode(m_pinConfig.STEP_PIN, OUTPUT);
  pinMode(m_pinConfig.DIR_PIN, OUTPUT);
  pinMode(m_pinConfig.CS_PIN, OUTPUT);
  pinMode(m_pinConfig.HOME_SENSOR_PIN, INPUT);

  // Default Modes
  digitalWrite(m_pinConfig.EN_PIN, HIGH); // disabled
  m_drv_enabled = false;
  digitalWrite(m_pinConfig.CS_PIN, HIGH);
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::Initialize() {
  m_move = false;

  this->WriteRegister(0x6C, 0x00000000); // disable driver

  uint32_t ms = 0;
  switch (drvS.microstep) {
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
  uint32_t chopConfWord = 0;
  chopConfWord |= 0x30410150;
  chopConfWord |= (ms << 24);
  chopConfWord |= Toff;
  this->WriteRegister(TMC2240_Registers::CHOPCONF, chopConfWord);

  // IHOLD_IRUN: run current, hold current, hold delay.
  uint32_t currentWord = 0;
  currentWord |= 0x00060000;
  currentWord |= ((uint32_t)(drvS.runningCurrent & 0x1F) << 8);
  currentWord |= ((uint32_t)(drvS.holdingCurrent & 0x1F));
  this->WriteRegister(TMC2240_Registers::IHOLD_IRUN, currentWord);

  // TPOWERDOWN: delay before switching to hold current (in ~1.6s units)
  this->WriteRegister(TMC2240_Registers::TPOWERDOWN, 10); // e.g. 10 = ~16s

  // TPWMTHRS: threshold for switching from stealthChop to spreadCycle
  this->WriteRegister(TMC2240_Registers::TPWMTHRS, 0xFFFFF); // Use stealthChop for all speeds

  // GCONF: enable StealthChop and diagnostic output config if needed
  this->WriteRegister(TMC2240_Registers::GCONF, 0x00000004); // en_pwm_mode

  // PWMCONF: configure StealthChop
  this->WriteRegister(TMC2240_Registers::PWMCONF, 0x00050480); // conservative default

  uint32_t data;
  uint8_t  status;
  this->ReadRegister(0x6C, &data, &status);

  if ((data & 0x0000000F) != Toff) {
    UpdateStatus(Status::NOT_INIT);
    return false;
  }

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

  UpdateStatus(Status::IDLE);
  return true;
}

/* ================================================================================== */
/*                                        READ                                        */
/* ================================================================================== */
uint32_t Stepper::HandleRead(uint8_t reg) {
  uint32_t res = 0;

  switch (reg) {
  case S_Reg::TARGET_POSITION:             res = static_cast<uint32_t>(pulseToUnit(motionS.targetPulse)); break;
  case S_Reg::TARGET_RPM:                  res = static_cast<uint32_t>(speedToRpm(motionS.targetSpeed)); break;
  case S_Reg::TEMPERATURE:                 res = floatTo32Bit(this->ReadTemperature()); break;
  case S_Reg::DRV_STATUS:                  res = static_cast<uint32_t>(this->ReadStatus()); break;
  case S_Reg::MOTOR_STATUS:                res = static_cast<uint32_t>(m_status); break;
  case S_Reg::OPERATION_MODE:              res = static_cast<uint32_t>(motionS.opMode); break;
  case S_Reg::ACEL_TIME:                   res = static_cast<uint32_t>(speedToRpm(motionS.acceleration)); break;
  case S_Reg::DECEL_TIME:                  res = static_cast<uint32_t>(speedToRpm(motionS.deceleration)); break;
  case S_Reg::CURRENT_RPM:                 res = static_cast<uint32_t>(speedToRpm(m_currentSpeed)); break;
  case S_Reg::CURRENT_POS:                 res = static_cast<uint32_t>(pulseToUnit(m_currentPulse)); break;
  case S_Reg::ACTUAL_ACCELERATION_TIME:    res = 0; break;
  case S_Reg::ACTUAL_DECCELERATION_TIME:   res = 0; break;
  case S_Reg::STOP_ON_STALL:               res = static_cast<uint32_t>(drvS.sg_enable); break;
  case S_Reg::MICROSTEPPING:               res = static_cast<uint32_t>(drvS.microstep); break;
  case S_Reg::RUNNING_CURRENT:             res = static_cast<uint32_t>(drvS.runningCurrent); break;
  case S_Reg::HOLDING_CURRENT:             res = static_cast<uint32_t>(drvS.holdingCurrent); break;
  case S_Reg::STALL_VALUE:                 res = static_cast<uint32_t>(this->ReadStallValue()); break;
  case S_Reg::HOMING_METHOD:               res = static_cast<uint32_t>(homingS.homingMethod); break;
  case S_Reg::HOMING_SENSOR_TRIGGER_VALUE: res = static_cast<uint32_t>(homingS.lh); break;
  case S_Reg::REQUEST_HOMING:              res = 0; break;
  case S_Reg::HOMED:                       res = 0; break;
  case S_Reg::POSITIONING_MODE:            res = static_cast<uint32_t>(motionS.posMode); break;
  default:                                 res = static_cast<uint32_t>(INVALID_REGISTER); break;
  }

  return res;
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
  case S_Reg::HOLDING_CURRENT:             result = this->SetHoldingCurrentPercentage(data); break;
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
bool Stepper::SetTargetPosition(int32_t pos) {
  switch (motionS.opMode) {
  case OpMode::POSITION:
    if (motionS.posMode == PositioningMode::RELATIVE) {
      motionS.targetPulse += unitToPulse(pos);
    } else {
      motionS.targetPulse = unitToPulse(pos);
    }
    break;

    // todo
    // case OpMode::VELOCITY:
    //   motionS.targetPulse += pos >= 0 ? DUMMY_POSITIVE : DUMMY_NEGATIVE; // dummy value
    //   break;
  }

  return true;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::SetCurrentPosition(int32_t pos) {
  if (m_status == Status::RUNNING) { return false; }
  m_currentPulse = unitToPulse(pos);
  return true;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::SetTargetRPM(uint32_t rpm) {
  motionS.targetSpeed = rpmToSpeed(rpm);
  return true;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::Move() {
  if (m_status != Status::IDLE) { return false; }

  enableDriver(true);

  // allow move
  m_move = true;
  return true;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::EmergencyStop() {
  m_move = false;
  enableDriver(false);
  return true;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::StopVelocity() {
  if (motionS.opMode != OpMode::VELOCITY) { return false; }
  // todo
  return false;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::EnableStepper() {
  // Can only enable in idle state
  if (m_status != Status::IDLE) return false;

  enableDriver(true);
  return true;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::DisableStepper() {
  // Can only disable in idle state
  if (m_status != Status::IDLE) return false;

  enableDriver(false);
  return true;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::SetOperationMode(uint32_t mode) {
  // Can only set in idle state
  if (m_status != Status::IDLE) return false;

  switch (mode) {
  case 0:  motionS.opMode = OpMode::POSITION; break;
  case 1:  motionS.opMode = OpMode::VELOCITY; break;
  case 2:  motionS.opMode = OpMode::INVERSE_TIME; break;
  default: return false;
  }
  return true;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::SetPositioningMode(uint32_t mode) {
  // Can only set in idle state
  if (m_status != Status::IDLE) return false;

  switch (mode) {
  case 0:  motionS.posMode = PositioningMode::ABSOLUTE; break;
  case 1:  motionS.posMode = PositioningMode::RELATIVE; break;
  default: return false;
  }
  return true;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::SetAccelerationTime(uint32_t millis) {
  // todo: Swap to units prolly
  float time_s = millis / 1000;

  motionS.acceleration = motionS.targetSpeed / time_s;

  return true;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::SetDeccelerationTime(uint32_t millis) {
  // todo: Swap to units prolly
  float time_s = millis / 1000;

  motionS.deceleration = motionS.targetSpeed / time_s;

  return true;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::SetStopOnStall(uint32_t userInput) {
  switch (userInput) {
  case 0:  drvS.sg_enable = false; break;
  case 1:  drvS.sg_enable = true; break;
  default: return false;
  }
  return true;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::SetMicrostepping(uint32_t userInput) {
  if (m_status != Status::IDLE) return false;

  switch (userInput) {
  case 1:   drvS.microstep = 1; break;
  case 2:   drvS.microstep = 2; break;
  case 4:   drvS.microstep = 4; break;
  case 8:   drvS.microstep = 8; break;
  case 16:  drvS.microstep = 16; break;
  case 32:  drvS.microstep = 32; break;
  case 64:  drvS.microstep = 64; break;
  case 128: drvS.microstep = 128; break;
  default:  return false;
  }

  // Reinitialize
  return Initialize();
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::SetRunningCurrent(uint32_t userInput) {
  // Require idle
  if (m_status != Status::IDLE) return false;

  if (userInput <= 0 || userInput > 31) return false;

  drvS.runningCurrent = userInput;

  // Reinitialize
  return Initialize();
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::SetHoldingCurrentPercentage(uint32_t userInput) {
  // Require idle
  if (m_status != Status::IDLE) return false;

  if (userInput <= 0 || userInput > 31) return false;

  drvS.holdingCurrent = userInput;

  // Reinitialize
  return Initialize();
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::SetHomingMethod(uint32_t userInput) {
  switch (userInput) {
  case 0:  homingS.homingMethod = HomingMethod::IMMEDIATE; break;
  case 1:  homingS.homingMethod = HomingMethod::TORQUE; break;
  case 2:  homingS.homingMethod = HomingMethod::SENSOR; break;
  default: return false;
  }
  return true;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::SetHomingSensorTriggerValue(uint32_t userInput) {
  switch (userInput) {
  case 0:  homingS.lh = false; break;
  case 1:  homingS.lh = true; break;
  default: return false;
  }
  return true;
}

/* ---------------------------------------------------------------------------------- */
bool Stepper::RequestHoming(uint32_t userInput) {
  // switch (userInput) {
  // case 0: homingS.runHoming = false; break;
  // case 1:
  //   homed     = false; // reset home flag
  //   runHoming = true;
  //   break;
  // default: return false;
  // }
  return true;
}

/* ---------------------------------------------------------------------------------- */
void Stepper::MoveInverseTime() {
  /*
  Args: Number of steps, time taken
  Output: RPM, timeAcel, timeDecel
  */
}

/* ================================================================================== */
/*                                       Status                                       */
/* ================================================================================== */
void Stepper::UpdateStatus(Status status) { m_status = status; }

/* ---------------------------------------------------------------------------------- */
bool Stepper::IsStalled(uint32_t sg_data, uint8_t status) {
  if (!drvS.sg_enable) return false;

  uint32_t currentRpm = speedToRpm(m_currentSpeed);

  // Lower than threshold
  if (currentRpm < drvS.sg_threshLow) return false;

  // TODO: at higher rpm, sudden dip in value can be used
  if (currentRpm > drvS.sg_threshHigh) return sg_data == 0;

  // Transition RPM
  if (currentRpm >= 700 && currentRpm <= 800) return false;

  // SG Flag from status at high RPM
  return bitRead(status, TMC2240_StatusFlag::STALLGUARD);
}

/* ---------------------------------------------------------------------------------- */
void Stepper::Step() {
  // Stalled
  if (m_isStalled) { return; }

  // In Position
  if (m_currentPulse == motionS.targetPulse) { return; }

  digitalWrite(m_pinConfig.STEP_PIN, m_step);
  m_step = !m_step;
  m_currentPulse += (m_direction ? 1 : -1);
}

/* ---------------------------------------------------------------------------------- */
unsigned long Stepper::UpdateTickPeriod(TickType_t dt_ticks) {
  // In Position
  if (m_currentPulse == motionS.targetPulse) {
    m_inPosition = true;
    return 0;
  }

  // Update in position flag
  m_inPosition = false;

  // Stalled
  if (m_isStalled) { return 0; }

  // Compute direction
  m_direction = motionS.targetPulse > m_currentPulse;
  digitalWrite(m_pinConfig.DIR_PIN, m_direction);

  long dt_us      = pdTICKS_TO_MS(dt_ticks);
  long pulse_rate = 0;

  // Compute pulse rate
  pulse_rate = motionS.useSCurve
                   ? computePulseRate_scurve(m_currentPulse, motionS.targetPulse, m_currentSpeed,
                                             motionS.targetSpeed, motionS.acceleration,
                                             motionS.deceleration, motionS.jerk, dt_us)
                   : computePulseRate_trapezoidal(
                         m_currentPulse, motionS.targetPulse, m_currentSpeed, motionS.targetSpeed,
                         motionS.acceleration, motionS.deceleration, dt_us);

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
    if (self->m_timerMux == nullptr) { continue; }
    if (self->m_hwtimer == nullptr) { continue; }

    // Update time stamp
    TickType_t now      = xTaskGetTickCount();
    TickType_t dt_ticks = now - last_tick;
    last_tick           = now;

    // Compute
    tick_freq         = self->UpdateTickPeriod(dt_ticks);
    self->m_isRunning = tick_freq != 0;

    // Update SW interrupt frequency
    portENTER_CRITICAL(self->m_timerMux);
    if (tick_freq > 0) {
      timerAlarmWrite(self->m_hwtimer, tick_freq, true);
    } else {
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
      self->m_isStalled = true;
      continue;
    }

    // Running State
    if (self->m_isRunning) {
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

/* ================================================================================== */
/*                                        Maths                                       */
/* ================================================================================== */
int32_t Stepper::unitToPulse(int32_t unit) {
  return (unit / drvS.unitsPerRev) * (drvS.microstep * drvS.fullstepPerRev);
}

int32_t Stepper::pulseToUnit(int32_t pulse) {
  return (pulse * drvS.unitsPerRev) / (drvS.microstep * drvS.fullstepPerRev);
}

float Stepper::rpmToSpeed(uint32_t rpm) {
  return (rpm / 60) * (drvS.microstep * drvS.fullstepPerRev);
}

uint32_t Stepper::speedToRpm(float speed) {
  return speed * 60 / (drvS.microstep * drvS.fullstepPerRev);
}