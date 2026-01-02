#ifndef STEPPER_H
#define STEPPER_H

#include "Ramp.h"
#include "Registers.h"
#include "TMC2240_Registers.h"
#include "TMC2240_SPI.h"
#include "Utils.h"
#include "functional"
#include <Arduino.h>

#define WRITE_SUCCESS    0x00000001
#define WRITE_FAIL       0x00000000
#define INVALID_REGISTER 0xFFFFFFFF

enum class Status {
  STALLED   = 0,
  OVERSPEED = 1,
  IDLE      = 2,
  RUNNING   = 3,
  POWER_ERR = 4,
  NOT_INIT  = 5,
};

static inline String getStatus(Status sts) {
  switch (sts) {
  case Status::STALLED:   return "STALLED";
  case Status::OVERSPEED: return "OVERSPEED";
  case Status::IDLE:      return "IDLE";
  case Status::RUNNING:   return "RUNNING";
  case Status::POWER_ERR: return "POWER_ERR";
  case Status::NOT_INIT:  return "NOT_INIT";
  default:                return "N/A";
  }
}

enum class HomingMethod {
  IMMEDIATE = 0,
  SENSOR    = 1,
  TORQUE    = 2,
};

enum class OpMode {
  POSITION     = 0,
  VELOCITY     = 1,
  INVERSE_TIME = 2,
};

enum class PositioningMode {
  RELATIVE = 0,
  ABSOLUTE = 1,
};

struct PinConfig {
  uint8_t EN_PIN;
  uint8_t DIR_PIN;
  uint8_t STEP_PIN;
  uint8_t CS_PIN;
  uint8_t HOME_SENSOR_PIN;
};

class Stepper {
public:
  Stepper(uint8_t id, PinConfig pinConfig, TMC2240_SPI *tmc2240spi, hw_timer_t *hwtimer,
          portMUX_TYPE *timerMux);

  /* ====================================== Setup ===================================== */
  /* Configure pinouts */
  void ConfigurePin(PinConfig pin);

  /* Initialize driver */
  bool Initialize();

  /* ====================================== Read ====================================== */
  /* Handles read queries from comm class */
  uint32_t HandleRead(uint8_t reg);

  /* Returns driver temperature */
  float ReadTemperature();

  /* Returns StallGuard value */
  uint16_t ReadStallValue();

  /* Returns driver status */
  uint8_t ReadStatus();

  /* ====================================== Write ===================================== */
  /* Handles write requests from comm class */
  uint32_t HandleWrite(uint8_t reg, uint32_t data);

  /* Sets target position (unit: microstep) */
  bool SetTargetPosition(int32_t pos);

  /* Overrides current position (unit: microstep) */
  bool SetCurrentPosition(int32_t pos);

  /* Sets target speed (unit: rpm) */
  bool SetTargetRPM(uint32_t rpm);

  /* Starts movement with current settings */
  bool Move();

  /* Stops motion immediately */
  bool EmergencyStop();

  /* Ramp stop (velocity mode only) */
  bool StopVelocity();

  /* Reinitializes stepper */
  bool EnableStepper();

  /* Disables stepper (only in idle state) */
  bool DisableStepper();

  /* Sets operation mode (pos / vel / ivt) */
  bool SetOperationMode(uint32_t mode);

  /* Sets positioning mode (abs / rel) */
  bool SetPositioningMode(uint32_t mode);

  /* Sets acceleration time (unit: milliseconds)*/
  bool SetAccelerationTime(uint32_t millis);

  /* Sets decceleration time (units: milliseconds) */
  bool SetDeccelerationTime(uint32_t millis);

  /* Sets stopOnStall flag */
  bool SetStopOnStall(uint32_t userInput);

  /* Sets microstepping value */
  bool SetMicrostepping(uint32_t userInput);

  /* Sets running current (1-31) */
  bool SetRunningCurrent(uint32_t userInput);

  /* Sets holding current percentage (scales with running current) */
  bool SetHoldingCurrentPercentage(uint32_t userInput);

  /* Sets homing method (immediate / torque / sensor) */
  bool SetHomingMethod(uint32_t userInput);

  /* Sets homing trigger value to HL / LH (sensor only) */
  bool SetHomingSensorTriggerValue(uint32_t userInput);

  /* Initializes homing movement */
  bool RequestHoming(uint32_t userInput);

  /* =========================== Ramp Generation & Stepping =========================== */
  /* Steps pin & handles current position */
  void Step();

  /* Inverse time move (?) */
  void MoveInverseTime(); // TODO

  /* ---------------------------------------------------------------------------------- */
  /* ---------------------------------------------------------------------------------- */
  /* ---------------------------------------------------------------------------------- */

private:
  uint8_t       m_id;
  PinConfig     m_pinConfig;
  TMC2240_SPI  *m_spi;
  hw_timer_t   *m_hwtimer  = nullptr; // timer instance
  portMUX_TYPE *m_timerMux = nullptr; // mutex

  // Default

  /* ================================================================================== */
  /*                                    Configurables                                   */
  /* ================================================================================== */

  // Motion Settings
  struct MotionSettings {
    OpMode          opMode       = OpMode::POSITION;
    PositioningMode posMode      = PositioningMode::RELATIVE;
    bool            useSCurve    = true;
    int32_t         targetPulse  = 0;
    float           targetSpeed  = 0.0f;
    float           acceleration = 0.0f;
    float           deceleration = 0.0f;
    float           jerkAcel     = 0.0f;
    float           jerkDecel    = 0.0f;
  };

  // Driver Settings
  struct DriverSettings {
    uint8_t  microstep      = 4; // ---------- motion
    uint8_t  fullstepPerRev = 200;
    uint32_t unitsPerRev    = 360;
    uint8_t  runningCurrent = 31; // --------- current
    uint8_t  holdingCurrent = 16;
    bool     sg_enable      = false; // ------ stall detection
    float    sg_threshLow   = 40.0f;
    float    sg_threshHigh  = 150.0f;
  };

  // Homing Settings
  struct HomingSettings {
    HomingMethod homingMethod = HomingMethod::IMMEDIATE;
    bool         lh           = false; // low to high
  };

  DriverSettings drvS;
  MotionSettings motionS;
  HomingSettings homingS;

  /* ================================================================================== */
  /*                                  Flags & Statuses                                  */
  /* ================================================================================== */

  // ReadBack & Status
  void   UpdateStatus(Status status);
  Status m_status = Status::NOT_INIT;

  // Flags
  bool m_step        = true;  // (f) Pin state for next step
  bool m_direction   = true;  // (f) Direction
  bool m_isStalled   = false; // (f) Stall
  bool m_isRunning   = false; // (f) Running
  bool m_inPosition  = true;  // (f) In Position
  bool m_move        = false; // (f) Move command
  bool m_drv_enabled = false; // (f) enable pin tracker

  // Ramp
  volatile int32_t m_currentPulse        = 0;
  float            m_currentSpeed        = 0.0f;
  float            m_sCurve_currentAccel = 0.0f;

  /* ================================================================================== */
  /*                                        Tasks                                       */
  /* ================================================================================== */
  bool m_tasksStarted = false;

  /* Ramp generation */
  unsigned long UpdateTickPeriod(TickType_t dt_ticks);
  static void   task_ComputeRampParam(void *parameters);

  /* Status */
  bool        IsStalled(uint32_t sg_data, uint8_t status);
  static void task_UpdateStatus(void *parameters);

  /* ================================================================================== */
  /*                                    Driver Comms                                    */
  /* ================================================================================== */
  const uint8_t Toff = {0x01};
  void          WriteRegister(const uint8_t address, const uint32_t data);
  void          ReadRegister(const uint8_t address, uint32_t *data, uint8_t *status);

  /* ================================================================================== */
  /*                                        Math                                        */
  /* ================================================================================== */
  int32_t  unitToPulse(int32_t unit);
  int32_t  pulseToUnit(int32_t pulse);
  float    rpmToSpeed(uint32_t rpm);
  uint32_t speedToRpm(float speed);

  /* ================================================================================== */
  /*                                   Driver Control                                   */
  /* ================================================================================== */
  void enableDriver(bool enable) {
    if (enable != m_drv_enabled) {
      digitalWrite(m_pinConfig.EN_PIN, !enable);
      m_drv_enabled = enable;
    }
  }
};

#endif // STEPPER_H
