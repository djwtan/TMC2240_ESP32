#ifndef STEPPER_H
#define STEPPER_H

#include "Kinematics.h"
#include "Registers.h"
#include "TMC2240_Registers.h"
#include "TMC2240_SPI.h"
#include "Utils.h"
#include <Arduino.h>

#define WRITE_SUCCESS    0x00000001
#define WRITE_FAIL       0x00000000
#define INVALID_REGISTER 0xFFFFFFFF

enum class MotorState {
  STALLED   = 0,
  OVERSPEED = 1,
  IDLE      = 2,
  RUNNING   = 3,
  POWER_ERR = 4,
  NOT_INIT  = 5,
};

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
  Stepper(uint8_t id, TMC2240_SPI *tmc2240spi, volatile bool *run, hw_timer_t *hwtimer,
          portMUX_TYPE *timerMux);

  /* ====================================== Setup ===================================== */
  /* Configure pinouts */
  void ConfigurePin(PinConfig pin);

  /* Initialize driver */
  void Initialize(bool *result = nullptr);

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
  uint32_t SetTargetPosition(int32_t pos);

  /* Overrides current position (unit: microstep) */
  uint32_t SetCurrentPosition(int32_t pos);

  /* Sets target speed (unit: rpm) */
  uint32_t SetTargetRPM(uint32_t rpm);

  /* Starts movement with current settings */
  uint32_t Move();

  /* Stops motion immediately */
  uint32_t EmergencyStop();

  /* Ramp stop (velocity mode only) */
  uint32_t StopVelocity();

  /* Reinitializes stepper */
  uint32_t EnableStepper();

  /* Disables stepper (only in idle state) */
  uint32_t DisableStepper();

  /* Sets operation mode (pos / vel / ivt) */
  uint32_t SetOperationMode(uint32_t mode);

  /* Sets positioning mode (abs / rel) */
  uint32_t SetPositioningMode(uint32_t mode);

  /* Sets acceleration time (unit: milliseconds)*/
  uint32_t SetAccelerationTime(uint32_t millis);

  /* Sets decceleration time (units: milliseconds) */
  uint32_t SetDeccelerationTime(uint32_t millis);

  /* Sets stopOnStall flag */
  uint32_t SetStopOnStall(uint32_t userInput);

  /* Sets microstepping value */
  uint32_t SetMicrostepping(uint32_t userInput);

  /* Sets running current (1-31) */
  uint32_t SetRunningCurrent(uint32_t userInput);

  /* Sets holding current percentage (scales with running current) */
  uint32_t SetHoldingCurrentPercentage(uint32_t userInput);

  /* Sets homing method (immediate / torque / sensor) */
  uint32_t SetHomingMethod(uint32_t userInput);

  /* Sets homing trigger value to HL / LH (sensor only) */
  uint32_t SetHomingSensorTriggerValue(uint32_t userInput);

  /* Initializes homing movement */
  uint32_t RequestHoming(uint32_t userInput);

  /* =========================== Ramp Generation & Stepping =========================== */
  /* Steps pin & handles current position */
  void Run();

  /* Inverse time move (?) */
  void MoveInverseTime(); // TODO

  /* Computes interrupt pulse width */
  unsigned long ComputeTimePeriod();

private:
  uint8_t        m_id;
  PinConfig      m_pinConfig;
  TMC2240_SPI   *m_spi;
  volatile bool *m_run;                // run flag
  hw_timer_t    *m_hwtimer  = nullptr; // timer instance
  portMUX_TYPE  *m_timerMux = nullptr; // mutex

  bool            enabled         = false;
  OpMode          opMode          = OpMode::POSITION;
  PositioningMode posMode         = PositioningMode::ABSOLUTE;
  HomingMethod    homingMethod    = HomingMethod::IMMEDIATE;
  bool            sensorHomeValue = false;

  // Return
  String _GenerateMessage();

  // Default
  const float   MAX_RPM        = 2400.0f;
  const int32_t DUMMY_POSITIVE = 500000;
  const int32_t DUMMY_NEGATIVE = -500000;

  // Set
  uint8_t microstep                = 4;
  uint8_t runningCurrent           = 31;
  uint8_t holdingCurrentPercentage = 50;
  uint8_t holdingCurrent           = runningCurrent * holdingCurrentPercentage / 100;

  int32_t targetPOS      = 0;
  int32_t targetPOSHold  = 0;
  float   targetRPM      = 0;
  float   targetRPM_Hold = 0;
  double  timeAcel_ms    = 2 * 1000000UL;
  double  timeDecel_ms   = 2 * 1000000UL;
  bool    stopOnStall    = false;
  bool    runHoming      = false;
  bool    homed          = false;

  // Test
  unsigned long actualAcelTime  = 0;
  unsigned long actualDecelTime = 0;

  // ReadBack & MotorState
  MotorState motorState = MotorState::NOT_INIT;
  void       _UpdateMotorState(MotorState mState);

  // StallGuard
  // todo: expose these values
  const float threshLow  = 40.0f;
  const float threshHigh = 150.0f;
  bool        _IsStalled();

  // Movement
  void _ComputeAccelerationParameters();
  void _ComputeDeccelerationParameters(float vmax);

  // Driven
  volatile int32_t currentPOS = 0;
  bool             _step      = true;

  float         currentRPM  = 0.0f;
  float         peakRPM     = 0.0f;
  bool          direction   = true;
  bool          acelerating = false;
  float         minRPM      = 0.0f;
  unsigned long stepDelay   = 0UL;
  unsigned long timeStamp   = micros();
  uint32_t      sAbs        = 0;

  // calculation
  bool          recomputeParam   = false;
  unsigned long t_0              = 0UL;
  unsigned long tDecel_0         = 0UL;
  int32_t       s_0              = 0;
  float         v_0              = 0.0f;
  uint32_t      sTotal           = 0;
  double        nAcel            = 0.0;
  uint32_t      sAcel            = 0;
  float         mDecel           = 0.0f;
  uint32_t      sDecel           = 0;
  uint32_t      sDecelRecomputed = 0;

  /* ================================================================================== */
  /*                                        Tasks                                       */
  /* ================================================================================== */
  bool m_tasksStarted = false;
  /* Ramp generation */
  static void task_ComputeRampParam(void *parameters);

  /* ================================================================================== */
  /*                                    Driver Comms                                    */
  /* ================================================================================== */
  const uint8_t Toff = {0x01};
  void          _RegWrite(const uint8_t address, const uint32_t data);
  void          _RegRead(const uint8_t address, uint32_t *data, uint8_t *status);
};

#endif // STEPPER_H
