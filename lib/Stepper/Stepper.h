#ifndef STEPPER_H
#define STEPPER_H

#include "Kinematics.h"
#include "Registers.h"
#include "TMC2240_SPI.h"
#include "TMC2240_Registers.h"
#include "Utils.h"
#include <Arduino.h>

#define WRITE_SUCCESS    0x00000001
#define WRITE_FAIL       0x00000000
#define INVALID_REGISTER 0xFFFFFFFF

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

enum class OpMode {
  POSITION,
  VELOCITY,
  INVERSE_TIME,
};

enum class PositioningMode {
  RELATIVE,
  ABSOLUTE,
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
  Stepper(uint8_t id);
  // set
  void ConfigurePin(PinConfig pin);
  void InitSPI(TMC2240_SPI *tmc2240spi);
  void Initialize(bool *result = nullptr);

  // read
  uint32_t HandleRead(uint8_t reg);
  float    ReadTemperature();
  uint16_t ReadStallValue();
  uint8_t  ReadStatus();

  // write
  uint32_t HandleWrite(uint8_t reg, uint32_t data);
  uint32_t SetTargetPosition(int32_t pos);
  uint32_t SetCurrentPosition(int32_t pos);
  uint32_t SetTargetRPM(uint32_t rpm);
  uint32_t Move();
  uint32_t EmergencyStop();
  uint32_t StopVelocity();
  uint32_t EnableStepper();
  uint32_t DisableStepper();
  uint32_t SetOperationMode(uint32_t mode);
  uint32_t SetPositioningMode(uint32_t mode);
  uint32_t SetAccelerationTime(uint32_t millis);
  uint32_t SetDeccelerationTime(uint32_t millis);
  uint32_t SetStopOnStall(uint32_t userInput);
  uint32_t SetMicrostepping(uint32_t userInput);
  uint32_t SetRunningCurrent(uint32_t userInput);
  uint32_t SetHoldingCurrentPercentage(uint32_t userInput);
  uint32_t SetHomingMethod(uint32_t userInput);
  uint32_t SetHomingSensorTriggerValue(uint32_t userInput);
  uint32_t RequestHoming(uint32_t userInput);

  // action
  void          Run();
  void          MoveInverseTime(); // TODO
  unsigned long ComputeTimePeriod();

private:
  uint8_t      pri_id;
  PinConfig    m_pinConfig;
  TMC2240_SPI *m_spi;

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
  const float threshLow  = 30.0f;
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
  float         minRPM      = 10.0f;
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

  // Driver Comm
  const uint8_t Toff = {0x01};
  void          _RegWrite(const uint8_t address, const uint32_t data);
  void          _RegRead(const uint8_t address, uint32_t *data, uint8_t *status);
};

#endif // STEPPER_H
