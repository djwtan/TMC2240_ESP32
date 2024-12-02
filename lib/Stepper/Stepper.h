#ifndef STEPPER_H
#define STEPPER_H

#include "Define.h"
#include "Motion.h"
#include <Arduino.h>
#include <SPI.h>

#define REG_TARGET_POSITION 0x00
#define REG_TARGET_RPM 0x01
#define REG_MOVE 0x02
#define REG_TEMPERATURE 0x03
#define REG_DRV_STATUS 0x04
#define REG_MOTOR_STATUS 0x05
#define REG_EMERGENCY_STOP 0x06
#define REG_STOP_VELOCITY 0x07
#define REG_ENABLE_STEPPER 0x08
#define REG_OPERATION_MODE 0x09
#define REG_ACEL_TIME 0x0A
#define REG_DECEL_TIME 0x0B
#define REG_CURRENT_RPM 0X0C
#define REG_CURRENT_POS 0x0D
#define REG_ACTUAL_ACCELERATION_TIME 0x0E
#define REG_ACTUAL_DECCELERATION_TIME 0x0F

struct PinConfig {
  uint8_t EN_PIN;
  uint8_t DIR_PIN;
  uint8_t STEP_PIN;
  uint8_t CS_PIN;
};

struct MotorConfig {
  uint8_t MICROSTEP = 128; // default=1
  uint8_t RUNNING_CURRENT = 31;
  uint8_t HOLDING_CURRENT = 16;
};

class Stepper {
public:
  Stepper(uint8_t id);
  // set
  void ConfigurePin(PinConfig pin);
  void ConfigureMotor(MotorConfig mconfig);
  void Initialize(bool *result = nullptr);
  void SetPositionMode(Mode_PositionCommand posMode); // TODO

  // read
  String HandleRead(uint8_t reg);
  float ReadTemperature();
  uint8_t ReadStatus();
  String ReadMotorState();

  // write
  String HandleWrite(uint8_t reg, uint32_t data);
  String WriteTargetPosition(int32_t pos);
  String WriteTargetRPM(uint32_t rpm);
  String Move();
  String EmergencyStop();
  String StopVelocity(); // TODO: Redo for velocity mode only
  String EnableStepper();
  String SetOperationMode(uint32_t mode);
  String SetAccelerationTime(uint32_t millis);
  String SetDeccelerationTime(uint32_t millis);
  String WriteCurrentPosition(int32_t pos);

  // action
  void ReleaseAxis();
  void Run();
  void MoveInverseTime(); // TODO
  unsigned long ComputeTimePeriod();

private:
  uint8_t pri_id;
  PinConfig m_pinConfig;
  MotorConfig m_motorConfig;
  SPISettings spiSettings;

  // Operation
  bool swEN{false};
  OperationMode opMode{OperationMode::POSITION};
  Mode_PositionCommand posMode{Mode_PositionCommand::RELATIVE};

  // Return
  String _GenerateMessage();

  // Default
  const float maxRPM{240.0f};
  const float minRPMSlow{10.0f}; // !guess
  const float minRPMFast{10.0f}; // !guess
  const float RPMThresh{150.0f};
  const int32_t DUMMY_POSITIVE{50000000};
  const int32_t DUMMY_NEGATIVE{-50000000};

  // Set
  int32_t targetPOS{0};
  float targetRPM{0};
  double timeAcel_ms{2 * 1000000UL};
  double timeDecel_ms{2 * 1000000UL};
  bool stopOnStall{false};

  // Test
  unsigned long actualAcelTime{0};
  unsigned long actualDecelTime{0};

  // ReadBack & MotorState
  MotorState motorState{MotorState::NOT_INIT};
  void _UpdateMotorState(MotorState mState);

  // StallGuard
  // todo: expose these values
  const float threshLow = 30.0f;
  const float threshHigh = 150.0f;
  bool _IsStalled();

  // Movement
  void _ComputeAccelerationParameters(float v0);
  void _ComputeDeccelerationParameters(float vmax);
  void _ComputeShortDeccelerationParameters(float vmax, uint32_t remainingSteps);

  // Driven
  volatile int32_t currentPOS{0};
  bool _step{true};

  float currentRPM{0.0f};
  float RPMPeak{0.0f};
  bool direction{true};
  bool acelerating{false};
  float minRPM{10.0f};
  unsigned long stepDelay{0UL};
  unsigned long timeStamp{micros()};
  uint32_t sAbs{0};

  // calculation
  unsigned long dt{0};
  bool recomputeParam{false};
  bool motionInited{false};

  unsigned long t_0{0UL};
  unsigned long tDecel_0{0UL};
  int32_t s_0{0};
  float v_0{0.0f};
  uint32_t sTotal{0};
  double nAcel{0.0};
  uint32_t sAcel{0};
  double nDecel{0};
  uint32_t sDecel{0};
  float mDecel{0.0f};

  uint32_t sDecelRecomputed{0};

  // Driver Comm
  const uint8_t Toff = {0x01};
  void _RegWrite(const uint8_t address, const uint32_t data);
  void _RegRead(const uint8_t address, uint32_t *data, uint8_t *status);
  void _SPIExchange(uint8_t *data, const int size);
};

#endif // STEPPER_H