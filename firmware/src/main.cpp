#include "Comm.h"
#include "Stepper.h"
#include "TMC2240_SPI.h"
#include "config.h"
#include "esp_log.h"
#include "pins.h"
#include <Arduino.h>
#include <SPI.h>

/* ========================== Constants and Globals =========================== */
constexpr int MAX_STEPPERS = 4;

Comm        comm;
TMC2240_SPI tmc2240spi;

Stepper     *steppers[MAX_STEPPERS]   = {nullptr};
hw_timer_t  *hw_timers[MAX_STEPPERS]  = {nullptr};
portMUX_TYPE timerMuxes[MAX_STEPPERS] = {portMUX_INITIALIZER_UNLOCKED, portMUX_INITIALIZER_UNLOCKED,
                                         portMUX_INITIALIZER_UNLOCKED,
                                         portMUX_INITIALIZER_UNLOCKED};
unsigned long stepDelays[MAX_STEPPERS] = {0};

/* ============================= ISR Functions ================================ */
void IRAM_ATTR onTimer0() {
  if (steppers[0] != nullptr) steppers[0]->Step();
}
void IRAM_ATTR onTimer1() {
  if (steppers[1] != nullptr) steppers[1]->Step();
}
void IRAM_ATTR onTimer2() {
  if (steppers[2] != nullptr) steppers[2]->Step();
}
void IRAM_ATTR onTimer3() {
  if (steppers[3] != nullptr) steppers[3]->Step();
}

void (*timerISRs[MAX_STEPPERS])() = {onTimer0, onTimer1, onTimer2, onTimer3};

/* =============================== Setup Helper =============================== */
void setupStepper(int index, PinConfig pinConfig, uint8_t csPin, uint8_t timerNum) {

  hw_timers[index] = timerBegin(timerNum, 80, true); // 1us per tick
  configASSERT(hw_timers[index]);

  steppers[index] =
      new Stepper(index, pinConfig, &tmc2240spi, hw_timers[index], &timerMuxes[index]);

  timerAttachInterrupt(hw_timers[index], timerISRs[index], true);
  timerAlarmWrite(hw_timers[index], ISR_TIME_DEFAULT, true);
  timerAlarmEnable(hw_timers[index]);
}

/* ================================ Setup ===================================== */
void setup() {
  SPI.begin(SCK, MISO, MOSI, S0_CS);

  setupStepper(0, {S0_EN, S0_DIR, S0_STEP, S0_CS, S0_HOME_SENSOR}, S0_CS, 0);
  setupStepper(1, {S1_EN, S1_DIR, S1_STEP, S1_CS, S1_HOME_SENSOR}, S1_CS, 1);
  setupStepper(2, {S2_EN, S2_DIR, S2_STEP, S2_CS, S2_HOME_SENSOR}, S2_CS, 2);
  setupStepper(3, {S3_EN, S3_DIR, S3_STEP, S3_CS, S3_HOME_SENSOR}, S3_CS, 3);

  Serial.begin(115200);
  // comm.init(&Serial);

  while (!Serial) {
    delay(100);
  }
  Serial.println(steppers[0]->Initialize());
}

/* ================================== Loop =================================== */
// void loop() { comm.readSerial(); }
void loop() {
  uint32_t curPos = steppers[0]->HandleRead(S_Reg::CURRENT_POS);
  uint32_t tarPos = steppers[0]->HandleRead(S_Reg::TARGET_POSITION);
  // Serial.print(curPos);
  // Serial.print("/");
  // Serial.println(tarPos);

  if (curPos == tarPos) {
    delay(5000);
    Serial.println("Set position");
    steppers[0]->SetTargetPosition(3600);
    steppers[0]->SetTargetRPM(720);
    // Serial.println(steppers[0]->HandleRead(S_Reg::TARGET_RPM));
    steppers[0]->SetAccelerationTime(1000);
    steppers[0]->SetDeccelerationTime(1000);

    Status sts = static_cast<Status>(steppers[0]->HandleRead(S_Reg::MOTOR_STATUS));
    Serial.println(getStatus(sts));

    // Serial.println("Move");
    steppers[0]->Move();
  }
}
