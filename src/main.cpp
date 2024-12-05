#include "Comm.h"
#include "Define.h"
#include "Stepper.h"
#include "TMC2240_SPI.h"
#include <Arduino.h>
#include <ESP32TimerInterrupt.h>
#include <SPI.h>

#define SCK 18
#define MISO 19
#define MOSI 23
#define SS0 5
#define SS1 17
#define ISR_TIME_DEFAULT 500000 // 0.5 seconds

/* ====================================== Test ====================================== */
#define LED_PIN 2 // In some ESP32 board have inbuilt LED
int LED_STATE = LOW;
/* ---------------------------------------------------------------------------------- */

Comm comm;
TMC2240_SPI tmc2240spi; // spi object

/* ==================================== Stepper0 ==================================== */
Stepper stepper0(0);
hw_timer_t *timer0 = NULL;                             // Hardware timer0 pointer
portMUX_TYPE timerMux0 = portMUX_INITIALIZER_UNLOCKED; // spinlock
volatile bool run0;

/* ==================================== Stepper1 ==================================== */
Stepper stepper1(1);
hw_timer_t *timer1 = NULL;                             // Hardware timer1 pointer
portMUX_TYPE timerMux1 = portMUX_INITIALIZER_UNLOCKED; // spinlock
volatile bool run1;

/* ===================================== Thread ===================================== */

void IRAM_ATTR onTimer0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  if (run0) {
    stepper0.Run();
  }
  portEXIT_CRITICAL_ISR(&timerMux0);

  /* ============================== Error State Flashing ============================== */
  // LED_STATE = !LED_STATE;
  // digitalWrite(LED_PIN, LED_STATE);
}

void IRAM_ATTR onTimer1() {
  portENTER_CRITICAL_ISR(&timerMux1);
  if (run1) {
    stepper1.Run();
  }
  portEXIT_CRITICAL_ISR(&timerMux1);
}

void setup() {
  Serial.begin(115200);

  /* ============================== SPI Comm with driver ============================== */
  SPI.begin(SCK, MISO, MOSI, SS0);

  tmc2240spi.RegisterCSPin(0, SS0);
  tmc2240spi.RegisterCSPin(1, SS1);

  /* =================================== Motor Pins =================================== */
  // Stepper 0
  PinConfig pinConfig0;
  pinConfig0.EN_PIN = 22;
  pinConfig0.DIR_PIN = 21;
  pinConfig0.STEP_PIN = 32;
  pinConfig0.CS_PIN = SS0;
  pinConfig0.HOME_SENSOR_PIN = 33;
  stepper0.ConfigurePin(pinConfig0);

  // Stepper 1
  PinConfig pinConfig1;
  pinConfig1.EN_PIN = 25;
  pinConfig1.DIR_PIN = 26;
  pinConfig1.STEP_PIN = 27;
  pinConfig1.CS_PIN = SS1;
  pinConfig1.HOME_SENSOR_PIN = 14;
  stepper0.ConfigurePin(pinConfig1);

  /* ================================== Init stepper ================================== */
  stepper0.InitSPI(&tmc2240spi);
  stepper1.InitSPI(&tmc2240spi);
  stepper0.Initialize();
  stepper1.Initialize();

  /* ============================= Pass stepper0 into comm ============================= */
  comm.initStepper(0, &stepper0);
  comm.initStepper(1, &stepper1);

  /* ==================================== Init LED ==================================== */
  pinMode(LED_PIN, OUTPUT);

  /* =============================== Run stepper0 thread =============================== */
  // ! WRONG METHOD
  // xTaskCreatePinnedToCore(runStepper,   // Function to implement the task
  //                         "runStepper", // Name of the task
  //                         1000,         // Stack size in bytes
  //                         NULL,         // Task input parameter
  //                         0,            // Priority of the task
  //                         NULL,         // Task handle.
  //                         0             // Core where the task should Run
  // );

  /* ================================= Timer Interrupt ================================ */
  timer0 = timerBegin(0, 80, true);                // timer0 0, prescalar: 80, UP counting. 1 tick = 1us
  timerAttachInterrupt(timer0, &onTimer0, true);   // Attach interrupt
  timerAlarmWrite(timer0, ISR_TIME_DEFAULT, true); // With 80 prescalar, 1 tick = 1us
  timerAlarmEnable(timer0);                        // Enable Timer0 with interrupt (Alarm Enable)

  timer1 = timerBegin(1, 80, true);                // timer0 0, prescalar: 80, UP counting. 1 tick = 1us
  timerAttachInterrupt(timer1, &onTimer1, true);   // Attach interrupt
  timerAlarmWrite(timer1, ISR_TIME_DEFAULT, true); // With 80 prescalar, 1 tick = 1us
  timerAlarmEnable(timer1);                        // Enable Timer0 with interrupt (Alarm Enable)

  comm.init(&Serial);
}

void loop() {

  /* =================================== Read serial ================================== */
  comm.readSerial();

  /* ==================================== Stepper 0 =================================== */
  unsigned long stepDelay0 = stepper0.ComputeTimePeriod();
  portENTER_CRITICAL(&timerMux0);
  if (stepDelay0 > 0) {
    run0 = true;
    timerAlarmWrite(timer0, stepDelay0, true);
  } else {
    run0 = false;
    timerAlarmWrite(timer0, ISR_TIME_DEFAULT, true);
  }
  portEXIT_CRITICAL(&timerMux0);

  /* ==================================== Stepper 1 =================================== */
  unsigned long stepDelay1 = stepper1.ComputeTimePeriod();
  portENTER_CRITICAL(&timerMux1);
  if (stepDelay1 > 0) {
    run1 = true;
    timerAlarmWrite(timer1, stepDelay1, true);
  } else {
    run1 = false;
    timerAlarmWrite(timer1, ISR_TIME_DEFAULT, true);
  }
  portEXIT_CRITICAL(&timerMux1);
}
