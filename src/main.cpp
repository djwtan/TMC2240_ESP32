#include "Comm.h"
#include "Define.h"
#include "Stepper.h"
#include <Arduino.h>
#include <ESP32TimerInterrupt.h>
#include <SPI.h>

#define SCK 18
#define MISO 19
#define MOSI 23
#define SS 5
#define ISR_TIME_DEFAULT 500000 // 0.5 seconds

/* ====================================== Test ====================================== */
#define LED_PIN 2 // In some ESP32 board have inbuilt LED
int LED_STATE = LOW;
/* ---------------------------------------------------------------------------------- */

Stepper stepper(0);
Comm comm;

hw_timer_t *timer = NULL;                             // Hardware timer pointer
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED; // spinlock

/* ===================================== Thread ===================================== */
volatile bool run;

void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (run) {
    stepper.Run();
    LED_STATE = !LED_STATE;
    digitalWrite(LED_PIN, LED_STATE);
  }
  portEXIT_CRITICAL_ISR(&timerMux);

  /* ============================== Error State Flashing ============================== */
  // LED_STATE = !LED_STATE;
  // digitalWrite(LED_PIN, LED_STATE);
}

void setup() {
  /* ============================== SPI Comm with driver ============================== */
  SPI.begin(SCK, MISO, MOSI, SS);

  /* =================================== Motor Pins =================================== */
  PinConfig pinConfig;
  pinConfig.EN_PIN = 32;
  pinConfig.DIR_PIN = 33;
  pinConfig.STEP_PIN = 25;
  pinConfig.CS_PIN = SS;
  pinConfig.HOME_SENSOR_PIN = 26;
  stepper.ConfigurePin(pinConfig);

  /* ================================== Init stepper ================================== */
  // bool result;
  stepper.Initialize();

  /* ============================= Pass stepper into comm ============================= */
  comm.initStepper(0, &stepper);

  /* ==================================== Init LED ==================================== */
  pinMode(LED_PIN, OUTPUT);

  /* =============================== Run stepper thread =============================== */
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
  timer = timerBegin(0, 80, true);                // timer 0, prescalar: 80, UP counting. 1 tick = 1us
  timerAttachInterrupt(timer, &onTimer, true);    // Attach interrupt
  timerAlarmWrite(timer, ISR_TIME_DEFAULT, true); // With 80 prescalar, 1 tick = 1us
  timerAlarmEnable(timer);                        // Enable Timer with interrupt (Alarm Enable)

  Serial.begin(115200);
  comm.init(&Serial);
}

void loop() {
  /* =================================== Read serial ================================== */
  comm.readSerial();

  unsigned long stepDelay = stepper.ComputeTimePeriod();

  /* =================================== Step motor? ================================== */
  portENTER_CRITICAL(&timerMux);
  if (stepDelay > 0) {
    run = true;
    timerAlarmWrite(timer, stepDelay, true);
  } else {
    run = false;
    timerAlarmWrite(timer, ISR_TIME_DEFAULT, true);
  }
  portEXIT_CRITICAL(&timerMux);
}
