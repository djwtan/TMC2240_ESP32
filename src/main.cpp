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
#define SS0 22
#define SS1 21
#define SS2 17
#define SS3 12
#define ISR_TIME_DEFAULT 500000 // 0.5 seconds

#define CONF_S0 true
#define CONF_S1 true
#define CONF_S2 true
#define CONF_S3 true

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

/* ==================================== Stepper2 ==================================== */
Stepper stepper2(2);
hw_timer_t *timer2 = NULL;                             // Hardware timer1 pointer
portMUX_TYPE timerMux2 = portMUX_INITIALIZER_UNLOCKED; // spinlock
volatile bool run2;

/* ==================================== Stepper3 ==================================== */
Stepper stepper3(3);
hw_timer_t *timer3 = NULL;                             // Hardware timer1 pointer
portMUX_TYPE timerMux3 = portMUX_INITIALIZER_UNLOCKED; // spinlock
volatile bool run3;

/* ====================================== Timer ===================================== */

void IRAM_ATTR onTimer0() {
  portENTER_CRITICAL_ISR(&timerMux0);
  if (run0) {
    stepper0.Run();
  }
  portEXIT_CRITICAL_ISR(&timerMux0);
}

void IRAM_ATTR onTimer1() {
  portENTER_CRITICAL_ISR(&timerMux1);
  if (run1) {
    stepper1.Run();
  }
  portEXIT_CRITICAL_ISR(&timerMux1);
}

void IRAM_ATTR onTimer2() {
  portENTER_CRITICAL_ISR(&timerMux2);
  if (run2) {
    stepper2.Run();
  }
  portEXIT_CRITICAL_ISR(&timerMux2);
}

void IRAM_ATTR onTimer3() {
  portENTER_CRITICAL_ISR(&timerMux3);
  if (run3) {
    stepper3.Run();
  }
  portEXIT_CRITICAL_ISR(&timerMux3);
}

void setup() {
  Serial.begin(115200);

  /* ============================== SPI Comm with driver ============================== */
  SPI.begin(SCK, MISO, MOSI, SS0);

  if (CONF_S0)
    tmc2240spi.RegisterCSPin(0, SS0);
  if (CONF_S1)
    tmc2240spi.RegisterCSPin(1, SS1);
  if (CONF_S2)
    tmc2240spi.RegisterCSPin(2, SS2);
  if (CONF_S3)
    tmc2240spi.RegisterCSPin(3, SS3);

  /* =================================== Motor Pins =================================== */
  // Stepper 0
  if (CONF_S0) {
    PinConfig pinConfig0;
    pinConfig0.EN_PIN = 14;
    pinConfig0.DIR_PIN = 16;
    pinConfig0.STEP_PIN = 33;
    pinConfig0.CS_PIN = SS0;
    pinConfig0.HOME_SENSOR_PIN = 36;
    stepper0.ConfigurePin(pinConfig0);
  }

  // Stepper 1
  if (CONF_S1) {
    PinConfig pinConfig1;
    pinConfig1.EN_PIN = 27;
    pinConfig1.DIR_PIN = 4;
    pinConfig1.STEP_PIN = 25;
    pinConfig1.CS_PIN = SS1;
    pinConfig1.HOME_SENSOR_PIN = 39;
    stepper1.ConfigurePin(pinConfig1);
  }

  // Stepper 2
  if (CONF_S2) {
    PinConfig pinConfig2;
    pinConfig2.EN_PIN = 13;
    pinConfig2.DIR_PIN = 2;
    pinConfig2.STEP_PIN = 26;
    pinConfig2.CS_PIN = SS2;
    pinConfig2.HOME_SENSOR_PIN = 34;
    stepper2.ConfigurePin(pinConfig2);
  }

  // Stepper 3
  if (CONF_S3) {
    PinConfig pinConfig3;
    pinConfig3.EN_PIN = 5;
    pinConfig3.DIR_PIN = 15;
    pinConfig3.STEP_PIN = 32;
    pinConfig3.CS_PIN = SS3;
    pinConfig3.HOME_SENSOR_PIN = 35;
    stepper3.ConfigurePin(pinConfig3);
  }

  /* ================================== Init stepper ================================== */
  if (CONF_S0) {
    stepper0.InitSPI(&tmc2240spi);
    stepper0.Initialize();
  }
  if (CONF_S1) {
    stepper1.InitSPI(&tmc2240spi);
    stepper1.Initialize();
  }
  if (CONF_S2) {
    stepper2.InitSPI(&tmc2240spi);
    stepper2.Initialize();
  }
  if (CONF_S3) {
    stepper3.InitSPI(&tmc2240spi);
    stepper3.Initialize();
  }

  /* ============================= Pass stepper0 into comm ============================= */
  if (CONF_S0)
    comm.initStepper(0, &stepper0);
  if (CONF_S1)
    comm.initStepper(1, &stepper1);
  if (CONF_S2)
    comm.initStepper(2, &stepper2);
  if (CONF_S3)
    comm.initStepper(3, &stepper3);

  /* ================================= Timer Interrupt ================================ */
  if (CONF_S0) {
    timer0 = timerBegin(0, 80, true);                // timer0 0, prescalar: 80, UP counting. 1 tick = 1us
    timerAttachInterrupt(timer0, &onTimer0, true);   // Attach interrupt
    timerAlarmWrite(timer0, ISR_TIME_DEFAULT, true); // With 80 prescalar, 1 tick = 1us
    timerAlarmEnable(timer0);                        // Enable Timer0 with interrupt (Alarm Enable)
  }

  if (CONF_S1) {
    timer1 = timerBegin(1, 80, true);                // timer0 0, prescalar: 80, UP counting. 1 tick = 1us
    timerAttachInterrupt(timer1, &onTimer1, true);   // Attach interrupt
    timerAlarmWrite(timer1, ISR_TIME_DEFAULT, true); // With 80 prescalar, 1 tick = 1us
    timerAlarmEnable(timer1);
  } // Enable Timer0 with interrupt (Alarm Enable)

  if (CONF_S2) {
    timer2 = timerBegin(2, 80, true);                // timer0 0, prescalar: 80, UP counting. 1 tick = 1us
    timerAttachInterrupt(timer2, &onTimer2, true);   // Attach interrupt
    timerAlarmWrite(timer2, ISR_TIME_DEFAULT, true); // With 80 prescalar, 1 tick = 1us
    timerAlarmEnable(timer2);
  } // Enable Timer0 with interrupt (Alarm Enable)

  if (CONF_S3) {
    timer3 = timerBegin(3, 80, true);                // timer0 0, prescalar: 80, UP counting. 1 tick = 1us
    timerAttachInterrupt(timer3, &onTimer3, true);   // Attach interrupt
    timerAlarmWrite(timer3, ISR_TIME_DEFAULT, true); // With 80 prescalar, 1 tick = 1us
    timerAlarmEnable(timer3);
  } // Enable Timer0 with interrupt (Alarm Enable)

  comm.init(&Serial);
}

void loop() {
  /* =================================== Read serial ================================== */
  comm.readSerial();
  unsigned long stepDelay0;
  unsigned long stepDelay1;
  unsigned long stepDelay2;
  unsigned long stepDelay3;

  /* ================================== Compute Step ================================== */
  if (CONF_S0)
    stepDelay0 = stepper0.ComputeTimePeriod();
  if (CONF_S1)
    stepDelay1 = stepper1.ComputeTimePeriod();
  if (CONF_S2)
    stepDelay2 = stepper2.ComputeTimePeriod();
  if (CONF_S3)
    stepDelay3 = stepper3.ComputeTimePeriod();

  /* ==================================== Stepper 0 =================================== */
  if (CONF_S0) {
    portENTER_CRITICAL(&timerMux0);
    if (stepDelay0 > 0) {
      run0 = true;
      timerAlarmWrite(timer0, stepDelay0, true);
    } else {
      run0 = false;
      timerAlarmWrite(timer0, ISR_TIME_DEFAULT, true);
    }
    portEXIT_CRITICAL(&timerMux0);
  }

  /* ==================================== Stepper 1 =================================== */
  if (CONF_S1) {
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

  /* ==================================== Stepper 2 =================================== */
  if (CONF_S2) {
    portENTER_CRITICAL(&timerMux2);
    if (stepDelay2 > 0) {
      run2 = true;
      timerAlarmWrite(timer2, stepDelay2, true);
    } else {
      run2 = false;
      timerAlarmWrite(timer2, ISR_TIME_DEFAULT, true);
    }
    portEXIT_CRITICAL(&timerMux2);
  }

  /* ==================================== Stepper 3 =================================== */
  if (CONF_S3) {
    portENTER_CRITICAL(&timerMux3);
    if (stepDelay3 > 0) {
      run3 = true;
      timerAlarmWrite(timer3, stepDelay3, true);
    } else {
      run3 = false;
      timerAlarmWrite(timer3, ISR_TIME_DEFAULT, true);
    }
    portEXIT_CRITICAL(&timerMux3);
  }
}
