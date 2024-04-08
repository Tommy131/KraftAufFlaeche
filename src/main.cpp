#include "pinDef.h"
#include "constants.h"
#include <DynamixelShield.h>
#include "PID.h"
#include "ToF.h"
#include "MotorControl.h"
#include "pathControl.h"
#include "PidData.h"
#include "RuntimeConfig.h"

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) // When using Arduino
  //Init for SoftSerial
  #include <SoftwareSerial.h>
  SoftwareSerial sw_serial_out(SOFT_DEBUG_RX, SOFT_DEBUG_TX); // DYNAMIXELShield UART RX/TX
  #define serial_out (&sw_serial_out)
#elif defined(ARDUINO_ARCH_ESP32)
  HardwareSerial* serial_out = &Serial;
#endif

//TODO: CONTROL TABLE MAX TORQUE

//Init for PID
pid::PID pidWall(default_pid_trim, 0, false);

//Init ToF
ToF distWall;
ToF backWall;

//drive
MotorControl motorControl(serial_out);

pathControl path(DEFAULT_DISTANCE, serial_out, &motorControl, &distWall, &backWall, &pidWall);
#ifndef PIO_UNIT_TESTING // for unit testing
void setup() {

  serial_out->begin(BAUD_SERIAL);
  serial_out->println("Setting up...");
  
  pinMode(PIN_XSHUT_TOF_1, OUTPUT);
  digitalWrite(PIN_XSHUT_TOF_1, HIGH);
  
  //drive init
  //motorControl.init();

  //PID init
  //pidWall.setpoint = 100.0;

  
  setOnTrimeUpdateCallback([](pid::pid_trim_t& upd) {
    pidWall.setTrim(upd);
    pidWall.printTrim(upd);
  });

  setOnSpeedUpdate([](uint8_t speed) {
    path.setSpeed(speed);
  });

  setOnDistanceUpdateCallback([](uint16_t distance) {
    path.setDist(distance);
  });

  setupRuntimeConfig();
  
  path.init();
} // setup

uint16_t dist = 0;

void loop() {
  //serial_out.println("loop()");

  bool val_ok = distWall.read_ToF_mm(dist);
  // if (!val_ok) {
  //   serial_out->println("ERROR: read_ToF_mm()");
  // }
  float steer = pidWall.calculations(dist);
  motorControl.normalDrive(50, steer);
  //path.loop();
#if defined(ARDUINO_ARCH_ESP32)
  serial_out->printf("Dist:%dmm, PID: %f\n", dist, steer);  //debug output for testing
#else
  serial_out->print("Dist: ");
  serial_out->print(dist, DEC);
  serial_out->print(", PID: ");
  serial_out->println(steer, 4);
#endif

  delay(100);
} // loop
#endif // PIO_UNIT_TESTING
