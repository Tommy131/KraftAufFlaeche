#include "pinDef.h"
#include "constants.h"
#include <DynamixelShield.h>
#include "PID.h"
#include "ToF.h"
#include "MotorControl.h"
#include "pathControl.h"

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
PID pidWall(KP_PID, KD_PID, KI_PID, 0, false);

//Init ToF
ToF distWall;

//pathControl path(100, serial_out);

//drive
MotorControl motorControl(serial_out);

#ifndef PIO_UNIT_TESTING // for unit testing
void setup() {

  serial_out->begin(BAUD_SERIAL);
  serial_out->println("Setting up...");
  
  pinMode(PIN_XSHUT_TOF_1, OUTPUT);
  digitalWrite(PIN_XSHUT_TOF_1, HIGH);
  
  //ToF init
  bool ok = distWall.init_ToF();
  if (!ok) {
    serial_out->println("ERROR: init_ToF() failed");
    delay(5000);
    while(true);
  }
  
  //drive init
  //motorControl.init();

  //PID init
  pidWall.setpoint = 100.0;
  
  //path.init();
} // setup

uint16_t dist = 0;

void loop() {
  //serial_out.println("loop()");

  bool val_ok = distWall.read_ToF_mm(dist);
  if (!val_ok) {
    serial_out->println("ERROR: read_ToF_mm()");
  }
  float test_dist = dist;
  float steer = pidWall.calculations(dist);
  //motorControl.normalDrive(100, steer);

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
