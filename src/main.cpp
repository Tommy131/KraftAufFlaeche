#include "pinDef.h"
#include <DynamixelShield.h>
#include "PID.h"
#include "ToF.h"
#include "MotorControl.h"

//Init for SoftSerial
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(9, 10); // DYNAMIXELShield UART RX/TX


//TODO: CONTROL TABLE MAX TORQUE

//Init for PID
PID pidWall(5, 0, 0, 0, 0);

//Init ToF
ToF distWall;

//drive
MotorControl motorControl(&soft_serial);

#ifndef PIO_UNIT_TESTING // for unit testing
void setup() {
  
  //SS
  soft_serial.begin(BAUD_SERIAL);
  soft_serial.println("Setting up...");

  //ToF init
  const bool ok = distWall.init_ToF();
  if (!ok) {
    soft_serial.println("ERROR: init_ToF() failed");
    delay(5000);
  }

  //drive init
  motorControl.init();

  //PID init
  pidWall.setpoint = 200.0;
} // setup

uint16_t dist = 0;

void loop() {
  soft_serial.println("loop()");

  const bool val_ok = distWall.read_ToF_mm(dist);
  if (!val_ok) {
    soft_serial.println("ERROR: read_ToF_mm()");
  }
  soft_serial.println(dist);  //debug output for testing

  float steer = pidWall.calculations(dist);
  motorControl.normalDrive(100, steer);


} // loop
#endif // PIO_UNIT_TESTING
