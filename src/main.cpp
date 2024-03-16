#include "pinDef.h"
#include <DynamixelShield.h>
#include "PID.h"
#include "ToF.h"
#include "MotorControl.h"
#include "pathControl.h"

//Init for SoftSerial
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(SOFT_DEBUG_RX, SOFT_DEBUG_TX); // DYNAMIXELShield UART RX/TX


//TODO: CONTROL TABLE MAX TORQUE

//Init for PID
PID pidWall(KD_PID, KD_PID, KI_PID, 0, false);

//Init ToF
ToF distWall;
pathControl path(100, &soft_serial);

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
  path.init();
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
