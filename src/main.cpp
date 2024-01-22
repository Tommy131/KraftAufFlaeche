#include "pinDef.h"
#include <DynamixelShield.h>
#include "PID.h"
#include "ToF.h"
#include "MotorControl.h"

//Init for SoftSerial
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX


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

  //ToF init
  distWall.init_ToF();

  //drive init
  motorControl.init();

  //PID init
  pidWall.setpoint = 200.0;
} // setup

uint16_t dist = 0;

void loop() {
  
  distWall.read_ToF_mm(dist);
  soft_serial.print(dist);  //debug output for testing

  float steer = pidWall.calculations(dist);
  motorControl.normalDrive(100, steer);


} // loop
#endif // PIO_UNIT_TESTING
