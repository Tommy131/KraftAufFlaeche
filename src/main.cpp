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
//PID pidWall(10, 0, 0, 0, 0);
//
////Init ToF
//ToF distWall;
//
//drive
MotorControl motorControl(&soft_serial);

void setup() {
  ////Pin Init
  //pinMode(PIN_FORW_DRIVE, INPUT_PULLUP);
  //pinMode(PIN_REV_DRIVE, INPUT_PULLUP);
  //pinMode(PIN_LEFT_DRIVE, INPUT_PULLUP);
  //pinMode(PIN_RIGHT_DRIVE, INPUT_PULLUP);
  //
  ////SS
  //soft_serial.begin(BAUD_SERIAL);
//
  ////ToF init
  //distWall.init_ToF();
//
  //drive init
  motorControl.init();
  
} // setup
//uint16_t dist = 0;
void loop() {
  
  //distWall.read_ToF_mm(dist);
  //float steer = pidWall.calculations(dist);
  motorControl.normalDrive(100, 0);


} // loop
