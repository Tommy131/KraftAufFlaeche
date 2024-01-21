/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/
#include "pinDef.h"
#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
  #include <SoftwareSerial.h>
  SoftwareSerial soft_serial(7, 8); // DYNAMIXELShield UART RX/TX
  #define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
  #define DEBUG_SERIAL SerialUSB
#else
  #define DEBUG_SERIAL Serial
#endif

const float DXL_PROTOCOL_VERSION = 1.0; // important, our motors are 1.0

DynamixelShield dxl;


// This namespace is required to use Control table item names
using namespace ControlTableItem;

void setup() {

  pinMode(PIN_FORW_DRIVE, INPUT_PULLUP);
  pinMode(PIN_REV_DRIVE, INPUT_PULLUP);
  pinMode(PIN_LEFT_DRIVE, INPUT_PULLUP);
  pinMode(PIN_RIGHT_DRIVE, INPUT_PULLUP);
  
  // For Uno, Nano, Mini, and Mega, use UART port of DYNAMIXEL Shield to debug.
  DEBUG_SERIAL.begin(115200);

  dxl.begin(1000000); // important, our motors are 1M
  // Set Port Protocol Version. This has to match with DYNAMIXEL protocol version.
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int i = 1; i < 3; i++) {
      dxl.ping(i);
      dxl.reboot(i);
      dxl.torqueOff(i);
      dxl.setOperatingMode(i, OP_VELOCITY);
      dxl.torqueOn(i);
  }
} // setup

void loop() {
  const bool forward = !digitalRead(PIN_FORW_DRIVE);
  const bool backward = !digitalRead(PIN_REV_DRIVE);
  const bool left = !digitalRead(PIN_LEFT_DRIVE);
  const bool right = !digitalRead(PIN_RIGHT_DRIVE);

  if (forward) {
    if (left) {
      dxl.setGoalVelocity(1, 40, UNIT_PERCENT);
      dxl.setGoalVelocity(2,-20, UNIT_PERCENT);
    } else if (right) {
      dxl.setGoalVelocity(1, 20, UNIT_PERCENT);
      dxl.setGoalVelocity(2,-40, UNIT_PERCENT);
    } else {
      dxl.setGoalVelocity(1, 40, UNIT_PERCENT);
      dxl.setGoalVelocity(2,-40, UNIT_PERCENT);
    }
  } else if (backward) {
    if (left) {
      dxl.setGoalVelocity(1, -20, UNIT_PERCENT);
      dxl.setGoalVelocity(2, 40, UNIT_PERCENT);
    } else if (right) {
      dxl.setGoalVelocity(1, -40, UNIT_PERCENT);
      dxl.setGoalVelocity(2, 20, UNIT_PERCENT);
    } else {
      dxl.setGoalVelocity(1, -40, UNIT_PERCENT);
      dxl.setGoalVelocity(2, 40, UNIT_PERCENT);
    }
  } else {
    dxl.setGoalVelocity(1, 0, UNIT_PERCENT);
    dxl.setGoalVelocity(2, 0, UNIT_PERCENT);
  }


  // dxl.setGoalVelocity(1, 40, UNIT_PERCENT);
  
  // dxl.setGoalVelocity(2,-40, UNIT_PERCENT);
  delay(1000);
/*
  DEBUG_SERIAL.print("Present PWM(raw) : ");
  DEBUG_SERIAL.println(dxl.getPresentPWM(i));
  delay(1000);

  // Set Goal PWM using percentage (-100.0 [%] ~ 100.0 [%])
  dxl.setGoalPWM(i, -40.8, UNIT_PERCENT);
  delay(1000);
  DEBUG_SERIAL.print("Present PWM(ratio) : ");
  DEBUG_SERIAL.println(dxl.getPresentPWM(i, UNIT_PERCENT));
  delay(1000);
  */
} // loop
