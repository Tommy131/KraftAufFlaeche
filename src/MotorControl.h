#include <Arduino.h>
#pragma once

#include <DynamixelShield.h>
#include <SoftwareSerial.h>
#include "constants.h"
using namespace ControlTableItem;


enum MotorMap{
    ML=1,   //Motor Left with ID
    MR=2   //Motor Right with ID
};


class MotorControl {
private:

    DynamixelShield dxl;

    const float DXL_PROTOCOL_VERSION = 1.0; // important, our motors are 1.0
    SoftwareSerial *softSerial;

    const uint8_t maxServos = 2;
    const uint32_t baudServos = 1000000; // important, our motors are 1M
    const uint8_t operatingMode = OP_VELOCITY;

    const uint8_t trim_motor = 0; 

public:
    MotorControl(SoftwareSerial *_softSerial);
    ~MotorControl();


    void init();

    /**
     * @brief Normal driving mode, with extra features vel=0 equals rotating on the spot
     * @param vel speed of complete robot, without rotation in percent (%) (-100 to 100)
     * @param rot rotation of robot in percent (%) (-100 to 100)
     * 
    */
    void normalDrive(int8_t vel, int8_t rot);

    /**
     * @brief logic for abstracting DXL quirks
     * @param vel velocity, speed ranging from -100 until 100
     * @param invert_motors Set to true if inverted
    */
    uint32_t calc_motor_vel(int8_t vel, bool invertMotors);
};
