#pragma once
#include <Arduino.h>
#include <HardwareSerial.h>

#include <DynamixelShield.h>
//Used Motors: https://emanual.robotis.com/docs/en/dxl/ax/ax-12w

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) //Include SoftwareSerial When using Arduino
    #include <SoftwareSerial.h>
#endif

#include "constants.h"
using namespace ControlTableItem;


enum MotorMap{
    MLeft=2,   //Motor Left with ID
    MRight=1   //Motor Right with ID
};



class MotorControl {
private:

    DynamixelShield dxl;

    const float DXL_PROTOCOL_VERSION = 1.0; // important, our motors are 1.0
    SerialType& serialOut;

    const uint8_t maxServos = 2;
    const uint32_t baudServos = 1000000; // important, our motors are 1M
    const uint8_t operatingMode = OP_VELOCITY;

    const uint8_t trim_motor = 0; 

public:
    MotorControl(SerialType& serialDebug);
    ~MotorControl();


    /**
     * @brief Initialises all motors and checks there Existence
     * @return returns output code either OUT_CODE_OK or OUT_ERR_MOTOR  
    */
    outputCode init();

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
