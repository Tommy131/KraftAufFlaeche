#pragma once

#include <Arduino.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) //Include SoftwareSerial When using Arduino
    #include <SoftwareSerial.h>
#endif

#include "constants.h"
#include "ToF.h"
#include "PID.h"
#include "MotorControl.h"


class pathControl {

private:
    uint16_t dist; //in mm

    ToF *frontToF;
    ToF *backToF;
    
    ToF frontToF_default;
    ToF backToF_default;


    PID pidDefault = PID(KD_PID, KD_PID, KI_PID, 0, false);
    PID *pid;
    
    MotorControl motorDefault = MotorControl(serial_out);
    MotorControl *motors;

    SerialType *serial_out;

public:
    pathControl(uint16_t _dist, SerialType *_serial_out);
    pathControl(uint16_t _dist, SerialType *_serial_out, MotorControl *_motors);
    pathControl(uint16_t _dist, SerialType *_serial_out, MotorControl *_motors, ToF *_front_ToF, ToF *_backToF, PID *_pid);
    ~pathControl();

    uint16_t getDist() const        { return dist; }
    void setDist(uint16_t _dist)    { dist = constrain(_dist, 0, 2000); }

    /**
     * @brief startups all uses Objects and sensors
     * @return bool value if init was successful ('true' if successful) 
    */
    uint8_t init();

    /**
     * @brief complete what should be done in one loop 
     * @return outputs output codes defined in pathControl.h
    */
    uint8_t loop();

    /**
     * @brief checks ToF sensor for occurrence of a corner in the control surface
     * @return outputs output codes defined in pathControl.h [OUT_CODE_OK, OUT_CODE_CORNER, OUT_CODE_NO_TOF_MESS]
    */
    uint8_t checkForCorner();

    /**
     * @brief executes a shortcut maneuver to shortcut corners 
    */
    void shortcutCorner();



};

