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

    enum driveModes{
        drive_normal                = 0,
        drive_corner                = 1,
        drive_one_no_new_ToF_data   = 2,
        drive_both_no_new_ToF_data  = 2,
    };

    enum ID_ToFSensor{
        ID_frontTof    = 0,
        ID_backToF     = 1,
    };

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

    uint8_t driveState  = drive_normal;
    float calc_steer = 0;
public:
    pathControl(uint16_t _dist, SerialType *_serial_out, MotorControl *_motors = nullptr, ToF *_front_ToF = nullptr, ToF *_backToF = nullptr, PID *_pid = nullptr);
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
     * @return outputs output codes defined in enum outputCode (constants.h)
    */
    uint8_t loop();

    /**
     * @brief checks ToF sensor for occurrence of a corner in the control surface
     * @param frontORback takes the enum ID_ToFSensor to select the to be evaluated sensor
     * @return outputs output codes defined in enum outputCode (constants.h) [OUT_CODE_OK, OUT_CODE_CORNER, OUT_CODE_NO_TOF_MESS]
    */
    uint8_t checkForCorner(bool frontORback);

    /**
     * @brief executes a shortcut maneuver to shortcut corners 
     * @return outputs output codes defined in enum outputCode (constants.h) [OUT_CODE_OK, OUT_CODE_ERR]
    */
    uint8_t shortcutCorner();



};

