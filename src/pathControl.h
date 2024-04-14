#pragma once

#include <Arduino.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) //Include SoftwareSerial When using Arduino
    #include <SoftwareSerial.h>
#endif

#include "constants.h"
#include "ToF.h"
#include "PID.h"
#include "MotorControl.h"
#include "RuntimeConfig.h"
#include "PidData.h"


class pathControl {

private:

    enum driveModes{
        drive_normal                = 0,
        drive_corner                = 1,
        drive_one_no_new_ToF_data   = 2,
        drive_both_no_new_ToF_data  = 3,
    };

    enum ID_ToFSensor{
        ID_frontTof    = 0,
        ID_backToF     = 1,
    };
    
    SerialType& serial_out;
    
    uint16_t dist; //in mm
    int8_t speed;

    ToF frontToF_default;
    ToF backToF_default;

    ToF *frontToF;
    ToF *backToF;
    


    uint32_t lastMS;
    const uint32_t loopIntervalTime = 50; //ms //interval of loop()

    pid::PID pidDefault = pid::PID(default_pid_trim, 0, false, serial_out);
    pid::PID *pidDist;

    pid::PID pidDefaultAngle = pid::PID(default_pid_angle_trim, 0, false, serial_out);
    pid::PID *pidAngle;
    
    MotorControl motorDefault = MotorControl(serial_out);
    MotorControl *motors;

    uint16_t real_ToF_dist;
    float rotation;

    driveModes driveState  = drive_normal;
    float calc_steer = 0;

    /**
     * @brief checks angle for validity
     * @return true if valid, false otherwise
     * */    
    inline bool checkAngle(float angle);
public:
    pathControl(uint16_t _dist, SerialType& _serial_out, MotorControl *_motors = nullptr, ToF *_front_ToF = nullptr, ToF *_backToF = nullptr, pid::PID *_pidDist = nullptr, pid::PID *_pidAngle = nullptr);
    ~pathControl();

    uint16_t getDist() const        { return dist; }
    void setDist(uint16_t _dist);

    /**
     * @brief startups all uses Objects and sensors
     * @return bool value if init was successful ('true' if successful) 
    */
    bool init();

    /**
     * @brief complete what should be done in one loop with interval defined loopIntervalTime
     * @return outputs output codes defined in enum outputCode (constants.h)
    */
    outputCode loop();

    /**
     * @brief checks ToF sensor for occurrence of a corner in the control surface
     * @param frontORback takes the enum ID_ToFSensor to select the to be evaluated sensor
     * @return outputs output codes defined in enum outputCode (constants.h) [OUT_CODE_OK, OUT_CODE_CORNER, OUT_CODE_NO_TOF_MESS]
    */
    outputCode checkForCorner(ID_ToFSensor frontORback);

    /**
     * @brief executes a shortcut maneuver to shortcut corners 
     * @return outputs output codes defined in enum outputCode (constants.h) [OUT_CODE_OK, OUT_CODE_ERR]
    */
    outputCode shortcutCorner();

    /**
     * @brief set the max. speed for drive
     * @param _speed speed value
    */
    void setSpeed(int8_t _speed);

    /**
     * @brief use both rays of ToF sensors to estimate the current angle the robot has to the wall. 90deg means parallel to the wall.
     * @return returns the angle in RAD 
    */
    float estimateAngle(uint16_t dist1_raw, uint16_t dist2_raw);

    /**
     * @brief use angle and the distance the wall to calculate real distance
     * @return return the distance in mm of perpendicular line
    */
    uint16_t estimateRealDistance(float angle, uint16_t dist_raw);

    /**
     * @brief use estimation functions to calculate the real distance from the wall
     * @return returns distance from wall 
    */
    uint16_t calculateDist(uint16_t dist1_raw, uint16_t dist2_raw);

    /**
     * @brief stops all motors
    */
   void stopMotors();
};

