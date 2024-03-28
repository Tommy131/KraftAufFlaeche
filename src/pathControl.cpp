#include "pathControl.h"

#include <Arduino.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) //Include SoftwareSerial When using Arduino
    #include <SoftwareSerial.h>
#endif

#include "constants.h"
#include "ToF.h"
#include "PID.h"
#include "MotorControl.h"


pathControl::pathControl(uint16_t _dist, SerialType *_serial_out, MotorControl *_motors, ToF  *_frontToF, ToF *_backToF, PID *_pid){
    dist = constrain(_dist, 0, 2000);

    if(motors == nullptr)       motors = &motorDefault;
    else                        motors = _motors;

    if(frontToF == nullptr)     frontToF = &backToF_default;
    else                        frontToF = _frontToF;

    if(_backToF == nullptr)     backToF = &backToF_default; 
    else                        backToF = _backToF;

    if(_pid == nullptr)         pid = &pidDefault;
    else                        pid = _pid;
}

pathControl::~pathControl(){}


uint8_t pathControl::init(){
    uint8_t ret = 0;
    if(!frontToF->getInit())    ret += !frontToF->init_ToF(PIN_XSHUT_TOF_1, ADDRESS_TOF_2);
    if(!backToF->getInit())     ret += !backToF->init_ToF();

    ret += !motors->init();

    pid->setpoint = dist;

    if(ret == 0) return OUT_CODE_OK;
    return OUT_CODE_ERR;
}


uint8_t pathControl::loop(){
    /**
     * 1. Check Sensors
     * 2. check for special action(shortcutCorner)
     * 3. RUN PID   //TODO: Separate PID for front and back?
     * 4. run Checks (Corner)
     * 5. write to Motors
    */
    
   //Step 1.
    uint16_t ToF_data = 0;
    bool ToF_valid_front = frontToF->read_ToF_mm(ToF_data);
    bool ToF_valid_back  = backToF->read_ToF_mm(ToF_data);                      
    bool ToF_valid_all = (ToF_valid_front && ToF_valid_back);   //indicates if all ToF's are operational //TODO: UNUSED?

    //Step 2.
    switch (driveState) {
    case drive_corner:
        return shortcutCorner();
        break;
    
    default:    //Normal Operation

        //Step 3.
        if(ToF_valid_back) {
            calc_steer = pid->calculations(backToF->getAvgRange());
        } else if (ToF_valid_front) {
            calc_steer = pid->calculations(frontToF->getAvgRange());
        } else {
            motors->normalDrive(MAX_SPEED / 2, -5); // slow down, and try to get closer to the wall
            return OUT_CODE_NO_TOF_MESS;
        }

        //Step 4.
        int16_t ret = checkForCorner(ID_frontTof);
        if(ret == OUT_CODE_NO_TOF_MESS) return ret;
        else if(ret == OUT_CODE_CORNER) driveState = drive_corner;
        else                            driveState = drive_normal;

        //Step 5.
        motors->normalDrive(MAX_SPEED, calc_steer);

        break;
    }

    return OUT_CODE_OK;
}

uint8_t pathControl::checkForCorner(bool frontORback){
    //TODO: More Sophisticated?
    ToF *readToF = backToF;
    ToF *secToF = frontToF; 
    if(!frontORback) {
        readToF = frontToF;
        secToF = backToF;
    }

    readToF->read_ToF_mm();
    if(readToF->getValidRead()){
        if(readToF->getAvgRange() >= CORNER_THR && secToF->getAvgRange() <= CORNER_THR)   {
            return OUT_CODE_CORNER;
        }
        return OUT_CODE_OK;
    } 
    return OUT_CODE_NO_TOF_MESS;
}

uint8_t pathControl::shortcutCorner(){
    motors->normalDrive(MAX_SPEED, -10);    //TODO TUNING!!!!!
    int16_t ret = 0;
    while(ret == OUT_CODE_CORNER) {
        ret = checkForCorner(ID_frontTof);
        if(ret == OUT_CODE_NO_TOF_MESS) return ret;
    }

    ret = 0;
    pid->reset();
    while(ret == OUT_CODE_CORNER) {
        ret = checkForCorner(ID_backToF);
        if(ret == OUT_CODE_NO_TOF_MESS) return ret;

    
        calc_steer = pid->calculations(frontToF->getAvgRange());
        motors->normalDrive(MAX_SPEED, calc_steer);

    }

    driveState = drive_normal;
    return OUT_CODE_OK;
}
