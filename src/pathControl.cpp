#include "pathControl.h"

#include <Arduino.h>
#include <SoftwareSerial.h>


#include "constants.h"
#include "ToF.h"
#include "PID.h"
#include "MotorControl.h"

pathControl::pathControl(uint16_t _dist, SoftwareSerial *_serial_out) {
    dist = constrain(_dist, 0, 2000);
    serial_out = _serial_out;
    motors = &motorDefault;
    pid = &pidDefault;
}

pathControl::pathControl(uint16_t _dist, SoftwareSerial *_serial_out, MotorControl *_motors) {
    dist = constrain(_dist, 0, 2000);
    serial_out = _serial_out;
    motors = _motors;
    pid = &pidDefault;
}

pathControl::pathControl(uint16_t _dist, SoftwareSerial *_serial_out, MotorControl *_motors, ToF  *_frontToF, ToF *_backToF, PID *_pid){
    dist = constrain(_dist, 0, 2000);
    serial_out = _serial_out;
    motors = _motors;
    frontToF = _frontToF;
    backToF = _backToF;
    pid = _pid;
}

pathControl::~pathControl(){}


uint8_t pathControl::init(){
    uint8_t ret = 0;
    if(!frontToF->getInit())    ret += !frontToF->init_ToF();
    if(!backToF->getInit())     ret += !backToF->init_ToF();

    motors->init();

    return ret;
}


uint8_t pathControl::loop(){
    //TODO: Implement Loop
    /**
     * 1. Check Sensors
     * 2. check for special action(shortcutCorner)
     * 2. RUN PID
     * 3. run Checks (Corner)
     * 4. write to Motors
    */
    return OUT_CODE_ERR;
}

uint8_t pathControl::checkForCorner(){
    //TODO: More Sophisticated?
    frontToF->read_ToF_mm();
    if(frontToF->getValidRead()){
        if(frontToF->getAvgRange() >= CORNER_THR)   {
            return OUT_CODE_CORNER;
        }
        return OUT_CODE_OK;
    } 
    return OUT_CODE_NO_TOF_MESS;
}

void pathControl::shortcutCorner(){
    //TODO
}
