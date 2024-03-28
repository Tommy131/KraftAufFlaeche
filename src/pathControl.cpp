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
    else return OUT_CODE_ERR;
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
        if(frontToF->getAvgRange() >= CORNER_THR && backToF->getAvgRange() <= CORNER_THR)   {
            return OUT_CODE_CORNER;
        }
        return OUT_CODE_OK;
    } 
    return OUT_CODE_NO_TOF_MESS;
}

void pathControl::shortcutCorner(){
    //TODO
}
