#include "pathControl.h"

#include <Arduino.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) //Include SoftwareSerial When using Arduino
    #include <SoftwareSerial.h>
#endif

#include "constants.h"
#include "ToF.h"
#include "PID.h"
#include "MotorControl.h"


pathControl::pathControl(uint16_t _dist, SerialType *_serial_out, MotorControl *_motors, ToF  *_frontToF, ToF *_backToF, pid::PID *_pid)
    : speed(MAX_SPEED) 
{
    dist = constrain(_dist, 0, 2000);

    if(_motors == nullptr)      motors = &motorDefault;
    else                        motors = _motors;

    if(_frontToF == nullptr)    frontToF = &backToF_default;
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

    pid->setSetPoint(dist);

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
            motors->normalDrive(speed / 2, -5); // slow down, and try to get closer to the wall
            return OUT_CODE_NO_TOF_MESS;
        }

        //Step 4.
        int16_t ret = checkForCorner(ID_frontTof);
        if(ret == OUT_CODE_NO_TOF_MESS) return ret;
        else if(ret == OUT_CODE_CORNER) driveState = drive_corner;
        else                            driveState = drive_normal;

        //Step 5.
        motors->normalDrive(speed, calc_steer);

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
    motors->normalDrive(speed, -10);    //TODO TUNING!!!!!
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
        motors->normalDrive(speed, calc_steer);

    }

    driveState = drive_normal;
    return OUT_CODE_OK;
}


void pathControl::setSpeed(uint8_t _speed) {
    Serial.print("Updating speed from: ");
    Serial.print(speed, DEC);
    Serial.print("; to ");
    Serial.println(_speed, DEC);
    speed = _speed;
}


void pathControl::setDist(uint16_t _dist) { 
    Serial.print("Updating distance from: ");
    Serial.print(dist, DEC);
    Serial.print("; to ");
    Serial.println(_dist, DEC);
    dist = constrain(_dist, 0, 2000); 
    pid->setSetPoint(dist);
}

float pathControl::estimateAngle(uint16_t dist1_raw, uint16_t dist2_raw){
    int16_t diff = (int16_t) (dist2_raw - dist1_raw);
    return atan2(DISTANCE_TOF, diff);

    if(dist1_raw < dist2_raw) {
        
    } else {
        int16_t diff = (int16_t) (dist1_raw - dist2_raw);
        return (atan2(DISTANCE_TOF, diff) + PI/2);
    }
}

uint16_t pathControl::estimateRealDistance(float angle, uint16_t dist_raw){
    uint16_t dist = (uint16_t) (cos(angle) * dist_raw);
    return dist;
}

uint16_t pathControl::calculateDist(uint16_t dist1_raw, uint16_t dist2_raw){
    float angle = estimateAngle(dist1_raw, dist2_raw);
    if(dist1_raw < dist2_raw){
        return estimateRealDistance(angle, dist1_raw);
    } else {
        return estimateRealDistance(angle, dist2_raw);
    }
}
