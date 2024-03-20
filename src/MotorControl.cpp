#include "Arduino.h"
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) //Include SoftwareSerial When using Arduino
    #include <SoftwareSerial.h>
#endif

#include "MotorControl.h"
#include "constants.h"


// This namespace is required to use Control table item names
using namespace ControlTableItem;

MotorControl::MotorControl(SerialType *_serialOut) 
    #if defined(ARDUINO_ARCH_ESP32)
            : dxl(DXL_SERIAL, PIN_SW_DXL)
    #endif
                    {
    serialOut = _serialOut;
}

MotorControl::~MotorControl() {

}

uint8_t MotorControl::init(){

    //DXL Init
    dxl.begin(baudServos); 
    DXL_SERIAL.setPins(PIN_RX_DXL, PIN_TX_DXL);
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    for (int i = 1; i < maxServos + 1; i++) {

        if(dxl.ping(i)){
            #ifdef DEBUG_SERVO
                serialOut.printf("Successfully found Servo with ID: %i - FW-Ver: %i - Model: %i\n", i, dxl.readControlTableItem(FIRMWARE_VERSION, i), dxl.getModelNumber(i));
            #endif //DEBUG_SERVO
        } else {
            #ifdef DEBUG_SERVO
                serialOut.printf("Cant find Servo with ID: %i; ABORT START\n", i);
            #endif //DEBUG_SERVO
            return OUT_CODE_ERR_MOTOR;
        }
        dxl.reboot(i);
        dxl.torqueOff(i);
        dxl.setOperatingMode(i, operatingMode);
        dxl.torqueOn(i);
    }
    return OUT_CODE_OK;
}



void MotorControl::normalDrive(int8_t vel, int8_t rot){
    
    vel = constrain(vel, -MAX_PERCENT, MAX_PERCENT);
    rot = constrain(rot, -MAX_PERCENT, MAX_PERCENT);
    
    int32_t mot1 = 0;
    int32_t mot2 = 0;

    if (rot < 0){
        mot1 = vel;
        mot2 = vel  * float(rot/float(-MAX_PERCENT));

    } else if(rot > 0) {

        mot2 = vel;
        mot1 = vel * float(rot/float(MAX_PERCENT));

    } else {
    
        mot2 = vel;
        mot1 = vel;
    }
    if(mot1 > 0){

    }

    bool inv_m1 = false;
    bool inv_m2 = false;
    if(mot1 > 0) inv_m1 = true;
    if(mot2 < 0) inv_m2 = true;
    uint32_t _m1 = map(abs(mot1), 0, MAX_PERCENT, 0, RES_MOTOR);
    uint32_t _m2 = map(abs(mot2), 0, MAX_PERCENT, 0, RES_MOTOR);
    
    _m1 ^= (inv_m1<<RES_MOTOR_BIT);
    _m2 ^= (inv_m2<<RES_MOTOR_BIT);

    dxl.setGoalVelocity(MotorMap::MLeft, _m1, UNIT_RAW);//-float(mot1), UNIT_PERCENT);
    dxl.setGoalVelocity(MotorMap::MRight, _m2, UNIT_RAW);//float(mot2), UNIT_PERCENT);
    /*
    if (trim_motor < 1.0) mot1 = (mot1*trim_motor);
    else if (trim_motor > 1.0) mot2 = (mot2/trim_motor);
    */
}


uint32_t MotorControl::calc_motor_vel(int8_t vel, bool invert_motors) {
    bool invert = false;
    uint32_t motor_speed = map(abs(vel), 0, MAX_PERCENT, 0, RES_MOTOR);
    if (invert_motors && (vel > 0)) {
        invert = true;
    }
    if (!invert_motors && (vel < 0)) {
        invert = true;
    }
    motor_speed ^= (invert<<RES_MOTOR_BIT);
    return motor_speed;
}
