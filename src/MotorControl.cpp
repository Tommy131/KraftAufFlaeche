#include "Arduino.h"
#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560) //Include SoftwareSerial When using Arduino
    #include <SoftwareSerial.h>
#endif

#include "MotorControl.h"
#include "constants.h"
#define DEBUG_SERVO

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
#if defined(ARDUINO_ARCH_ESP32)
    DXL_SERIAL.setPins(PIN_RX_DXL, PIN_TX_DXL);
#endif
    dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

    for (int i = 1; i < maxServos + 1; i++) {

        if(dxl.ping(i)){
            #ifdef DEBUG_SERVO
                serialOut->printf("Successfully found Motor with ID: %i - FW-Ver: %i - Model: %i\n", i, dxl.readControlTableItem(FIRMWARE_VERSION, i), dxl.getModelNumber(i));
            #endif //DEBUG_SERVO
        } else {
            #ifdef DEBUG_SERVO
                serialOut->printf("Cant find Motor with ID: %i; ABORT START\n", i);
            #endif //DEBUG_SERVO
            return OUT_CODE_ERR_MOTOR;
        }
        uint8_t ret = 0;
        ret += !dxl.torqueOff(i);
        ret += !dxl.setOperatingMode(i, operatingMode);
        ret += !dxl.torqueOn(i);
        if(ret != 0) {
            #ifdef DEBUG_SERVO
                serialOut->print("ERROR ON INIT with ID: ");
                serialOut->println(i);
            #endif //DEBUG_SERVO
            return OUT_CODE_ERR_MOTOR;    
        }
    }
    return OUT_CODE_OK;
}



void MotorControl::normalDrive(int8_t vel, int8_t rot){
    
    vel = constrain(vel, -MAX_PERCENT, MAX_PERCENT);
    rot = constrain(rot, -MAX_PERCENT, MAX_PERCENT);
    
    int32_t mot1 = 0; //LEFT
    int32_t mot2 = 0; //RIGHT

    if (rot < 0){       // Turn left
        mot2 = vel;
        mot1 = vel  * (1 - float(rot/float(-MAX_PERCENT)));

    } else if(rot > 0) { // Turn right

        mot1 = vel;
        mot2 = vel * (1 - float(rot/float(MAX_PERCENT)));

    } else {
    
        mot2 = vel;
        mot1 = vel;
    }

    dxl.setGoalVelocity(MotorMap::MLeft, calc_motor_vel(mot1, false), UNIT_RAW);
    dxl.setGoalVelocity(MotorMap::MRight, calc_motor_vel(mot2, true), UNIT_RAW);

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
