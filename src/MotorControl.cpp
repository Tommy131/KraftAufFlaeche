#include "Arduino.h"

#include "MotorControl.h"
#include "SoftwareSerial.h"
#include "constants.h"


// This namespace is required to use Control table item names
using namespace ControlTableItem;

MotorControl::MotorControl(SoftwareSerial *_softSerial) {
    softSerial = _softSerial;
}

MotorControl::~MotorControl() {

}

void MotorControl::init(){

  //DXL Init
  dxl.begin(baudServos); 
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  for (int i = 1; i < maxServos + 1; i++) {
      dxl.ping(i);
      dxl.reboot(i);
      dxl.torqueOff(i);
      dxl.setOperatingMode(i, operatingMode);
      dxl.torqueOn(i);
  }
      dxl.reboot(1);
      dxl.torqueOff(1);
      dxl.setOperatingMode(1, operatingMode);
      dxl.torqueOn(1);
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

    dxl.setGoalVelocity(MotorMap::ML, _m1, UNIT_RAW);//-float(mot1), UNIT_PERCENT);
    dxl.setGoalVelocity(MotorMap::MR, _m2, UNIT_RAW);//float(mot2), UNIT_PERCENT);
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
